/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

// STD
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// ROS
#include <sensor_msgs/RegionOfInterest.h>
#include "boost/multi_array.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include <arena_camera/arena_camera_node.h>
#include <arena_camera/encoding_conversions.h>

using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera
{

Arena::ISystem* pSystem_ = nullptr;
Arena::IDevice* pDevice_ = nullptr;
Arena::IImage* pImage_ = nullptr;
const uint8_t* pData_ = nullptr;

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

float currentGain();
float currentExposure();
float currentGamma();
std::string currentROSEncoding();

///////////////////////////////////////////////////////////////////
////////        ArenaCameraNode::ArenaCameraNode()         ////////
///////////////////////////////////////////////////////////////////
ArenaCameraNode::ArenaCameraNode()
  : nh_("~")
  , arena_camera_parameter_set_()
  // srv init
  , set_binning_srv_(nh_.advertiseService("set_binning", &ArenaCameraNode::setBinningCallback, this))
  , set_roi_srv_(nh_.advertiseService("set_roi", &ArenaCameraNode::setROICallback, this))
  , set_exposure_srv_(nh_.advertiseService("set_exposure", &ArenaCameraNode::setExposureCallback, this))
  , set_gain_srv_(nh_.advertiseService("set_gain", &ArenaCameraNode::setGainCallback, this))
  , set_gamma_srv_(nh_.advertiseService("set_gamma", &ArenaCameraNode::setGammaCallback, this))
  , set_brightness_srv_(nh_.advertiseService("set_brightness", &ArenaCameraNode::setBrightnessCallback, this))
  , set_sleeping_srv_(nh_.advertiseService("set_sleeping", &ArenaCameraNode::setSleepingCallback, this))
  , set_user_output_srvs_()
  // Arena
  , arena_camera_(nullptr)
  // others
  , it_(new image_transport::ImageTransport(nh_))
  , img_raw_pub_(it_->advertiseCamera("image_raw", 1))
  , img_rect_pub_(nullptr)
  , grab_imgs_raw_as_(nh_, "grab_images_raw", boost::bind(&ArenaCameraNode::grabImagesRawActionExecuteCB, this, _1),
                      false)
  , grab_imgs_rect_as_(nullptr)
  , pinhole_model_(nullptr)
  , cv_bridge_img_rect_(nullptr)
  , camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_)) // should this be freed in ~() ?
  , sampling_indices_()
  , brightness_exp_lut_()
  , is_sleeping_(false)
{
  diagnostics_updater_.setHardwareID("none");
  diagnostics_updater_.add("camera_availability", this, &ArenaCameraNode::create_diagnostics);
  diagnostics_updater_.add("intrinsic_calibration", this, &ArenaCameraNode::create_camera_info_diagnostics);
  diagnostics_trigger_ = nh_.createTimer(ros::Duration(2), &ArenaCameraNode::diagnostics_timer_callback_, this);

  init();
  ROS_INFO("init done.");
}


////////    ArenaCameraNode::create_diagnostics()     ////////
void ArenaCameraNode::create_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
}


///// ArenaCameraNode::create_camera_info_diagnostics() //////
void ArenaCameraNode::create_camera_info_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (camera_info_manager_->isCalibrated())
  {
    stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found");
  }
  else
  {
    stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found");
  }
}

////// ArenaCameraNode::diagnostics_timer_callback_() ///////
void ArenaCameraNode::diagnostics_timer_callback_(const ros::TimerEvent&)
{
  diagnostics_updater_.update();
}



///////////////////////////////////////////////////////////////////
////////        ArenaCameraNode::setupRectification()         ////////
///////////////////////////////////////////////////////////////////
void ArenaCameraNode::setupRectification()
{
  if (!img_rect_pub_)
  {
    img_rect_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("detect_image", 1));
  }

  if (!grab_imgs_rect_as_)
  {
    grab_imgs_rect_as_ = new GrabImagesAS(
        nh_, "grab_images_rect", boost::bind(&ArenaCameraNode::grabImagesRectActionExecuteCB, this, _1), false);
    grab_imgs_rect_as_->start();
  }

  if (!pinhole_model_)
  {
    pinhole_model_ = new image_geometry::PinholeCameraModel();
  }

  pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
  if (!cv_bridge_img_rect_)
  {
    cv_bridge_img_rect_ = new cv_bridge::CvImage();
  }
  cv_bridge_img_rect_->header = img_raw_msg_.header;
  cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
}

struct CameraPublisherImpl
{
  image_transport::Publisher image_pub_;
  ros::Publisher info_pub_;
  bool unadvertised_;
  // double constructed_;
};

class CameraPublisherLocal
{
public:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  CameraPublisherImpl* impl_;
};



////////        ArenaCameraNode::getNumSubscribersRect()         ////////
uint32_t ArenaCameraNode::getNumSubscribersRect() const
{
  return camera_info_manager_->isCalibrated() ? img_rect_pub_->getNumSubscribers() : 0;
}


////////        ArenaCameraNode::getNumSubscribers()         ////////
uint32_t ArenaCameraNode::getNumSubscribers() const
{
  return img_raw_pub_.getNumSubscribers() + img_rect_pub_->getNumSubscribers();
}


////////        ArenaCameraNode::getNumSubscribersRaw()         ////////
uint32_t ArenaCameraNode::getNumSubscribersRaw() const
{
  return ((CameraPublisherLocal*)(&img_raw_pub_))->impl_->image_pub_.getNumSubscribers();
}



///////////////////////////////////////////////////////////////////
////////        ArenaCameraNode::spin()         ////////
///////////////////////////////////////////////////////////////////
void ArenaCameraNode::spin()
{
  if (camera_info_manager_->isCalibrated()) {
    ROS_INFO_ONCE("Camera is calibrated");
  }
  else {
    ROS_INFO_ONCE("Camera not calibrated");
  }

  // not connected exception
  if (pDevice_->IsConnected() == false) {
    ROS_ERROR("Arena camera has been removed, trying to reset");
    pSystem_->DestroyDevice(pDevice_);
    pDevice_ = nullptr;
    Arena::CloseSystem(pSystem_);
    pSystem_ = nullptr;
    for (ros::ServiceServer& user_output_srv : set_user_output_srvs_)
    {
      user_output_srv.shutdown();
    }
    ros::Duration(0.5).sleep();  // sleep for half a second
    init();
    return;
  }

  // get image
  if (!isSleeping() && (img_raw_pub_.getNumSubscribers() || getNumSubscribersRect()))
  {
    if (getNumSubscribersRaw() || getNumSubscribersRect()) {
      if(arena_camera_parameter_set_.polarized_merge_) {
        if (!grabPolarizedImage())
        {
          ROS_INFO("Did not get image.");
          return;
        }
      }
      else {
        if (!grabImage())
        {
          ROS_INFO("Did not get image.");
          return;
        }
      }
    }

    if (img_raw_pub_.getNumSubscribers() > 0) {
      // get actual cam_info-object in every frame, because it might have
      // changed due to a 'set_camera_info'-service call
      sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
      cam_info->header.stamp = img_raw_msg_.header.stamp;

      // Publish via image_transport
      img_raw_pub_.publish(img_raw_msg_, *cam_info);
      ROS_INFO_ONCE("Number subscribers received");
    }
    if (getNumSubscribersRect() > 0 && camera_info_manager_->isCalibrated()) {
      cv_bridge_img_rect_->header.stamp = img_raw_msg_.header.stamp;
      assert(pinhole_model_->initialized());
      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_raw_msg_, img_raw_msg_.encoding);
      pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
      pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect_->image);
      img_rect_pub_->publish(*cv_bridge_img_rect_);
      ROS_INFO_ONCE("Number subscribers rect received");
    }
  }
}



///////////////////////////////////////////////////////////////////
////////           ArenaCameraNode::grabImage()            ////////
///////////////////////////////////////////////////////////////////
/*Note: When polarized_merge is true, we use grabPolarImage() instead */
bool ArenaCameraNode::grabImage()
{
  bool hardware_trigger = arena_camera_parameter_set_.hardware_trigger_;
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  try
  {
    if (!hardware_trigger){
      bool isTriggerArmed = false;
      do {
        isTriggerArmed = Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }
    else {
      
    }
    
    pImage_ = pDevice_->GetImage(50000);
    pData_ = pImage_->GetData();
    img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
    memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);
    img_raw_msg_.header.stamp = ros::Time::now();
    pDevice_->RequeueBuffer(pImage_);
    return true;
  }
  catch (GenICam::GenericException& e)
  {
    ROS_WARN_STREAM("An exception in streaming: " << e.GetDescription());
    return false;
  }
}

//////// ArenaCameraNode::grabImagesRawActionExecuteCB() ////////
void ArenaCameraNode::grabImagesRawActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
{
  camera_control_msgs::GrabImagesResult result;
  result = grabImagesRaw(goal, &grab_imgs_raw_as_);
  grab_imgs_raw_as_.setSucceeded(result);
}


///////////////////////////////////////////////////////////////////
//////   ArenaCameraNode::grabImagesRectActionExecuteCB()    //////
///////////////////////////////////////////////////////////////////
void ArenaCameraNode::grabImagesRectActionExecuteCB(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal)
{
  camera_control_msgs::GrabImagesResult result;
  if (!camera_info_manager_->isCalibrated())
  {
    result.success = false;
    grab_imgs_rect_as_->setSucceeded(result);
    return;
  }
  else
  {
    result = grabImagesRaw(goal, std::ref(grab_imgs_rect_as_));
    if (!result.success)
    {
      grab_imgs_rect_as_->setSucceeded(result);
      return;
    }

    for (std::size_t i = 0; i < result.images.size(); ++i)
    {
      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(result.images[i], result.images[i].encoding);
      pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
      cv_bridge::CvImage cv_bridge_img_rect;
      cv_bridge_img_rect.header = result.images[i].header;
      cv_bridge_img_rect.encoding = result.images[i].encoding;
      pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect.image);
      cv_bridge_img_rect.toImageMsg(result.images[i]);
    }
    grab_imgs_rect_as_->setSucceeded(result);
  }
}

///////////////////////////////////////////////////////////////////
////////         ArenaCameraNode::grabImagesRaw()          ////////
///////////////////////////////////////////////////////////////////
camera_control_msgs::GrabImagesResult
ArenaCameraNode::grabImagesRaw(const camera_control_msgs::GrabImagesGoal::ConstPtr& goal, GrabImagesAS* action_server)
{
  camera_control_msgs::GrabImagesResult result;
  camera_control_msgs::GrabImagesFeedback feedback;

  if (goal->exposure_given && goal->exposure_times.empty())
  {
    ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
                     << "'exposure_given' is true, but the 'exposure_times' vector is "
                     << "empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  if (goal->gain_given && goal->gain_values.empty())
  {
    ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
                     << "'gain_given' is true, but the 'gain_values' vector is "
                     << "empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.empty())
  {
    ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
                     << "'brightness_given' is true, but the 'brightness_values' vector"
                     << " is empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.empty())
  {
    ROS_ERROR_STREAM("GrabImagesRaw action server received request and "
                     << "'gamma_given' is true, but the 'gamma_values' vector is "
                     << "empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  std::vector<size_t> candidates;
  candidates.resize(4);  // gain, exposure, gamma, brightness
  candidates.at(0) = goal->gain_given ? goal->gain_values.size() : 0;
  candidates.at(1) = goal->exposure_given ? goal->exposure_times.size() : 0;
  candidates.at(2) = goal->brightness_given ? goal->brightness_values.size() : 0;
  candidates.at(3) = goal->gamma_given ? goal->gamma_values.size() : 0;

  size_t n_images = *std::max_element(candidates.begin(), candidates.end());

  if (goal->exposure_given && goal->exposure_times.size() != n_images)
  {
    ROS_ERROR_STREAM("Size of requested exposure times does not match to "
                     << "the size of the requested vaules of brightness, gain or "
                     << "gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->gain_given && goal->gain_values.size() != n_images)
  {
    ROS_ERROR_STREAM("Size of requested gain values does not match to "
                     << "the size of the requested exposure times or the vaules of "
                     << "brightness or gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.size() != n_images)
  {
    ROS_ERROR_STREAM("Size of requested gamma values does not match to "
                     << "the size of the requested exposure times or the vaules of "
                     << "brightness or gain! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.size() != n_images)
  {
    ROS_ERROR_STREAM("Size of requested brightness values does not match to "
                     << "the size of the requested exposure times or the vaules of gain or "
                     << "gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && !(goal->exposure_auto || goal->gain_auto))
  {
    ROS_ERROR_STREAM("Error while executing the GrabImagesRawAction: A "
                     << "target brightness is provided but Exposure time AND gain are "
                     << "declared as fix, so its impossible to reach the brightness");
    result.success = false;
    return result;
  }

  result.images.resize(n_images);
  result.reached_exposure_times.resize(n_images);
  result.reached_gain_values.resize(n_images);
  result.reached_gamma_values.resize(n_images);
  result.reached_brightness_values.resize(n_images);

  result.success = true;

  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  float previous_exp, previous_gain, previous_gamma;
  if (goal->exposure_given)
  {
    previous_exp = Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime");
  }
  if (goal->gain_given)
  {
    previous_gain = currentGain();
  }
  if (goal->gamma_given)
  {
    previous_gamma = currentGamma();
  }
  if (goal->brightness_given)
  {
    previous_gain = currentGain();
    previous_exp = currentExposure();
  }

  for (std::size_t i = 0; i < n_images; ++i)
  {
    if (goal->exposure_given)
    {
      result.success = setExposure(goal->exposure_times[i], result.reached_exposure_times[i]);
    }
    if (goal->gain_given)
    {
      result.success = setGain(goal->gain_values[i], result.reached_gain_values[i]);
    }
    if (goal->gamma_given)
    {
      result.success = setGamma(goal->gamma_values[i], result.reached_gamma_values[i]);
    }
    if (goal->brightness_given)
    {
      int reached_brightness;
      result.success =
          setBrightness(goal->brightness_values[i], reached_brightness, goal->exposure_auto, goal->gain_auto);
      result.reached_brightness_values[i] = static_cast<float>(reached_brightness);
      //    result.reached_exposure_times[i] = currentExposure();
      //    result.reached_gain_values[i] = currentGain();
    }
    if (!result.success)
    {
      ROS_ERROR_STREAM("Error while setting one of the desired image "
                       << "properties in the GrabImagesRawActionCB. Aborting!");
      break;
    }

    sensor_msgs::Image& img = result.images[i];
    img.encoding = currentROSEncoding();
    img.height = pImage_->GetHeight();
    img.width = pImage_->GetWidth();
    // step = full row length in bytes, img_size = (step * rows),
    // imagePixelDepth already contains the number of channels
    img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);

    img.header.stamp = ros::Time::now();
    img.header.frame_id = cameraFrame();
    feedback.curr_nr_images_taken = i + 1;

    if (action_server != nullptr)
    {
      action_server->publishFeedback(feedback);
    }
  }
  
  if (camera_info_manager_)
  {
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
    result.cam_info = *cam_info;
  }

  // restore previous settings:
  float reached_val;
  if (goal->exposure_given)
  {
    setExposure(previous_exp, reached_val);
  }
  if (goal->gain_given)
  {
    setGain(previous_gain, reached_val);
  }
  if (goal->gamma_given)
  {
    setGamma(previous_gamma, reached_val);
  }
  if (goal->brightness_given)
  {
    setGain(previous_gain, reached_val);
    setExposure(previous_exp, reached_val);
  }
  return result;
}


////////        ArenaCameraNode::setUserOutputCB()         ////////
bool ArenaCameraNode::setUserOutputCB(const int output_id, camera_control_msgs::SetBool::Request& req,
                                      camera_control_msgs::SetBool::Response& res)
{
  //  res.success = arena_camera_->setUserOutput(output_id, req.data);
  return true;
}


////////          ArenaCameraNode::setAutoflash()          ////////
bool ArenaCameraNode::setAutoflash(const int output_id, camera_control_msgs::SetBool::Request& req,
                                   camera_control_msgs::SetBool::Response& res)
{
  ROS_INFO("AutoFlashCB: %i -> %i", output_id, req.data);
  std::map<int, bool> auto_flashs;
  auto_flashs[output_id] = req.data;
  //    arena_camera_->setAutoflash(auto_flashs);
  res.success = true;
  return true;
}


///////////////////////////////////////////////////////////////////
////////      ArenaCameraNode::setSleepingCallback()       ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::setSleepingCallback(camera_control_msgs::SetSleeping::Request& req,
                                          camera_control_msgs::SetSleeping::Response& res)
{
  is_sleeping_ = req.set_sleeping;

  if (is_sleeping_)
  {
    ROS_INFO("Setting Arena Camera Node to sleep...");
  }
  else
  {
    ROS_INFO("Arena Camera Node continues grabbing");
  }

  res.success = true;
  return true;
}


////////          ArenaCameraNode::isSleeping()           ////////
bool ArenaCameraNode::isSleeping()
{
  return is_sleeping_;
}


///////////////////////////////////////////////////////////////////
////////        ArenaCameraNode::~ArenaCameraNode()        ////////
///////////////////////////////////////////////////////////////////
ArenaCameraNode::~ArenaCameraNode()
{
  if (pDevice_ != nullptr) {
    pSystem_->DestroyDevice(pDevice_);
  }

  if (pSystem_ != nullptr) {
    Arena::CloseSystem(pSystem_);
  }

  if (it_) {
    delete it_;
    it_ = nullptr;
  }
  if (grab_imgs_rect_as_) {
    grab_imgs_rect_as_->shutdown();
    delete grab_imgs_rect_as_;
    grab_imgs_rect_as_ = nullptr;
  }

  if (img_rect_pub_) {
    delete img_rect_pub_;
    img_rect_pub_ = nullptr;
  }

  if (cv_bridge_img_rect_) {
    delete cv_bridge_img_rect_;
    cv_bridge_img_rect_ = nullptr;
  }

  if (pinhole_model_) {
    delete pinhole_model_;
    pinhole_model_ = nullptr;
  }
}

}  // namespace arena_camera
