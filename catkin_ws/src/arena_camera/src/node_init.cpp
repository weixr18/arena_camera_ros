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

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;


void getPolarizedImage(cv::Mat& res);
float currentGain();
float currentExposure();
float currentGamma();
int64_t currentBinningX();
int64_t currentBinningY();
std::string currentROSEncoding();

////////              ArenaCameraNode::init()              ////////
void ArenaCameraNode::init()
{
  arena_camera_parameter_set_.readFromRosParameterServer(nh_);
  // arena_camera_parameter_set_.setCameraInfoURL(nh_, "file://${ROS_HOME}/camera_info/camera.yaml");
  if (!initAndRegister()) {
    ros::shutdown();
    return;
  }
  if (!startGrabbing()) {
    ros::shutdown();
    return;
  }
}


///////////////////////////////////////////////////////////////////
////////                   createDevice()                  ////////
///////////////////////////////////////////////////////////////////
bool createDevice(const std::string& device_user_id_to_open)
{
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  for (auto& info: deviceInfos){
    std::cout << "Device IP address: " << info.IpAddressStr() << std::endl;
    std::cout << "Device MAC address: " << info.MacAddressStr() << std::endl;
  }

  if (deviceInfos.size() == 0)
  {
    Arena::CloseSystem(pSystem_);
    pSystem_ = nullptr;
    return false;
  }
  else
  {
    if (device_user_id_to_open.empty())
    {
      Arena::DeviceInfo info = deviceInfos[0];
      pDevice_ = pSystem_->CreateDevice(deviceInfos[0]);
      return true;
    }
    else
    {
      std::vector<Arena::DeviceInfo>::iterator it;
      bool found_desired_device = false;

      for (it = deviceInfos.begin(); it != deviceInfos.end(); ++it)
      {
        std::string device_user_id_found(it->UserDefinedName());
        std::cout << "Device User id: " << device_user_id_found << std::endl;
        if ((0 == device_user_id_to_open.compare(device_user_id_found)) ||
            (device_user_id_to_open.length() < device_user_id_found.length() &&
             (0 ==
              device_user_id_found.compare(device_user_id_found.length() - device_user_id_to_open.length(),
                                           device_user_id_to_open.length(), device_user_id_to_open))))
        {
          found_desired_device = true;
          break;
        }
      }
      if (found_desired_device)
      {
        ROS_INFO_STREAM("Found the desired camera with DeviceUserID " << device_user_id_to_open << ": ");

        pDevice_ = pSystem_->CreateDevice(*it);
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't find the camera that matches the "
                         << "given DeviceUserID: " << device_user_id_to_open << "! "
                         << "Either the ID is wrong or the cam is not yet connected");
        return false;
      }
    }
  }
}


///////////////////////////////////////////////////////////////////
////////        ArenaCameraNode::initAndRegister()        ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::initAndRegister()
{
  bool device_found_ = false;
  device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());

  if (device_found_ == false)
  {
    // wait and retry until a camera is present
    ros::Time end = ros::Time::now() + ros::Duration(15.0);
    ros::Rate r(0.5);
    while (ros::ok() && device_found_ == false)
    {
      device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());
      if (ros::Time::now() > end)
      {
        ROS_WARN_STREAM("No camera present. Keep waiting ...");
        end = ros::Time::now() + ros::Duration(15.0);
      }
      r.sleep();
      ros::spinOnce();
    }
  }
  else
  {
    ROS_INFO_STREAM("Camera " << arena_camera_parameter_set_.deviceUserID() << " is found!");
  }

  if (!ros::ok())
  {
    return false;
  }

  return true;
}



///////////////////////////////////////////////////////////////////
////////         ArenaCameraNode::startGrabbing()          ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::startGrabbing()
{
  auto  pNodeMap = pDevice_->GetNodeMap();
  bool hardware_trigger = arena_camera_parameter_set_.hardware_trigger_;

  /*--------------------- Arena device prior streaming settings ---------------------*/
  try
  {
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////  Info  ////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // CameraInfo-msg 
    CameraInfo initial_cam_info;
    setupInitialCameraInfo(initial_cam_info);
    camera_info_manager_->setCameraInfo(initial_cam_info);

    if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
        !camera_info_manager_->validateURL(arena_camera_parameter_set_.cameraInfoURL())) {
      ROS_INFO_STREAM("[Init] CameraInfoURL needed for rectification! ROS-Param: "
                      << "'" << nh_.getNamespace() << "/camera_info_url' = '"
                      << arena_camera_parameter_set_.cameraInfoURL() << "' is invalid!");
      ROS_INFO_STREAM("[Init] CameraInfoURL should have following style: "
                       << "'file:///full/path/to/local/file.yaml' or "
                       << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
      ROS_WARN_STREAM("[Init] Will only provide distorted /image_raw images!");
    }
    else {
      // override initial camera info if the url is valid
      if (camera_info_manager_->loadCameraInfo(arena_camera_parameter_set_.cameraInfoURL()))
      {
        setupRectification();
        // set the correct tf frame_id
        CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->header.frame_id = img_raw_msg_.header.frame_id;
        camera_info_manager_->setCameraInfo(*cam_info);
      }
      else {
        ROS_WARN_STREAM("[Init] Will only provide distorted /image_raw images!");
      }
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////  Mode  ////////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // image encoding
    setImageEncoding(arena_camera_parameter_set_.imageEncoding());
    // trigger mode
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "AcquisitionMode", "Continuous");
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
    if (!hardware_trigger) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource", "Software");
      ROS_INFO_STREAM("[Init] Setting *Software* trigger mode.");
    }
    else {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource", "Line0");
      ROS_INFO_STREAM("[Init] Setting *Hardware* trigger mode.");
    }
    // Framerate
    if (!hardware_trigger) {
      auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
      auto currentFrameRate = Arena::GetNodeValue<double>(pNodeMap , "AcquisitionFrameRate");
      auto maximumFrameRate = GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();
      if (cmdlnParamFrameRate >= maximumFrameRate) {
        // requested framerate larger than device max so we trancate it
        arena_camera_parameter_set_.setFrameRate(nh_, maximumFrameRate);
        ROS_WARN("[Init] Desired framerate %.2f Hz (rounded) is higher than max possible. Will limit "
                "framerate device max : %.2f Hz (rounded)", cmdlnParamFrameRate, maximumFrameRate);
      }
      else if (cmdlnParamFrameRate == maximumFrameRate){
        ROS_INFO("[Init] Framerate is %.2f Hz", cmdlnParamFrameRate);
      }
      else if (cmdlnParamFrameRate == -1) {
        arena_camera_parameter_set_.setFrameRate(nh_, maximumFrameRate);
        ROS_WARN("[Init] Framerate is set to device max : %.2f Hz", maximumFrameRate);
      }
      else {
        // requested framerate is valid so we set it to the device
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
        Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate", cmdlnParamFrameRate);
        ROS_INFO("[Init] Framerate is set to: %.2f Hz", cmdlnParamFrameRate);
      }
    }

    ////////////////////////////////////////////////////////////////////////
    /////////////////////////////  Brightness  /////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Exposure
    if (arena_camera_parameter_set_.exposure_auto_) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Continuous");
      ROS_INFO_STREAM("[Init] Settings Exposure to auto/continuous");
    }
    else {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");
      ROS_INFO_STREAM("[Init] Settings Exposure to fixed/off");
    }
    if (arena_camera_parameter_set_.exposure_given_) {
      float reached_exposure;
      if (setExposure(arena_camera_parameter_set_.exposure_, reached_exposure)) {
        ROS_INFO_STREAM(
          "[Init] Setting exposure to " << arena_camera_parameter_set_.exposure_
          << ", reached: " << reached_exposure
        );
      }
    }
    // Gain
    if (arena_camera_parameter_set_.gain_auto_) {
      // gain_auto_ will be already set to false if gain_given_ is true
      // read params () solved the priority between them
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Continuous");
      // todo update parameter on the server
      ROS_INFO_STREAM("[Init] Settings Gain to auto/continuous");
    }
    else {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      // todo update parameter on the server
      ROS_INFO_STREAM("[Init] Settings Gain to fixed/off");
    }
    if (arena_camera_parameter_set_.gain_given_) {
      float reached_gain;
      if (setGain(arena_camera_parameter_set_.gain_, reached_gain)) {
        ROS_INFO_STREAM("[Init] Setting gain to: " << arena_camera_parameter_set_.gain_ << ", reached: " << reached_gain);
      }
    }

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////////  Others  ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // White balance
    GenApi::CBooleanPtr pBalanceWhite = pNodeMap->GetNode("BalanceWhiteEnable");
    if (GenApi::IsWritable(pBalanceWhite)) {
      ROS_INFO("[Init] BalanceWhiteEnable set to true.");
      Arena::SetNodeValue<bool>(pNodeMap, "BalanceWhiteEnable", true);
    }
    else {
      ROS_INFO("[Init] BalanceWhiteEnable IS NOT WRITABLE.");
    }
    // Gamma
    if (arena_camera_parameter_set_.gamma_given_)
    {
      float reached_gamma;
      if (setGamma(arena_camera_parameter_set_.gamma_, reached_gamma))
      {
        ROS_INFO_STREAM(
          "[Init] Setting gamma to " << arena_camera_parameter_set_.gamma_ 
          << ", reached: " << reached_gamma);
      }
    }
    // binning x/y 
    if (arena_camera_parameter_set_.binning_x_given_){
      size_t reached_binning_x;
      if (setBinningX(arena_camera_parameter_set_.binning_x_, reached_binning_x))
      {
        ROS_INFO_STREAM("[Init] Setting horizontal binning_x to " << arena_camera_parameter_set_.binning_x_);
        ROS_WARN_STREAM("[Init] The image width of the camera_info-msg will "
                        << "be adapted, so that the binning_x value in this msg remains 1");
      }
    }
    if (arena_camera_parameter_set_.binning_y_given_) {
      size_t reached_binning_y;
      if (setBinningY(arena_camera_parameter_set_.binning_y_, reached_binning_y))
      {
        ROS_INFO_STREAM("[Init] Setting vertical binning_y to " << arena_camera_parameter_set_.binning_y_);
        ROS_WARN_STREAM("[Init] The image height of the camera_info-msg will "
                        << "be adapted, so that the binning_y value in this msg remains 1");
      }
    }
    // Stream
    Arena::SetNodeValue<GenICam::gcstring>(
      pDevice_->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
    // GeVSCPD
    GenApi::CIntegerPtr pGevSCPD = pNodeMap->GetNode("GevSCPD");
    if (GenApi::IsWritable(pGevSCPD)) {
      int64_t GevSCPD = 5000;
      ROS_INFO("[Init] GevSCPD set to %ld.", GevSCPD);
      Arena::SetNodeValue<int64_t>(pNodeMap, "GevSCPD", GevSCPD);
    }
    else {
      ROS_INFO("[Init] GevSCPD IS NOT WRITABLE.");
    }

    
  }
  catch (GenICam::GenericException& e) {
    ROS_ERROR_STREAM("[Init] Error occurred while setting parameters: \r\n" << e.GetDescription());
    return false;
  }

  /*------------------------------ Trigger First Image ------------------------------*/
  try {
    pDevice_->StartStream(); // start stream
    if (!hardware_trigger) {
      // software trigger
      GenApi::CStringPtr pTriggerMode = pDevice_->GetNodeMap()->GetNode("TriggerMode");
      ROS_INFO_STREAM("[Init] Software trigger, waiting for 1st image... \r\n");
      bool isTriggerArmed = false;
      do {
        isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
    }
    else {
      // hardware trigger
      // GenApi::CStringPtr pTriggerMode = pDevice_->GetNodeMap()->GetNode("TriggerMode");
      ROS_INFO_STREAM("[Init] Hardware trigger, waiting for 1st image... \r\n");
    }
    
    // get image data
    pImage_ = pDevice_->GetImage(50000);
    pData_ = pImage_->GetData();

    // polarized merge & publish to topic
    if(arena_camera_parameter_set_.polarized_merge_) {
      int original_height = (int)pImage_->GetHeight();
      int original_width = (int)pImage_->GetWidth();
      cv::Mat res = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);
      getPolarizedImage(res);
      img_raw_msg_.height = original_height / 2;
      img_raw_msg_.width = original_width / 2;
      img_raw_msg_.step = img_raw_msg_.width * 1;
      int new_size = img_raw_msg_.height * img_raw_msg_.step;
      img_raw_msg_.data.resize(new_size);
      memcpy(&img_raw_msg_.data[0], res.data, new_size);
    }
    else {
      img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
      memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);
    }

  }
  catch (GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("[Init] Error while grabbing first image occurred: \r\n" << e.GetDescription());
    return false;
  }

  /*------------------------------ First Image Publish------------------------------*/
  img_raw_msg_.header.frame_id = arena_camera_parameter_set_.cameraFrame();
  img_raw_msg_.encoding = currentROSEncoding();
  // calculate H and W
  int original_height = (int)pImage_->GetHeight();
  int original_width = (int)pImage_->GetWidth();
  if(arena_camera_parameter_set_.polarized_merge_) {
    img_raw_msg_.height = original_height / 2;
    img_raw_msg_.width = original_width / 2;
  }
  else {
    img_raw_msg_.height = (int)pImage_->GetHeight();
    img_raw_msg_.width = (int)pImage_->GetWidth();
  }
  setupSamplingIndices(sampling_indices_, img_raw_msg_.height, img_raw_msg_.width, 100);
  // step
  /* step = full row length in bytes, img_size = (step * rows), imagePixelDepth already contains the number of channels                                   */
  img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);
  // set camera name
  auto res = camera_info_manager_->setCameraName(std::string(
    Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str()));
  if (!res){
    // valid name contains only alphanumeric signs and '_'
    auto userid = std::string(Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str());
    ROS_WARN_STREAM("[Init] [" << userid << "] is not valid for camera_info_manager");
  }

  /*------------------------------ Init done. ------------------------------*/
  ROS_INFO_STREAM("Startup settings: "
                  << "encoding = '" << currentROSEncoding() << "', "
                  << "binning = [" << currentBinningX() << ", " << currentBinningY() << "], "
                  << "exposure = " << currentExposure() << ", "
                  << "gain = " << currentGain() << ", "
                  << "gamma = " << currentGamma() << ", "
                  << "shutter mode = " << arena_camera_parameter_set_.shutterModeString());

  grab_imgs_raw_as_.start();
  pDevice_->RequeueBuffer(pImage_);
  return true;
}



///////////////////////////////////////////////////////////////////
////////     ArenaCameraNode::setupInitialCameraInfo()     ////////
///////////////////////////////////////////////////////////////////
void ArenaCameraNode::setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg)
{
  std_msgs::Header header;
  header.frame_id = arena_camera_parameter_set_.cameraFrame();
  header.stamp = ros::Time::now();

  // http://www.ros.org/reps/rep-0104.html
  // If the camera is uncalibrated, the matrices D, K, R, P should be left
  // zeroed out. In particular, clients may assume that K[0] == 0.0
  // indicates an uncalibrated camera.
  cam_info_msg.header = header;

  // The image dimensions with which the camera was calibrated. Normally
  // this will be the full camera resolution in pixels. They remain fix, even
  // if binning is applied
  // rows and colums
  cam_info_msg.height = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height");
  cam_info_msg.width = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width");

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficient.
  // Empty D and distortion_model indicate that the CameraInfo cannot be used
  // to rectify points or images, either because the camera is not calibrated
  // or because the rectified image was produced using an unsupported
  // distortion model, e.g. the proprietary one used by Bumblebee cameras
  // [http://www.ros.org/reps/rep-0104.html].
  cam_info_msg.distortion_model = "";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] D.
  cam_info_msg.D = std::vector<double>(5, 0.);

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]  --> 3x3 row-major matrix
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
  // using the focal lengths (fx, fy) and principal point (cx, cy).
  cam_info_msg.K.assign(0.0);

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  cam_info_msg.R.assign(0.0);

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix of the
  // processed (rectified) image. That is, the left 3x3 portion is the normal
  // camera intrinsic matrix for the rectified image. It projects 3D points
  // in the camera coordinate frame to 2D pixel coordinates using the focal
  // lengths (fx', fy') and principal point (cx', cy') - these may differ from
  // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
  // cameras will also have R = the identity and P[1:3,1:3] = K.
  // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
  // position of the optical center of the second camera in the first
  // camera's frame. We assume Tz = 0 so both cameras are in the same
  // stereo image plane. The first camera always has Tx = Ty = 0.
  // For the right (second) camera of a horizontal stereo pair,
  // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
  // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
  // rectified image is given by:
  // [u v w]' = P * [X Y Z 1]'
  //        x = u / w
  //        y = v / w
  //  This holds for both images of a stereo pair.
  cam_info_msg.P.assign(0.0);

  // Binning refers here to any camera setting which combines rectangular
  // neighborhoods of pixels into larger "super-pixels." It reduces the
  // resolution of the output image to (width / binning_x) x (height /
  // binning_y). The default values binning_x = binning_y = 0 is considered the
  // same as binning_x = binning_y = 1 (no subsampling).
  //  cam_info_msg.binning_x = currentBinningX();
  //  cam_info_msg.binning_y = currentBinningY();

  // Region of interest (subwindow of full camera resolution), given in full
  // resolution (unbinned) image coordinates. A particular ROI always denotes
  // the same window of pixels on the camera sensor, regardless of binning
  // settings. The default setting of roi (all values 0) is considered the same
  // as full resolution (roi.width = width, roi.height = height).

  // todo? do these has ti be set via Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX"); or so ?
  cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}



}