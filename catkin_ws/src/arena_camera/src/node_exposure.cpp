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


////////                  currentGain()                    ////////
float currentGain()
{
  GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

  if (!pGain || !GenApi::IsReadable(pGain))
  {
    ROS_WARN_STREAM("No gain value");
    return -1.;
  }
  else
  {
    float gainValue = pGain->GetValue();
    return gainValue;
  }
}



////////                 currentExposure()                 ////////
float currentExposure()
{
  GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");
  if (!pExposureTime || !GenApi::IsReadable(pExposureTime)) {
    ROS_WARN_STREAM("No exposure time value, returning -1");
    return -1.;
  }
  else
  {
    float exposureValue = pExposureTime->GetValue();
    return exposureValue;
  }
}


////////     disableAllRunningAutoBrightessFunctions()     ////////
void disableAllRunningAutoBrightessFunctions()
{
  // fix bug: pNodeMap_ is nullptr and never assigned !!!!
  GenApi::INodeMap* pNodeMap_ = pDevice_->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "ExposureAuto", "Off");
  Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "GainAuto", "Off");
  ROS_DEBUG("Disabled AutoBrightness.");

  // if (!pExposureAuto || !GenApi::IsWritable(pExposureAuto) || !pGainAuto || !GenApi::IsWritable(pGainAuto))
  // {
  //   ROS_WARN_STREAM("Unable to disable auto brightness");
  //   return;
  // }
  // else
  // {
  //   Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "ExposureAuto", "Off");
  //   Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "GainAuto", "Off");
  // }
}



/*********************************************** Exposure ******************************************************/


////////      ArenaCameraNode::setExposureCallback()      ////////
bool ArenaCameraNode::setExposureCallback(camera_control_msgs::SetExposure::Request& req,
                                          camera_control_msgs::SetExposure::Response& res)
{
  res.success = setExposure(req.target_exposure, res.reached_exposure);
  return true;
}


///////////////////////////////////////////////////////////////////
////////          ArenaCameraNode::setExposure()           ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::setExposure(const float& target_exposure, float& reached_exposure)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
  {
    // success if the delta is smaller then the exposure step
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok())
    {
      if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
      {
        // success if the delta is smaller then the exposure step
        return true;
      }

      if (ros::Time::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    ROS_ERROR_STREAM("Error in setExposure(): Unable to set target"
                     << " exposure before timeout");
    return false;
  }
}



////////       ArenaCameraNode::setExposureValue()         ////////
bool ArenaCameraNode::setExposureValue(const float& target_exposure, float& reached_exposure)
{
  try
  {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");

    GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");

    float exposure_to_set = target_exposure;
    if (exposure_to_set < pExposureTime->GetMin())
    {
      ROS_WARN_STREAM("Desired exposure (" << exposure_to_set << ") "
                                           << "time unreachable! Setting to lower limit: " << pExposureTime->GetMin());
      exposure_to_set = pExposureTime->GetMin();
    }
    else if (exposure_to_set > pExposureTime->GetMax())
    {
      ROS_WARN_STREAM("Desired exposure (" << exposure_to_set << ") "
                                           << "time unreachable! Setting to upper limit: " << pExposureTime->GetMax());
      exposure_to_set = pExposureTime->GetMax();
    }

    pExposureTime->SetValue(exposure_to_set);
    reached_exposure = pExposureTime->GetValue();
    // if ( std::fabs(reached_exposure - exposure_to_set) > exposureStep() )
    // {
    //     // no success if the delta between target and reached exposure
    //     // is greater then the exposure step in ms
    //     return false;
    // }
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target exposure to " << target_exposure
                                                                      << " occurred:" << e.GetDescription());
    return false;
  }
  return true;
}






/************************************************** Gain ******************************************************/


////////        ArenaCameraNode::setGainCallback()         ////////
bool ArenaCameraNode::setGainCallback(camera_control_msgs::SetGain::Request& req,
                                      camera_control_msgs::SetGain::Response& res)
{
  res.success = setGain(req.target_gain, res.reached_gain);
  return true;
}


///////////////////////////////////////////////////////////////////
////////            ArenaCameraNode::setGain()             ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::setGain(const float& target_gain, float& reached_gain)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  // if (!arena_camera_->isReady()) {
  //         ROS_WARN("Error in setGain(): arena_camera_ is not ready!");
  //         return false;
  // }
  //
  if (ArenaCameraNode::setGainValue(target_gain, reached_gain))
  {
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok())
    {
      if (ArenaCameraNode::setGainValue(target_gain, reached_gain))
      {
        return true;
      }

      if (ros::Time::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    ROS_ERROR_STREAM("Error in setGain(): Unable to set target "
                     << "gain before timeout");
    return false;
  }
}


////////          ArenaCameraNode::setGainValue()          ////////
bool ArenaCameraNode::setGainValue(const float& target_gain, float& reached_gain)
{
  try
  {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");

    GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
    float truncated_gain = target_gain;
    if (truncated_gain < pGain->GetMin())
    {
      ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
                                       << "percent out of range [0.0 - 1.0]! Setting to lower "
                                       << "limit: 0.0");
      truncated_gain = pGain->GetMin();
    }
    else if (truncated_gain > pGain->GetMax())
    {
      ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
                                       << "percent out of range [0.0 - 1.0]! Setting to upper "
                                       << "limit: 1.0");
      truncated_gain = pGain->GetMax();
    }

    float gain_to_set = pGain->GetMin() + truncated_gain * (pGain->GetMax() - pGain->GetMin());
    pGain->SetValue(gain_to_set);
    reached_gain = currentGain();
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target gain to " << target_gain
                                                                  << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}







/************************************************ Brightness ****************************************************/


////////    ArenaCameraNode::setBrightnessCallback()       ////////
bool ArenaCameraNode::setBrightnessCallback(camera_control_msgs::SetBrightness::Request& req,
                                            camera_control_msgs::SetBrightness::Response& res)
{
  
  GenApi::INodeMap* pNodeMap_ = pDevice_->GetNodeMap();
  res.success = setBrightness(req.target_brightness, 
            res.reached_brightness, req.exposure_auto, req.gain_auto);
  if (req.brightness_continuous)
  {
    if (req.exposure_auto)
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "ExposureAuto", "Continuous");
    }
    if (req.gain_auto)
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap_, "GainAuto", "Continuous");
    }
  }
  
  // int64_t t_b_v = Arena::GetNodeValue<int64_t>(pNodeMap_, "TargetBrightness");
  // ROS_DEBUG_STREAM("Target Brightness: " << t_b_v);
  res.reached_exposure_time = currentExposure();
  res.reached_gain_value = currentGain();
  res.reached_brightness = calcCurrentBrightness();
  return true;
}



///////////////////////////////////////////////////////////////////
////////         ArenaCameraNode::setBrightness()          ////////
///////////////////////////////////////////////////////////////////
bool ArenaCameraNode::setBrightness(const int& target_brightness, 
      int& reached_brightness, const bool& exposure_auto, const bool& gain_auto)
{
  GenApi::INodeMap* pNodeMap_ = pDevice_->GetNodeMap();
  // target brightness must be less than 255
  int target_brightness_co = std::min(255, target_brightness);

  // smart brightness search
  // initially sets the last rememberd exposure time
  int target_brightness_co_ = std::max(target_brightness_co, 50);
  if (brightness_exp_lut_.at(target_brightness_co_) != 0.0)
  {
    float reached_exp;
    if (!setExposure(brightness_exp_lut_.at(target_brightness_co_), reached_exp))
    {
      ROS_WARN_STREAM("Tried to speed-up exposure search with initial"
                      << " guess, but setting the exposure failed!");
    }
    else
    {
      ROS_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                       << " guess of " << reached_exp);
    }
  }

  // check input auto exposure/gain
  if (!exposure_auto && !gain_auto) {
    ROS_WARN_STREAM("Neither Auto Exposure Time ('exposure_auto') nor Auto "
                    << "Gain ('gain_auto') are enabled! Hence gain and exposure time "
                    << "are assumed to be fix and the target brightness (" << target_brightness_co
                    << ") can not be reached!");
    return false;
  }

  // set brightness
  Arena::SetNodeValue<int64_t>(pNodeMap_, "TargetBrightness", target_brightness);
  return true;
}



////////      ArenaCameraNode::setupSamplingIndices()      ////////
void ArenaCameraNode::setupSamplingIndices(
  std::vector<std::size_t>& indices, std::size_t rows, 
  std::size_t cols, int downsampling_factor)
{
  indices.clear();
  std::size_t min_window_height = static_cast<float>(rows) / static_cast<float>(downsampling_factor);
  cv::Point2i start_pt(0, 0);
  cv::Point2i end_pt(cols, rows);
  // ROS_DEBUG("sample: (cols, rows) = (%d, %d)", cols, rows);
  
  // add the iamge center point only once
  sampling_indices_.push_back(0.5 * rows * cols);
  genSamplingIndicesRec(indices, min_window_height, start_pt, end_pt, cols);
  std::sort(indices.begin(), indices.end());
  return;
}


////////     ArenaCameraNode::genSamplingIndicesRec()      ////////
void ArenaCameraNode::genSamplingIndicesRec(
    std::vector<std::size_t>& indices, const std::size_t& min_window_height,
    const cv::Point2i& s /*start*/, const cv::Point2i& e /*end*/,
    const int image_width)
{
  if (static_cast<std::size_t>(std::abs(e.y - s.y)) <= min_window_height)
  {
    return;  // abort criteria -> shrinked window has the min_col_size
  }
  /*
  * sampled img:      point:                             idx:
  * s 0 0 0 0 0 0  a) [(e.x-s.x)*0.5, (e.y-s.y)*0.5]     a.x*a.y*0.5
  * 0 0 0 d 0 0 0  b) [a.x,           1.5*a.y]           b.y*img_rows+b.x
  * 0 0 0 0 0 0 0  c) [0.5*a.x,       a.y]               c.y*img_rows+c.x
  * 0 c 0 a 0 f 0  d) [a.x,           0.5*a.y]           d.y*img_rows+d.x
  * 0 0 0 0 0 0 0  f) [1.5*a.x,       a.y]               f.y*img_rows+f.x
  * 0 0 0 b 0 0 0
  * 0 0 0 0 0 0 e
  */
  cv::Point2i a, b, c, d, f, delta;
  a = s + 0.5 * (e - s);  // center point
  delta = 0.5 * (e - s);
  b = s + cv::Point2i(delta.x, 1.5 * delta.y);
  c = s + cv::Point2i(0.5 * delta.x, delta.y);
  d = s + cv::Point2i(delta.x, 0.5 * delta.y);
  f = s + cv::Point2i(1.5 * delta.x, delta.y);
  indices.push_back(b.y * image_width + b.x);
  indices.push_back(c.y * image_width + c.x);
  indices.push_back(d.y * image_width + d.x);
  indices.push_back(f.y * image_width + f.x);
  genSamplingIndicesRec(indices, min_window_height, s, a, image_width);
  genSamplingIndicesRec(indices, min_window_height, a, e, image_width);
  genSamplingIndicesRec(indices, min_window_height, cv::Point2i(s.x, a.y), cv::Point2i(a.x, e.y), image_width);
  genSamplingIndicesRec(indices, min_window_height, cv::Point2i(a.x, s.y), cv::Point2i(e.x, a.y), image_width);
  return;
}


////////      ArenaCameraNode::calcCurrentBrightness()     ////////
float ArenaCameraNode::calcCurrentBrightness()
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  if (img_raw_msg_.data.empty())
  {
    return 0.0;
  }

  float sum = 0.0;
  if (sensor_msgs::image_encodings::isMono(img_raw_msg_.encoding))
  {
    // The mean brightness is calculated using a subset of all pixels
    for (const std::size_t& idx : sampling_indices_)
    {
      sum += img_raw_msg_.data.at(idx);
    }
    if (sum > 0.0)
    {
      sum /= static_cast<float>(sampling_indices_.size());
    }
  }
  else
  {
    // The mean brightness is calculated using all pixels and all channels
    sum = std::accumulate(img_raw_msg_.data.begin(), img_raw_msg_.data.end(), 0);
    if (sum > 0.0)
    {
      sum /= static_cast<float>(img_raw_msg_.data.size());
    }
  }
  return sum;
}



/* We DO NOT use this function although it is funcional.  *
  * Instead, we use setBrightness().                       */
bool ArenaCameraNode::manualBrightnessSearch(const int& target_brightness, int& reached_brightness)
{
  // 0. Preparations
  ROS_INFO("manualBrightnessSearch() target_brightness: %d", target_brightness);
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  ros::Time begin = ros::Time::now();  // time measurement for the exposure search
  int target_brightness_co = std::min(255, target_brightness); // target brightness must be less than 255
  ROS_DEBUG_STREAM("target_brightness_co: " << target_brightness_co);

  // 1. preserve the exposure time
  int target_brightness_co_ = std::max(target_brightness_co, 50);
  if (brightness_exp_lut_.at(target_brightness_co_) != 0.0) {
    float reached_exp;
    if (!setExposure(brightness_exp_lut_.at(target_brightness_co_), reached_exp)) {
      ROS_WARN_STREAM("Tried to speed-up exposure search with initial"
                      << " guess, but setting the exposure failed!");
    }
    else {
      ROS_DEBUG_STREAM("Speed-up exposure search with initial exposure"
                       << " guess of " << reached_exp);
    }
  }

  // 2. get actual image & calculate current brightness
  if (!grabImage()) {
    ROS_ERROR("Failed to grab image, can't calculate current brightness!");
    return false;
  }
  float current_brightness = calcCurrentBrightness();
  ROS_DEBUG_STREAM("New brightness request for target brightness " 
    << target_brightness_co << ", current brightness = " << current_brightness);
  if (std::fabs(current_brightness - static_cast<float>(target_brightness_co)) <= 1.0) {
    // target brightness already reached, return true.
    reached_brightness = static_cast<int>(current_brightness);
    ros::Time end = ros::Time::now();
    ROS_DEBUG_STREAM("Brightness reached without exposure search, duration: " << (end - begin).toSec());
    return true;  
  }

  // 3. prepare to search
  disableAllRunningAutoBrightessFunctions(); // deactivate ExposureAuto & AutoGain
  ros::Time start_time = ros::Time::now();
  ros::Time search_timeout = start_time;
  if (target_brightness_co < 205) {
    search_timeout += ros::Duration(arena_camera_parameter_set_.exposure_search_timeout_);
  }
  else {
    // need more time for great exposure values
    search_timeout += ros::Duration(10.0 + arena_camera_parameter_set_.exposure_search_timeout_);
  }
  ROS_DEBUG_STREAM("exposure_search_timeout_: " << arena_camera_parameter_set_.exposure_search_timeout_ << "s");

  // 4. recursive search
  bool is_brightness_reached = false;
  GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");
  float e_low = pExposureTime->GetMin();  // lower exposure time
  float e_high = pExposureTime->GetMax(); // higher exposure time
  float e_tar;                            // target exposure time
  float e_cur = currentExposure();        // current exposure time
  float b_low = 0.0;                      // lower brightness
  float b_high = 255.0;                   // higher brightness
  float b_tar = static_cast<float>(target_brightness_co);     // target brightness
  float& b_cur = current_brightness;      // current brightness
  while (ros::ok())
  {
    // get image & calc brightness
    if (!grabImage()) {
      return false;
    }
    b_cur = calcCurrentBrightness();
    ROS_DEBUG_STREAM("current_brightness: " << b_cur);
    // reset low/high
    if (b_cur < b_tar){
      b_low = b_cur;
      e_low = e_cur;
    }
    else {
      b_high = b_cur;
      e_high = e_cur;
    }
    ROS_DEBUG_STREAM(
      "b_cur: " << b_cur << ", " << "b_tar: " << b_tar << ", " 
      << "b_low: " << b_low << ", " << "b_high: " << b_high 
    );
    ROS_DEBUG_STREAM(
      "e_cur: " << e_cur << ", " << "e_tar: " << e_tar << ", " 
      << "e_low: " << e_low << ", " << "e_high: " << e_high 
    );
    // judge if target reached
    if ((e_high - e_low) < 100.0) {
      ROS_DEBUG("Search has ended."); // error in 0.0001s is acceptable.
      if ((b_high - b_low) < 1.0){
        is_brightness_reached = true;
      }
      break;
    }
    // calc target exposure & set
    e_tar = e_low + (e_high - e_low) * (b_tar - b_low) / (b_high - b_low);
    setExposure(e_tar, e_cur);
    // timeout judgement
    if (ros::Time::now() > search_timeout)
    {
      // cancel all running brightness search by deactivating ExposureAuto
      disableAllRunningAutoBrightessFunctions();
      ROS_WARN_STREAM("Did not reach the target brightness before "
                      << "timeout of " << (search_timeout - start_time).sec << " sec! Stuck at brightness "
                      << current_brightness << " and exposure " << currentExposure() << "us");
      reached_brightness = static_cast<int>(current_brightness);
      return false;
    }
  }

  // 5. search success
  ROS_DEBUG_STREAM("Finally reached brightness: " << current_brightness);
  reached_brightness = static_cast<int>(current_brightness);
  if (std::abs(target_brightness_co - reached_brightness) < 2){
    is_brightness_reached = true;
  }

  // store reached brightness - exposure tuple for next times search
  if (is_brightness_reached)
  {
    if (brightness_exp_lut_.at(reached_brightness) == 0.0) {
      brightness_exp_lut_.at(reached_brightness) = currentExposure();
    }
    else {
      brightness_exp_lut_.at(reached_brightness) += currentExposure();
      brightness_exp_lut_.at(reached_brightness) *= 0.5;
    }
    if (brightness_exp_lut_.at(target_brightness_co) == 0.0) {
      brightness_exp_lut_.at(target_brightness_co) = currentExposure();
    }
    else {
      brightness_exp_lut_.at(target_brightness_co) += currentExposure();
      brightness_exp_lut_.at(target_brightness_co) *= 0.5;
    }
  }
  ros::Time end = ros::Time::now();
  ROS_DEBUG_STREAM("Brightness search duration: " << (end - begin).toSec());
  return is_brightness_reached;
}

}