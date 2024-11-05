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


////////        ArenaCameraNode::frameRate()         ////////
const double& ArenaCameraNode::frameRate() const
{
  return arena_camera_parameter_set_.frameRate();
}


////////        ArenaCameraNode::cameraFrame()         ////////
const std::string& ArenaCameraNode::cameraFrame() const
{
  return arena_camera_parameter_set_.cameraFrame();
}



/*********************************************** Encoding ******************************************************/


////////        currentROSEncoding()         ////////
std::string currentROSEncoding()
{
  std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat"));
  std::string ros_encoding("");
  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding))
  {
    std::stringstream ss;
    ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding << "' found! This is bad because this case "
                                                                          "should never occur!";
    throw std::runtime_error(ss.str());
    return "NO_ENCODING";
  }
  return ros_encoding;
}



////////        ArenaCameraNode::setImageEncoding()         ////////
bool ArenaCameraNode::setImageEncoding(const std::string& ros_encoding)
{
  std::string gen_api_encoding;
  bool conversion_found = encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
  if (!conversion_found)
  {
    if (ros_encoding.empty())
    {
      return false;
    }
    else {
      std::string fallbackPixelFormat = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat").c_str();
      ROS_ERROR_STREAM(
            "Can't convert ROS encoding '" << ros_encoding  
            << "' to a corresponding GenAPI encoding! Will use current "
            << "pixel format ( " << fallbackPixelFormat << " ) as fallback!"
        ); 
      return false;
    }
  }
  try {
    GenApi::CEnumerationPtr pPixelFormat = pDevice_->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat))
    {
      Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", gen_api_encoding.c_str());
      if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4"){
        ROS_WARN_STREAM("ROS grabbing image data from 3D pixel format, unable to display in image viewer");
      }
    }
  }
  catch (const GenICam::GenericException& e) {
    ROS_ERROR_STREAM("An exception while setting target image encoding to '" << ros_encoding
                  << "' occurred: " << e.GetDescription());
    return false;
  }
  return true;
}


/*********************************************** Binning ******************************************************/


////////        ArenaCameraNode::setBinningCallback()         ////////
bool ArenaCameraNode::setBinningCallback(camera_control_msgs::SetBinning::Request& req,
                                         camera_control_msgs::SetBinning::Response& res)
{
  size_t reached_binning_x, reached_binning_y;
  bool success_x = setBinningX(req.target_binning_x, reached_binning_x);
  bool success_y = setBinningY(req.target_binning_y, reached_binning_y);
  res.reached_binning_x = static_cast<uint32_t>(reached_binning_x);
  res.reached_binning_y = static_cast<uint32_t>(reached_binning_y);
  res.success = success_x && success_y;
  return true;
}


////////        currentBinningX()         ////////
int64_t currentBinningX()
{
  GenApi::CIntegerPtr BinningHorizontal = pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

  if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal))
  {
    ROS_WARN_STREAM("No binningY value, returning -1");
    return -1;
  }
  else
  {
    float binningXValue = BinningHorizontal->GetValue();
    return binningXValue;
  }
}


////////        currentBinningY()         ////////
int64_t currentBinningY()
{
  GenApi::CIntegerPtr BinningVertical = pDevice_->GetNodeMap()->GetNode("BinningVertical");

  if (!BinningVertical || !GenApi::IsReadable(BinningVertical))
  {
    ROS_WARN_STREAM("No binningY value, returning -1");
    return -1;
  }
  else
  {
    float binningYValue = BinningVertical->GetValue();
    return binningYValue;
  }
}



////////        setBinningXValue()         ////////
bool setBinningXValue(const size_t& target_binning_x, size_t& reached_binning_x)
{
  try
  {
    GenApi::CIntegerPtr pBinningHorizontal = pDevice_->GetNodeMap()->GetNode("BinningHorizontal");
    if (GenApi::IsWritable(pBinningHorizontal))
    {
      size_t binning_x_to_set = target_binning_x;
      if (binning_x_to_set < pBinningHorizontal->GetMin())
      {
        ROS_WARN_STREAM("Desired horizontal binning_x factor(" << binning_x_to_set << ") unreachable! Setting to lower "
                                                               << "limit: " << pBinningHorizontal->GetMin());
        binning_x_to_set = pBinningHorizontal->GetMin();
      }
      else if (binning_x_to_set > pBinningHorizontal->GetMax())
      {
        ROS_WARN_STREAM("Desired horizontal binning_x factor(" << binning_x_to_set << ") unreachable! Setting to upper "
                                                               << "limit: " << pBinningHorizontal->GetMax());
        binning_x_to_set = pBinningHorizontal->GetMax();
      }

      pBinningHorizontal->SetValue(binning_x_to_set);
      reached_binning_x = currentBinningX();
    }
    else
    {
      ROS_WARN_STREAM("Camera does not support binning. Will keep the "
                      << "current settings");
      reached_binning_x = currentBinningX();
    }
  }

  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target horizontal "
                     << "binning_x factor to " << target_binning_x << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}



////////        ArenaCameraNode::setBinningX()         ////////
bool ArenaCameraNode::setBinningX(const size_t& target_binning_x, size_t& reached_binning_x)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setBinningXValue(target_binning_x, reached_binning_x))
  {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok())
    {
      if (setBinningXValue(target_binning_x, reached_binning_x))
      {
        break;
      }
      if (ros::Time::now() > timeout)
      {
        ROS_ERROR_STREAM("Error in setBinningX(): Unable to set target "
                         << "binning_x factor before timeout");
        CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_x = currentBinningX();
        camera_info_manager_->setCameraInfo(*cam_info);
        //   img_raw_msg_.width = pImage_->GetWidth();
        //  step = full row length in bytes, img_size = (step * rows),
        //  imagePixelDepth already contains the number of channels
        //  img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel()
        //  / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}


////////        setBinningYValue()         ////////
bool setBinningYValue(const size_t& target_binning_y, size_t& reached_binning_y)
{
  try
  {
    GenApi::CIntegerPtr pBinningVertical = pDevice_->GetNodeMap()->GetNode("BinningVertical");
    if (GenApi::IsWritable(pBinningVertical))
    {
      size_t binning_y_to_set = target_binning_y;
      if (binning_y_to_set < pBinningVertical->GetMin())
      {
        ROS_WARN_STREAM("Desired horizontal binning_y factor(" << binning_y_to_set << ") unreachable! Setting to lower "
                                                               << "limit: " << pBinningVertical->GetMin());
        binning_y_to_set = pBinningVertical->GetMin();
      }
      else if (binning_y_to_set > pBinningVertical->GetMax())
      {
        ROS_WARN_STREAM("Desired horizontal binning_y factor(" << binning_y_to_set << ") unreachable! Setting to upper "
                                                               << "limit: " << pBinningVertical->GetMax());
        binning_y_to_set = pBinningVertical->GetMax();
      }

      pBinningVertical->SetValue(binning_y_to_set);
      reached_binning_y = currentBinningY();
    }
    else
    {
      ROS_WARN_STREAM("Camera does not support binning. Will keep the "
                      << "current settings");
      reached_binning_y = currentBinningY();
    }
  }

  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target horizontal "
                     << "binning_y factor to " << target_binning_y << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}



////////        ArenaCameraNode::setBinningY()         ////////
bool ArenaCameraNode::setBinningY(const size_t& target_binning_y, size_t& reached_binning_y)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);

  if (!setBinningYValue(target_binning_y, reached_binning_y))
  {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok())
    {
      if (setBinningYValue(target_binning_y, reached_binning_y))
      {
        break;
      }
      if (ros::Time::now() > timeout)
      {
        ROS_ERROR_STREAM("Error in setBinningY(): Unable to set target "
                         << "binning_y factor before timeout");
        CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_y = currentBinningY();
        camera_info_manager_->setCameraInfo(*cam_info);
        img_raw_msg_.width = pImage_->GetWidth();
        //  step = full row length in bytes, img_size = (step * rows),
        //  imagePixelDepth already contains the number of channels
        img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}



/*********************************************** ROI ******************************************************/


////////        ArenaCameraNode::setROICallback()         ////////
bool ArenaCameraNode::setROICallback(camera_control_msgs::SetROI::Request& req,
                                     camera_control_msgs::SetROI::Response& res)
{
  res.success = setROI(req.target_roi, res.reached_roi);
  return true;
}


////////        currentROI()         ////////
sensor_msgs::RegionOfInterest currentROI()
{
  sensor_msgs::RegionOfInterest roi;
  roi.width = pImage_->GetWidth();
  roi.height = pImage_->GetHeight();
  ;
  roi.x_offset = pImage_->GetOffsetX();
  roi.y_offset = pImage_->GetOffsetY();
  return roi;
}



////////        ArenaCameraNode::setROI()         ////////
bool ArenaCameraNode::setROI(const sensor_msgs::RegionOfInterest target_roi, sensor_msgs::RegionOfInterest& reached_roi)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  // TODO: set ROI
  return true;
}




/*********************************************** Gamma ******************************************************/


////////        ArenaCameraNode::setGammaCallback()         ////////
bool ArenaCameraNode::setGammaCallback(camera_control_msgs::SetGamma::Request& req,
                                       camera_control_msgs::SetGamma::Response& res)
{
  res.success = setGamma(req.target_gamma, res.reached_gamma);
  return true;
}



////////        currentGamma()         ////////
float currentGamma()
{
  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

  if (!pGamma || !GenApi::IsReadable(pGamma))
  {
    ROS_WARN_STREAM("No gamma value, returning -1");
    return -1.;
  }
  else
  {
    float gammaValue = pGamma->GetValue();
    return gammaValue;
  }
}



////////        ArenaCameraNode::setGamma()         ////////
bool ArenaCameraNode::setGamma(const float& target_gamma, float& reached_gamma)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  if (ArenaCameraNode::setGammaValue(target_gamma, reached_gamma))
  {
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the gamma value
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok())
    {
      if (ArenaCameraNode::setGammaValue(target_gamma, reached_gamma))
      {
        return true;
      }

      if (ros::Time::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    ROS_ERROR_STREAM("Error in setGamma(): Unable to set target "
                     << "gamma before timeout");
    return false;
  }
  return true;
}


////////        ArenaCameraNode::setGammaValue()         ////////
bool ArenaCameraNode::setGammaValue(const float& target_gamma, float& reached_gamma)
{
  // for GigE cameras you have to enable gamma first

  GenApi::CBooleanPtr pGammaEnable = pDevice_->GetNodeMap()->GetNode("GammaEnable");
  if (GenApi::IsWritable(pGammaEnable))
  {
    pGammaEnable->SetValue(true);
  }

  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
  if (!pGamma || !GenApi::IsWritable(pGamma))
  {
    reached_gamma = -1;
    return true;
  }
  else
  {
    try
    {
      float gamma_to_set = target_gamma;
      if (pGamma->GetMin() > gamma_to_set)
      {
        gamma_to_set = pGamma->GetMin();
        ROS_WARN_STREAM("Desired gamma unreachable! Setting to lower limit: " << gamma_to_set);
      }
      else if (pGamma->GetMax() < gamma_to_set)
      {
        gamma_to_set = pGamma->GetMax();
        ROS_WARN_STREAM("Desired gamma unreachable! Setting to upper limit: " << gamma_to_set);
      }
      pGamma->SetValue(gamma_to_set);
      reached_gamma = currentGamma();
    }
    catch (const GenICam::GenericException& e)
    {
      ROS_ERROR_STREAM(
        "An exception while setting target gamma to " << target_gamma << " occurred: " << e.GetDescription()
      );
      return false;
    }
  }
  return true;
}


}