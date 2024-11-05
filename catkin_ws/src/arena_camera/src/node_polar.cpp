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
void calcBrightnessImage(cv::Mat& d90, cv::Mat& d45, cv::Mat& d135, cv::Mat&d0, cv::Mat& res);
void splitCvPolarizedImage(cv::Mat& src, cv::Mat& d90, cv::Mat& d45, cv::Mat& d135, cv::Mat&d0);


///////////////////////////////////////////////////////////////////
////////      ArenaCameraNode::grabPolarizedImage()        ////////
///////////////////////////////////////////////////////////////////

bool ArenaCameraNode::grabPolarizedImage()
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  try
  {
    // get image
    GenApi::CStringPtr pTriggerMode = pDevice_->GetNodeMap()->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode))
    {
      bool isTriggerArmed = false;
      do
      {
        isTriggerArmed = Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }
    pImage_ = pDevice_->GetImage(50000);
    pData_ = pImage_->GetData();

    // polarized merge
    int original_height = (int)pImage_->GetHeight();
    int original_width = (int)pImage_->GetWidth();
    cv::Mat res = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);
    getPolarizedImage(res);

    // img_raw_msg_.height = original_height / 2;
    // img_raw_msg_.width = original_width / 2;
    int new_size = img_raw_msg_.height * img_raw_msg_.step;
    img_raw_msg_.data.resize(new_size);

    memcpy(&img_raw_msg_.data[0], res.data, new_size);
    img_raw_msg_.header.stamp = ros::Time::now();
    pDevice_->RequeueBuffer(pImage_);
    return true;
  }
  catch (GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Error while grabbing first image occurred: \r\n" << e.GetDescription());
    return false;
  }
  catch (cv::Exception& e){
    ROS_ERROR_STREAM("Error while grabbing first image occurred: \r\n" << e.what());
    return false;
  }
}



////////        getPolarizedImage()         ////////
void getPolarizedImage(cv::Mat& res) 
{
  int original_height = (int)pImage_->GetHeight();
  int original_width = (int)pImage_->GetWidth();

  // cv::Mat img = cv::Mat(original_height, original_width, CV_8UC1, (void*)pData_);
  cv::Mat img = cv::Mat::zeros(original_height, original_width, CV_8UC1);
  memcpy(img.data, pData_, original_height*original_width);
  
  cv::Mat d90 = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);
  cv::Mat d45 = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);
  cv::Mat d135 = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);
  cv::Mat d0 = cv::Mat::zeros(original_height / 2, original_width / 2, CV_8UC1);

  splitCvPolarizedImage(img, d90, d45, d135, d0);
  calcBrightnessImage(d90, d45, d135, d0, res);

  ROS_INFO_ONCE("original h,w = %d, %d", original_height, original_width);
  ROS_INFO_ONCE("res h,w = %d, %d", res.rows, res.cols);
}



////////        splitCvPolarizedImage()         ////////
void splitCvPolarizedImage(cv::Mat& src, cv::Mat& d90, cv::Mat& d45, cv::Mat& d135, cv::Mat&d0){

  ////////////////
  // d90  | d45 //
  // -----------//
  // d135 | d0  //
  ////////////////

  // asserts
  assert(src.depth() == CV_8UC1);
  assert(src.rows == d90.rows * 2);
  assert(src.rows == d45.rows * 2);
  assert(src.rows == d135.rows * 2);
  assert(src.rows == d0.rows * 2);
  assert(src.cols == d90.cols * 2);
  assert(src.cols == d45.cols * 2);
  assert(src.cols == d135.cols * 2);
  assert(src.cols == d0.cols * 2);
  assert(src.depth() == d90.depth());
  assert(src.depth() == d45.depth());
  assert(src.depth() == d135.depth());
  assert(src.depth() == d0.depth());

  int original_width = src.cols;
  int original_height = src.rows;

  // tmp mats
  cv::Mat tmp0 = cv::Mat::zeros(original_height / 2, original_width, CV_8UC1);
  cv::Mat tmp1 = cv::Mat::zeros(original_height / 2, original_width, CV_8UC1);

  // row split
  for(int i = 0; i < src.rows; i++){
    int i_new = i / 2;
    if (i % 2 == 0){
      src.row(i).copyTo(tmp0.row(i_new));
    }
    else{
      src.row(i).copyTo(tmp1.row(i_new));
    }
    // if(i < src.rows / 2){
    //   src.row(i).copyTo(tmp0.row(i));
    // }
  }

  // col split
  for(int j = 0; j < src.cols; j++){
    int j_new = j / 2;
    if(j % 2 == 0){
      tmp0.col(j).copyTo(d90.col(j_new));
      tmp1.col(j).copyTo(d135.col(j_new));
    }
    else {
      tmp0.col(j).copyTo(d45.col(j_new));
      tmp1.col(j).copyTo(d0.col(j_new));
    }
    // if(j < src.cols / 2){
    //   tmp0.col(j).copyTo(d90.col(j));
    // }
  }
  return;
}


////////        calcBrightnessImage()         ////////
void calcBrightnessImage(cv::Mat& d90, cv::Mat& d45, cv::Mat& d135, cv::Mat&d0, cv::Mat& res){
  // asserts

  assert(res.depth() == CV_8UC1);
  assert(res.rows == d90.rows);
  assert(res.rows == d45.rows);
  assert(res.rows == d135.rows);
  assert(res.rows == d0.rows);
  assert(res.cols == d90.cols);
  assert(res.cols == d45.cols);
  assert(res.cols == d135.cols);
  assert(res.cols == d0.cols);
  assert(res.depth() == d90.depth());
  assert(res.depth() == d45.depth());
  assert(res.depth() == d135.depth());
  assert(res.depth() == d0.depth());

  d90.convertTo(d90, CV_32F, 1, 0);
  d45.convertTo(d45, CV_32F, 1, 0);
  d135.convertTo(d135, CV_32F, 1, 0);
  d0.convertTo(d0, CV_32F, 1, 0);
  res.convertTo(res, CV_32F, 1, 0);
  res = d90 + d45 + d135 + d0;
  res = res / 4;
  res.convertTo(res, CV_8U, 1, 0);
  return;
}



}