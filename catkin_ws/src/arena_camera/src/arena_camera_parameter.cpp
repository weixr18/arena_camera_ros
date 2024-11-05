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
#include <ios> // std::boolalpha 

// ROS
#include <sensor_msgs/image_encodings.h>

// Arena node
#include <arena_camera/arena_camera_parameter.h>

namespace arena_camera
{
ArenaCameraParameter::ArenaCameraParameter()
  : camera_frame_("arena_camera")
  , device_user_id_("")
  , camera_info_url_("")

  ///////////////////////////////
  //////// mode settings ////////
  ///////////////////////////////
  , hardware_trigger_(false)
  , polarized_merge_(false)
  , shutter_mode_(SM_DEFAULT)
  , auto_flash_(false)

  ///////////////////////////////
  // exposure related settings //
  ///////////////////////////////
  , exposure_(10000.0)
  , exposure_given_(true)
  , gain_(0.5)
  , gain_given_(true)
  , exposure_search_timeout_(5.)
  , auto_exp_upper_lim_(0.0)
  , brightness_(100)
  , brightness_given_(false)
  , brightness_continuous_(false)
  , exposure_auto_(false)
  , gain_auto_(false)

  ///////////////////////////////
  /////// other settings ////////
  ///////////////////////////////
  , frame_rate_(5.0)
  , image_encoding_("")
  , image_encoding_given_(false)
  , binning_x_(1)
  , binning_y_(1)
  , binning_x_given_(false)
  , binning_y_given_(false)
  , downsampling_factor_exp_search_(1)
  , gamma_(1.0)
  , gamma_given_(false)
  , mtu_size_(3000)
  , inter_pkg_delay_(1000)

{
}

ArenaCameraParameter::~ArenaCameraParameter()
{
}




/****************************************** Read parameters ********************************************/


void ArenaCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
{
  nh.param<std::string>("camera_frame", camera_frame_, "arena_camera");
  nh.param<std::string>("device_user_id", device_user_id_, "");
  nh.param<std::string>("camera_info_url", camera_info_url_, "");
  if (nh.hasParam("camera_info_url"))
  {
    nh.getParam("camera_info_url", camera_info_url_);
  }
  readModeParameters(nh);
  readBrightnessParameters(nh);
  readOtherParameters(nh);
  validateParameterSet(nh);
  return;
}


void ArenaCameraParameter::readBrightnessParameters(const ros::NodeHandle& nh) {

  //////////////////////////////////
  /// exposure: auto or fixed? /////
  //////////////////////////////////
  /*
    exposure_given | exposure_auto_       | action           |
                   |                      |                  |
    ---------------|----------------------|------------------|
    1     F                 F             | Use default fixed exposure value. |
          
    2     F                 T             | Normal auto exposure |

    3     T                 F             | Normal fixed exposure |

    4     T                 F             | Abnormal, given fixed value overrides "exposure_auto". |
  */
  exposure_given_ = nh.hasParam("exposure");
  bool exposure_auto_given = nh.hasParam("exposure_auto");
  if (!exposure_given_) {
    if (!exposure_auto_) {
      // 1 FF
      ROS_DEBUG_STREAM(
        "[Bright] exposure is NOT given and exposure_auto False, will use default fixed exposure value" << exposure_
      );
      exposure_auto_ = false;
    }
    else {
      // 2 FT
      nh.getParam("exposure_auto", exposure_auto_);
      ROS_DEBUG_STREAM("[Bright] exposure_auto_ is given and has value " << exposure_auto_);
    }
  }
  else { // exposure_given_ True
    if (!exposure_auto_) {
      // 3 TF
      nh.getParam("exposure", exposure_);
      ROS_DEBUG_STREAM("[Bright] exposure is given and has value " << exposure_);
    } 
    else {
      // 4 TT
      ROS_DEBUG_STREAM("[Bright] exposure is given and exposure_auto True, will set exposure_auto to False.");
      exposure_auto_ = false;
    }
  }


  //////////////////////////////////
  ///// gain: auto or fixed? ///////
  //////////////////////////////////
  /*
    gain_given | gain_auto_       | action           |
                   |                      |                  |
    ---------------|----------------------|------------------|
    1     F                 F             | Use default fixed gain value. |
          
    2     F                 T             | Normal auto gain |

    3     T                 F             | Normal fixed gain |

    4     T                 F             | Abnormal, given fixed value overrides "gain_auto". |
  */
  gain_given_ = nh.hasParam("gain");
  bool gain_auto_given = nh.hasParam("gain_auto");
  if (!gain_given_) {
    if (!gain_auto_) {
      // 1 FF
      ROS_DEBUG_STREAM(
        "[Bright] gain is NOT given and gain_auto False, will use default fixed gain value" << gain_
      );
      gain_auto_ = false;
    }
    else {
      // 2 FT
      nh.getParam("gain_auto", gain_auto_);
      ROS_DEBUG_STREAM("[Bright] gain_auto_ is given and has value " << gain_auto_);
    }
  }
  else { // gain_given_ True
    if (!gain_auto_) {
      // 3 TF
      nh.getParam("gain", gain_);
      ROS_DEBUG_STREAM("[Bright] gain is given and has value " << gain_);
    } 
    else {
      // 4 TT
      ROS_DEBUG_STREAM( "[Bright] gain is given and gain_auto True, will set gain_auto to False.");
      gain_auto_ = false;
    }
  }

  ////////////////////////////////////
  /// brightness: auto or fixed? /////
  ////////////////////////////////////
  brightness_given_ = nh.hasParam("brightness");
  auto ignoreBrightness = brightness_given_ && gain_given_ && exposure_given_;
  if (ignoreBrightness)
  {
    ROS_WARN_STREAM("[Bright] Gain ('gain') and Exposure Time ('exposure') "
                    << "are given as startup ros-parameter and hence assumed to be "
                    << "fixed! The desired brightness (" << brightness_ << ") can't "
                    << "be reached, so the brightness will be ignored.");
    brightness_given_ = false;
  }
  else {
    // auto brightness
    if (!brightness_given_) {
      ROS_DEBUG_STREAM(
        "[Bright] Auto brightness mode, but brightness is NOT given. " 
        << "Will use default brightness value " << brightness_);
    }
    else {
      nh.getParam("brightness", brightness_);
      ROS_DEBUG_STREAM("[Bright] brightness is given and has value " << brightness_);
    }

    if (nh.hasParam("brightness_continuous")) {
      nh.getParam("brightness_continuous", brightness_continuous_);
      ROS_DEBUG_STREAM("[Bright] auto brightness will be continuous.");
    }
    else {
      ROS_DEBUG_STREAM("[Bright] CAUTION: Brightness value will be reached only ONCE.");
    }

  }


  /////// exposure search ///////
  if (nh.hasParam("exposure_search_timeout")) {
    nh.getParam("exposure_search_timeout", exposure_search_timeout_);
    ROS_DEBUG_STREAM("[Bright] exposure_search_timeout is given and has value " << exposure_search_timeout_);
  }
  else{
    ROS_DEBUG_STREAM("[Bright] use default exposure_search_timeout: " << 5.);
    nh.param<double>("exposure_search_timeout", exposure_search_timeout_, (float)5.);
  }
  nh.param<int>("downsampling_factor_exposure_search", downsampling_factor_exp_search_, 20);
  nh.param<double>("auto_exposure_upper_limit", auto_exp_upper_lim_, 10000000.);
}





void ArenaCameraParameter::readModeParameters(const ros::NodeHandle& nh) {

  // image encoding
  image_encoding_given_ = nh.hasParam("image_encoding");
  if (nh.hasParam("image_encoding"))
  {
    std::string encoding;
    nh.getParam("image_encoding", encoding);
    image_encoding_ = encoding;
    /*if (!encoding.empty() &&
    	!sensor_msgs::image_encodings::isMono(encoding) &&
    	!sensor_msgs::image_encodings::isColor(encoding) &&
    	!sensor_msgs::image_encodings::isBayer(encoding) &&
    	encoding != sensor_msgs::image_encodings::YUV422){
    	ROS_WARN_STREAM(
        "Desired image encoding parameter: '" << encoding
    		<< "' is not part of the 'sensor_msgs/image_encodings.h' list!"
    		<< " Will not set encoding"
      );
    	encoding = std::string("");
    }*/
    
  }

  // shutter mode
  std::string shutter_param_string;
  nh.param<std::string>("shutter_mode", shutter_param_string, "");
  if (shutter_param_string == "rolling") {
    shutter_mode_ = SM_ROLLING;
  }
  else if (shutter_param_string == "global") {
    shutter_mode_ = SM_GLOBAL;
  }
  else if (shutter_param_string == "global_reset") {
    shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
  }
  else {
    shutter_mode_ = SM_DEFAULT;
  }

  // auto flash
  nh.param<bool>("auto_flash", auto_flash_, false);
  nh.param<bool>("auto_flash_line_2", auto_flash_line_2_, true);
  nh.param<bool>("auto_flash_line_3", auto_flash_line_3_, true);
  ROS_WARN("Autoflash: %i, line2: %i , line3: %i ", auto_flash_, auto_flash_line_2_, auto_flash_line_3_);

  // hardware trigger
  if (nh.hasParam("hardware_trigger"))
  {
    nh.getParam("hardware_trigger", hardware_trigger_);
    ROS_DEBUG_STREAM("hardware_trigger is given and has value " << hardware_trigger_);
  }
}





void ArenaCameraParameter::readOtherParameters(const ros::NodeHandle& nh){
  // frame rate
  if (nh.hasParam("frame_rate")) {
    nh.getParam("frame_rate", frame_rate_);
    ROS_DEBUG_STREAM("frame_rate is given and has value " << frame_rate_);
  }
  // binning 
  binning_x_given_ = nh.hasParam("binning_x");
  if (binning_x_given_) {
    int binning_x;
    nh.getParam("binning_x", binning_x);
    ROS_DEBUG_STREAM( "binning x is given and has value " << binning_x);
    if (binning_x > 32 || binning_x < 0)
    {
      ROS_WARN_STREAM("Desired horizontal binning_x factor not in valid "
                      << "range! Binning x = " << binning_x << ". Will reset it to "
                      << "default value (1)");
      binning_x_given_ = false;
    }
    else
    {
      binning_x_ = static_cast<size_t>(binning_x);
    }
  }
  binning_y_given_ = nh.hasParam("binning_y");
  if (binning_y_given_) {
    int binning_y;
    nh.getParam("binning_y", binning_y);
    ROS_DEBUG_STREAM("binning y is given and has value " << binning_y);
    if (binning_y > 32 || binning_y < 0)
    {
      ROS_WARN_STREAM("Desired vertical binning_y factor not in valid "
                      << "range! Binning y = " << binning_y << ". Will reset it to "
                      << "default value (1)");
      binning_y_given_ = false;
    }
    else
    {
      binning_y_ = static_cast<size_t>(binning_y);
    }
  } 

  // gamma
  gamma_given_ = nh.hasParam("gamma");
  if (gamma_given_) {
    nh.getParam("gamma", gamma_);
    ROS_DEBUG_STREAM("gamma is given and has value " << gamma_);
  }

  // mtu size
  if (nh.hasParam("gige_mtu_size")) {
    nh.getParam("gige_mtu_size", mtu_size_);
  }
  // inter_pkg_delay
  if (nh.hasParam("gige_inter_pkg_delay")) {
    nh.getParam("gige_inter_pkg_delay", inter_pkg_delay_);
  }
}






/****************************************** Others ********************************************/





void ArenaCameraParameter::adaptDeviceUserId(const ros::NodeHandle& nh, const std::string& device_user_id)
{
  device_user_id_ = device_user_id;
  nh.setParam("device_user_id", device_user_id_);
}

void ArenaCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
{
  if (!device_user_id_.empty())
  {
    ROS_INFO_STREAM("Trying to open the following camera: " << device_user_id_.c_str());
  }
  else
  {
    ROS_INFO_STREAM("No Device User ID set -> Will open the camera device "
                    << "found first");
  }

  if (frame_rate_ < 0 && frame_rate_ != -1)
  {
    ROS_WARN_STREAM("Unexpected frame rate (" << frame_rate_ << "). Will "
                                              << "reset it to default value which is 5 Hz");
    frame_rate_ = 5.0;
    nh.setParam("frame_rate", frame_rate_);
  }

  if (exposure_given_ && (exposure_ <= 0.0 || exposure_ > 1e7))
  {
    ROS_WARN_STREAM("Desired exposure measured in microseconds not in "
                    << "valid range! Exposure time = " << exposure_ << ". Will "
                    << "reset it to default value!");
    exposure_given_ = false;
  }

  if (gain_given_ && (gain_ < 0.0 || gain_ > 1.0))
  {
    ROS_WARN_STREAM("Desired gain (in percent) not in allowed range! "
                    << "Gain = " << gain_ << ". Will reset it to default value!");
    gain_given_ = false;
  }

  if (brightness_given_ && (brightness_ < 0.0 || brightness_ > 255))
  {
    ROS_WARN_STREAM("Desired brightness not in allowed range [0 - 255]! "
                    << "Brightness = " << brightness_ << ". Will reset it to "
                    << "default value!");
    brightness_given_ = false;
  }

  if (exposure_search_timeout_ < 5.)
  {
    ROS_WARN_STREAM("Low timeout for exposure search detected! Exposure "
                    << "search may fail.");
  }
  return;
}


const std::string& ArenaCameraParameter::deviceUserID() const
{
  return device_user_id_;
}

std::string ArenaCameraParameter::shutterModeString() const
{
  if (shutter_mode_ == SM_ROLLING)
  {
    return "rolling";
  }
  else if (shutter_mode_ == SM_GLOBAL)
  {
    return "global";
  }
  else if (shutter_mode_ == SM_GLOBAL_RESET_RELEASE)
  {
    return "global_reset";
  }
  else
  {
    return "default_shutter_mode";
  }
}

const std::string& ArenaCameraParameter::imageEncoding() const
{
  return image_encoding_;
}

const std::string& ArenaCameraParameter::cameraFrame() const
{
  return camera_frame_;
}

const double& ArenaCameraParameter::frameRate() const
{
  return frame_rate_;
}

void ArenaCameraParameter::setFrameRate(const ros::NodeHandle& nh, const double& frame_rate)
{
  frame_rate_ = frame_rate;
  nh.setParam("frame_rate", frame_rate_);
}

const std::string& ArenaCameraParameter::cameraInfoURL() const
{
  return camera_info_url_;
}

void ArenaCameraParameter::setCameraInfoURL(const ros::NodeHandle& nh, const std::string& camera_info_url)
{
  camera_info_url_ = camera_info_url;
  nh.setParam("camera_info_url", camera_info_url_);
}

}  // namespace arena_camera
