# Yaml parameter formats

## Informations

+ **camera_frame**: The tf frame under which the images were published
  + e.g. LUCID-TRI050QC-1

+ **device_user_id**: The DeviceUserID of the camera. If empty, the first camera found in the device list will be used
  + e.g. "Cam6BF4", "Cam6C05"

+ **camera_info_url**
  + The CameraInfo URL (Uniform Resource Locator) where the optional intrinsic camera calibration parameters are stored. This URL string will be parsed from the ROS-CameraInfoManager.
  + online: http://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#details
  + local file e.g. "file:///home/adlink/.ros/camera_info/calib_TRI050QC_1.yaml"


## Mode 

+ **image_encoding**
  + The encoding of the pixels -- channel meaning, ordering, size taken from the list of strings in include/sensor_msgs/image_encodings.h. 
  + The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8', 'bayer_gbrg8' and 'bayer_rggb8'. 
  + The Default values are 'mono8' and 'rgb8'
  + For ROS, only "mono8" is supported.
  + e.g. "mono8"

+ **shutter_mode**
  + Mode of camera's shutter.
  + The supported modes are "rolling", "global" and "global_reset"
  + Default value is "" (empty) means default_shutter_mode
  + e.g. "global"

+ **polarized_merge**
  + For Polarized camera, set this value to true and the 4 polarized channels will merge to 1, therefore the image is the same as unpolarized cameras.  
  + Otherwise, the image will be the raw image.
  + e.g. false

+ **hardware_trigger**
  + Use hardware trigger means use signal from wires to trigger image grabbing, such as 'Line 0'.
  + For the mapping of GPIO ports and hardware channel names, see LUCID website below.
  + https://support.thinklucid.com/app-note-using-gpio-on-lucid-cameras/
  + e.g. false



## Brightness related 

see: "Brightness param sets"

+ **exposure**
  + The exposure time in microseconds to be set after opening the camera.
  + At night, recommand value is >30000.0
  + if exposure > 90000.0, there may be motion blur on the image
  + e.g. 5000

+ **gain**
  + The target gain *in percent* of the maximal value the camera supports. For USB-Cameras, the gain is in dB.
  + For GigE-Cameras(e.g. LUCID), it is given in so called 'device specific units', ranges (0.0, 48.0).
  + e.g. 0.7


+ **brightness**
  + The average intensity value of the images. 
  + It depends the exposure time as well as the gain setting. 
    + If 'exposure' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure vaules is many times higher than the gain range). 
    + If 'gain' is provided, the interface will try to reach the desired brightness by only varying the exposure time. 
    + If 'gain' AND 'exposure' are given, it is not possible to reach the brightness, because both are assumed to be fix, therefore this param is dismissed.
    + if 'binning_x' or 'binning_y' is set, The set value will affect the brightness **before** the merge of pixels. Therefore choose smaller values may be wise.
    + Whether 'polarized_merge' is set or not, there won't be difference.
  + e.g. 30

+ **brightness_continuous**
  + Only relevant if 'brightness' is set.
  + The brightness_continuous flag controls the auto brightness function.
    + If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values.
    + If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. 
    + This is only possible for values in the **possible auto range** of the arena API, which is e.g. **[50 - 205]** for acA2500-14um and acA1920-40gm
  + e.g. true

+ **exposure_auto** and **gain_auto**
  + Only relevant, if 'brightness' is set.
  + If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags  must be set.
    + If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise.
    + If only 'exposure_auto' is set, the desired brightness will be reached by adapting the exposure time.
    + If only 'gain_auto' is set, the desired brightness will be reached by adapting the gain.
  + If conflicting occurs, we will choose the fixed mode as default.
    + e.g. if both 'exposure_auto' and 'exposure' are set, we will ignore 'exposure_auto'.
    + If neither 'exposure_auto' and 'exposure' is set, we will set 'exposure' to a default value and set 'exposure_auto' false.
  + e.g.
    + exposure_auto: true
    + gain_auto: true

+ **exposure_auto_dampling**
  + ???????????????????????????????
  + e.g. 1.0

+ **exposure_search_timeout**
  + The timeout is seconds while searching the exposure which is connected to the desired brightness. 
  + If the brightness swings, this parameter has to be decreased.
  + e.g. 1.0

+ **auto_exposure_upper_limit**
  + The maximum exposure_search_timeout upper limit, in us.
  + e.g. 2000000.0




## Others


+ **frame_rate**
  + The desired publisher frame rate if listening to the topics.
  + This parameter can only be set once at startup.
  + Calling the GrabImages-Action can result in a higher framerate.
  + If 'hardware_trigger' is set, this parameter is ignored.
  + e.g. 7.0


+ **gamma**
  + Gamma correction of pixel intensity.
  + Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).
  + range: (?, ?)
  + e.g. 0.5


+ **binning_x** and **binning_y**
  + Binning factor to get downsampled images. 
  + It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels", and reduces the resolution of the output image to (width / binning_x) x (height / binning_y).
  + The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).
  + minimum:1, maximum: 8
  + e.g.
    + binning_x: 2
    + binning_y: 2

+ **gige_mtu_size**
  + The MTU size. Only used for GigE cameras.
  + To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. 
  + A value greater 3000 should be good. But for RaspberryPI, the recommand value is 1500.
  + Default value is 1500.
  + For LUCID cameras, the maximum value is 9000/
  + e.g. 9000


+ **gige_inter_pkg_delay**
  + Only used for GigE cameras.
  + The inter-package delay in ticks to prevent lost frames.
  + For most of GigE-Cameras, a value of 1000 is reasonable.
  + For cameras used on a RaspberryPI, this value should be set to 11772.
  + e.g. 1000



## Brightness Param SETs

```yaml

################# [All autonomous] ##################

brightness: 50

brightness_continuous: true

exposure_auto: true

gain_auto: true

exposure_search_timeout: 0.1

exposure_auto_dampling: 1.0

auto_exposure_upper_limit: 100000

################# [Auto exposure] ##################

brightness: 50

brightness_continuous: true

exposure_auto: true

gain: 5.0

gain_auto: false

exposure_search_timeout: 0.1

exposure_auto_dampling: 1.0

auto_exposure_upper_limit: 100000

################## [Auto gain] ##################

brightness: 50

brightness_continuous: true

exposure_auto: false

exposure: 40000

gain_auto: true

exposure_search_timeout: 0.1

exposure_auto_dampling: 1.0

auto_exposure_upper_limit: 100000

################# [All fixed] ##################

exposure_auto: false

gain_auto: false

exposure: 40000

gain: 5.0
```
