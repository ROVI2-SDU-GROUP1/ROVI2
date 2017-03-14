#include <caros/pointgrey_camera.h>

using namespace FlyCapture2;

namespace caros
{
PointGreyCamera::PointGreyCamera() : CameraInterface()
{
  serial_ = 0;
  capture_running_ = false;
}

PointGreyCamera::PointGreyCamera(unsigned int serial) : CameraInterface(serial), bus_mgr_(), cam_()
{
  serial_ = fw_serial_;
  capture_running_ = false;
}

PointGreyCamera::~PointGreyCamera()
{
}

bool PointGreyCamera::init()
{
  ROS_INFO_STREAM("Connecting to camera serial: " << serial_);
  ROS_INFO_STREAM("Actually connecting to camera.");
  connect();
  if (cam_.IsConnected())
    return true;
  else
    return false;
}

void PointGreyCamera::connect()
{
  if (!cam_.IsConnected())
  {
    Error error;
    PGRGuid guid;      // GUIDS are NOT persistent accross executions, do not store them.
    if (serial_ != 0)  // If we have a specific camera to connect to.
    {
      error = bus_mgr_.GetCameraFromSerialNumber(serial_, &guid);
      std::stringstream serial_string;
      serial_string << serial_;
      std::string msg = "PointGreyCamera::connect Could not find camera with serial number: " + serial_string.str() +
                        ". Is that camera plugged in?";
      PointGreyCamera::handleError(msg, error);
    }
    else  // Connect to any camera (the first)
    {
      error = bus_mgr_.GetCameraFromIndex(0, &guid);
      PointGreyCamera::handleError("PointGreyCamera::connect Failed to get first connected camera", error);
    }

    FlyCapture2::InterfaceType ifType;
    error = bus_mgr_.GetInterfaceTypeFromGuid(&guid, &ifType);
    PointGreyCamera::handleError("PointGreyCamera::connect Failed to get interface style of camera", error);

    // Not supported yet
    /*if (ifType == FlyCapture2::INTERFACE_GIGE)
    {
      // Set packet size:
      if (auto_packet_size_)
        setupGigEPacketSize(guid);
      else
        setupGigEPacketSize(guid, packet_size_);

      // Set packet delay:
      setupGigEPacketDelay(guid, packet_delay_);
    }*/

    error = cam_.Connect(&guid);
    PointGreyCamera::handleError("PointGreyCamera::connect Failed to connect to camera", error);

    // Get camera info to check if camera is running in color or mono mode
    CameraInfo c_info;
    error = cam_.GetCameraInfo(&c_info);
    PointGreyCamera::handleError("PointGreyCamera::connect  Failed to get camera info.", error);
    is_color_ = c_info.isColorCamera;
    cam_name_ = c_info.modelName;

    // Enable metadata
    EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = cam_.SetEmbeddedImageInfo(&info);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not enable metadata", error);
  }
}

void PointGreyCamera::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  capture_running_ = false;
  if (cam_.IsConnected())
  {
    Error error = cam_.Disconnect();
    PointGreyCamera::handleError("PointGreyCamera::disconnect Failed to disconnect camera", error);
  }
}

void PointGreyCamera::start()
{
  if (cam_.IsConnected() && !capture_running_)
  {
    // Start capturing images
    Error error = cam_.StartCapture();
    PointGreyCamera::handleError("PointGreyCamera::start Failed to start capture", error);
    capture_running_ = true;
  }
}

void PointGreyCamera::stop()
{
  if (cam_.IsConnected() && capture_running_)
  {
    // Stop capturing images
    capture_running_ = false;
    Error error = cam_.StopCapture();
    PointGreyCamera::handleError("PointGreyCamera::stop Failed to stop capture", error);
  }
}

bool PointGreyCamera::shutdown()
{
  // Stop capturing images
  capture_running_ = false;
  Error error = cam_.StopCapture();
  PointGreyCamera::handleError("PointGreyCamera::stop Failed to stop capture", error);
  // Disconnect camera
  disconnect();
  return true;
}

std::string PointGreyCamera::getName() const
{
  return cam_name_;
}

bool PointGreyCamera::getRawImagePair(sensor_msgs::ImagePtr &img_left, sensor_msgs::ImagePtr &img_right,
                                      uint64_t &timestamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cam_.IsConnected() && capture_running_)
  {
    // Make a FlyCapture2::Image to hold the buffer returned by the camera.
    Image raw_image;
    // Retrieve an image
    Error error = cam_.RetrieveBuffer(&raw_image);
    PointGreyCamera::handleError("PointGreyCamera::grabStereoImage Failed to retrieve buffer", error);
    metadata_ = raw_image.GetMetadata();

    // Set header timestamp as embedded for now
    TimeStamp embeddedTime = raw_image.GetTimeStamp();
    img_left->header.stamp.sec = embeddedTime.seconds;
    img_left->header.stamp.nsec = 1000 * embeddedTime.microSeconds;

    // GetBitsPerPixel returns 16, but that seems to mean "2 8 bit pixels,
    // one for each image". Therefore, we don't use it
    // uint8_t bits_per_pixel = raw_image.GetBitsPerPixel();

    // Set the image encoding
    std::string image_encoding = sensor_msgs::image_encodings::MONO8;
    BayerTileFormat bayer_format = raw_image.GetBayerTileFormat();

    if (is_color_ && bayer_format != NONE)
    {
      switch (bayer_format)
      {
        case RGGB:
          image_encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
          break;
        case GRBG:
          image_encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          break;
        case GBRG:
          image_encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
          break;
        case BGGR:
          image_encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          break;
      }
    }
    else  // Mono camera
    {
      image_encoding = sensor_msgs::image_encodings::MONO8;
    }

    // Set up the output images
    img_left->encoding = image_encoding;
    img_right->encoding = image_encoding;
    img_left->height = raw_image.GetRows();
    img_right->height = img_left->height;
    img_left->width = raw_image.GetCols();
    img_right->width = img_left->width;
    img_left->step = raw_image.GetStride() / 2;
    img_right->step = img_left->step;
    img_left->is_bigendian = 0;
    img_right->is_bigendian = 0;
    size_t st0 = (img_left->height * img_left->step);
    img_left->data.resize(st0);
    img_right->data.resize(st0);

    // Get the image data
    const uint8_t *raw_data = raw_image.GetData();

    // Step through the raw data and set each image in turn
    for (size_t i = 0; i < raw_image.GetRows(); i++)  // Rows
    {
      for (size_t j = 0; j < raw_image.GetCols();
           j++)  // Columns that need to have the 16 bits split into 2 8 bit groups
      {
        size_t index = i * img_left->step + j;
        size_t raw_index = 2 * index;
        img_left->data[index] = raw_data[raw_index];
        img_right->data[index] = raw_data[raw_index + 1];
      }
    }

    return true;
  }
  else if (cam_.IsConnected())
  {
    throw CameraNotRunningException(
        "PointGreyCamera::grabStereoImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabStereoImage not connected!");
  }
}

bool PointGreyCamera::setNewConfiguration(caros_camera::PointGreyConfig &config, const uint32_t &level)
{
  if (!cam_.IsConnected())
  {
    PointGreyCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> lock(mutex_);

  // return true if we can set values as desired.
  bool ret_val = true;

  // Check video mode
  VideoMode v_mode;  // video mode desired
  Mode fmt_7_mode;   // fmt_7_mode to set
  ret_val &= PointGreyCamera::getVideoModeFromString(config.video_mode, v_mode, fmt_7_mode);

  // Only change video mode if we have to.
  // dynamic_reconfigure will report anything other than LEVEL_RECONFIGURE_RUNNING if we need to change videomode.
  if (level != PointGreyCamera::LEVEL_RECONFIGURE_RUNNING)
  {
    bool was_running = false;  // Check if camera is running, and if it is, stop it.
    if (cam_.IsConnected() && capture_running_)
      was_running = true;

    if (v_mode == VIDEOMODE_FORMAT7)
    {
      PixelFormat fmt_7_pix_fmt;
      PointGreyCamera::getFormat7PixelFormatFromString(config.format7_color_coding, fmt_7_pix_fmt);
      // Oh no, these all need to be converted into uints, so my pass by reference trick doesn't work
      uint16_t uwidth = (uint16_t)config.format7_roi_width;
      uint16_t uheight = (uint16_t)config.format7_roi_height;
      uint16_t uoffsetx = (uint16_t)config.format7_x_offset;
      uint16_t uoffsety = (uint16_t)config.format7_y_offset;
      ret_val &= PointGreyCamera::setFormat7(fmt_7_mode, fmt_7_pix_fmt, uwidth, uheight, uoffsetx, uoffsety);
      config.format7_roi_width = uwidth;
      config.format7_roi_height = uheight;
      config.format7_x_offset = uoffsetx;
      config.format7_y_offset = uoffsety;
    }
    else
    {
      // Need to set just videoMode
      PointGreyCamera::setVideoMode(v_mode);
    }
    // Restart the camera if it was running
    if (was_running)
    {
      PointGreyCamera::start();
    }
  }

  // Set frame rate
  ret_val &= PointGreyCamera::setProperty(FRAME_RATE, false, config.frame_rate);

  // Set exposure
  ret_val &= PointGreyCamera::setProperty(AUTO_EXPOSURE, config.auto_exposure, config.exposure);

  // Set shutter time
  double shutter = 1000.0 * config.shutter_speed;  // Needs to be in milliseconds
  ret_val &= PointGreyCamera::setProperty(SHUTTER, config.auto_shutter, shutter);
  config.shutter_speed = shutter / 1000.0;  // Needs to be in seconds

  // Set gain
  ret_val &= PointGreyCamera::setProperty(GAIN, config.auto_gain, config.gain);

  // Set pan
  unsigned int pan = config.pan;
  unsigned int not_used = 0;
  ret_val &= PointGreyCamera::setProperty(PAN, false, pan, not_used);
  config.pan = pan;

  // Set tilt
  unsigned int tilt = config.tilt;
  ret_val &= PointGreyCamera::setProperty(TILT, false, tilt, not_used);
  config.tilt = tilt;

  // Set brightness
  ret_val &= PointGreyCamera::setProperty(BRIGHTNESS, false, config.brightness);

  // Set gamma
  ret_val &= PointGreyCamera::setProperty(GAMMA, false, config.gamma);

  // Set white balance
  uint16_t blue = config.white_balance_blue;
  uint16_t red = config.white_balance_red;
  ret_val &= PointGreyCamera::setWhiteBalance(config.auto_white_balance, blue, red);
  config.white_balance_blue = blue;
  config.white_balance_red = red;

  // Set trigger
  switch (config.trigger_polarity)
  {
    case caros_camera::PointGrey_Low:
    case caros_camera::PointGrey_High:
    {
      bool temp = config.trigger_polarity;
      ret_val &= PointGreyCamera::setExternalTrigger(config.enable_trigger, config.trigger_mode, config.trigger_source,
                                                     config.trigger_parameter, config.trigger_delay, temp);
      config.strobe1_polarity = temp;
    }
    break;
    default:
      ret_val &= false;
  }

  // Set strobe
  switch (config.strobe1_polarity)
  {
    case caros_camera::PointGrey_Low:
    case caros_camera::PointGrey_High:
    {
      bool temp = config.strobe1_polarity;
      ret_val &= PointGreyCamera::setExternalStrobe(config.enable_strobe1, caros_camera::PointGrey_GPIO1,
                                                    config.strobe1_duration, config.strobe1_delay, temp);
      config.strobe1_polarity = temp;
    }
    break;
    default:
      ret_val &= false;
  }

  return ret_val;
}

void PointGreyCamera::setGain(double &gain)
{
  PointGreyCamera::setProperty(GAIN, false, gain);
}

void PointGreyCamera::setBRWhiteBalance(bool auto_white_balance, uint16_t &blue, uint16_t &red)
{
  PointGreyCamera::setWhiteBalance(auto_white_balance, blue, red);
}

void PointGreyCamera::setVideoMode(FlyCapture2::VideoMode &video_mode)
{
  // Just set max frame rate, the actual double parameter will do the fine adjustments
  FrameRate frame_rate = FRAMERATE_7_5;  // Most reliable, so set as default.
  if (video_mode == VIDEOMODE_640x480Y8)
  {
    frame_rate = FRAMERATE_30;
  }
  else if (video_mode == VIDEOMODE_1280x960Y8)
  {
    frame_rate = FRAMERATE_15;
  }
  else if (video_mode == VIDEOMODE_1280x960Y16)
  {
    frame_rate = FRAMERATE_7_5;
  }
  else if (video_mode == VIDEOMODE_FORMAT7)
  {
    frame_rate = FRAMERATE_FORMAT7;
  }
  Error error = cam_.SetVideoModeAndFrameRate(video_mode, frame_rate);
  PointGreyCamera::handleError("PointGreyCamera::setVideoMode Could not set video mode", error);
}

bool PointGreyCamera::setFormat7(FlyCapture2::Mode &fmt_7_mode, FlyCapture2::PixelFormat &fmt_7_pix_fmt,
                                 uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x,
                                 uint16_t &roi_offset_y)
{
  // return true if we can set values as desired.
  bool ret_val = true;
  // Error for checking if functions went okay
  Error error;

  // Get Format7 information
  Format7Info fmt_7_info;
  bool supported;
  fmt_7_info.mode = fmt_7_mode;
  error = cam_.GetFormat7Info(&fmt_7_info, &supported);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Could not get Format 7 information", error);
  if (!supported)
  {
    throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 mode not supported on this camera.");
  }

  // Make Format7 Configuration
  Format7ImageSettings fmt_7_image_settings;
  fmt_7_image_settings.mode = fmt_7_mode;
  fmt_7_image_settings.pixelFormat = fmt_7_pix_fmt;

  // Check Width
  roi_width = roi_width / fmt_7_info.imageHStepSize *
              fmt_7_info.imageHStepSize;  // Locks the width into an appropriate multiple using an integer divide
  if (roi_width == 0)
  {
    fmt_7_image_settings.width = fmt_7_info.maxWidth;
  }
  else if (roi_width > fmt_7_info.maxWidth)
  {
    roi_width = fmt_7_info.maxWidth;
    fmt_7_image_settings.width = fmt_7_info.maxWidth;
    ret_val &= false;
  }
  else
  {
    fmt_7_image_settings.width = roi_width;
  }

  // Check Height
  roi_height = roi_height / fmt_7_info.imageVStepSize *
               fmt_7_info.imageVStepSize;  // Locks the height into an appropriate multiple using an integer divide
  if (roi_height == 0)
  {
    fmt_7_image_settings.height = fmt_7_info.maxHeight;
  }
  else if (roi_height > fmt_7_info.maxHeight)
  {
    roi_height = fmt_7_info.maxHeight;
    fmt_7_image_settings.height = fmt_7_info.maxHeight;
    ret_val &= false;
  }
  else
  {
    fmt_7_image_settings.height = roi_height;
  }

  // Check OffsetX
  roi_offset_x = roi_offset_x / fmt_7_info.offsetHStepSize *
                 fmt_7_info.offsetHStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
  if (roi_offset_x > (fmt_7_info.maxWidth - fmt_7_image_settings.width))
  {
    roi_offset_x = fmt_7_info.maxWidth - fmt_7_image_settings.width;
    ret_val &= false;
  }
  fmt_7_image_settings.offsetX = roi_offset_x;

  // Check OffsetY
  roi_offset_y = roi_offset_y / fmt_7_info.offsetVStepSize *
                 fmt_7_info.offsetVStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
  if (roi_offset_y > fmt_7_info.maxHeight - fmt_7_image_settings.height)
  {
    roi_offset_y = fmt_7_info.maxHeight - fmt_7_image_settings.height;
    ret_val &= false;
  }
  fmt_7_image_settings.offsetY = roi_offset_y;

  // Validate the settings to make sure that they are valid
  Format7PacketInfo fmt_7_packet_info;
  bool valid;
  error = cam_.ValidateFormat7Settings(&fmt_7_image_settings, &valid, &fmt_7_packet_info);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Error validating Format 7 settings", error);
  if (!valid)
  {
    throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 Settings Not Valid.");
  }

  // Stop the camera to allow settings to change.
  error = cam_.SetFormat7Configuration(&fmt_7_image_settings, fmt_7_packet_info.recommendedBytesPerPacket);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Could not send Format7 configuration to the camera", error);

  // Get camera info to check if camera is running in color or mono mode
  CameraInfo c_info;
  error = cam_.GetCameraInfo(&c_info);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7  Failed to get camera info.", error);
  is_color_ = c_info.isColorCamera;

  return ret_val;
}

bool PointGreyCamera::getVideoModeFromString(std::string &vmode, FlyCapture2::VideoMode &vmode_out,
                                             FlyCapture2::Mode &fmt_7_mode)
{
  // return true if we can set values as desired.
  bool ret_val = true;

  // Get camera info to check if color or black and white chameleon
  CameraInfo c_info;
  Error error = cam_.GetCameraInfo(&c_info);
  PointGreyCamera::handleError("PointGreyCamera::getVideoModeFromString  Failed to get camera info.", error);

  if (vmode.compare("640x480_mono8") == 0)
  {
    vmode_out = VIDEOMODE_640x480Y8;
  }
  else if (vmode.compare("640x480_mono16") == 0)
  {
    vmode_out = VIDEOMODE_640x480Y16;
  }
  else if (vmode.compare("1280x960_mono8") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y8;
    if (c_info.isColorCamera)  // Is color camera, set the output differently
    {
      vmode = "1280x960_bayer8";
      ret_val &= false;
    }
  }
  else if (vmode.compare("1280x960_bayer8") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y8;
    if (!c_info.isColorCamera)  // Is black and white camera, set the output differently
    {
      vmode = "1280x960_mono8";
      ret_val &= false;
    }
  }
  else if (vmode.compare("1280x960_mono16") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y16;
  }
  else if (vmode.compare("format7_mode0") == 0)
  {
    fmt_7_mode = MODE_0;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if (vmode.compare("format7_mode1") == 0)
  {
    fmt_7_mode = MODE_1;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if (vmode.compare("format7_mode2") == 0)
  {
    fmt_7_mode = MODE_2;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if (vmode.compare("format7_mode3") == 0)
  {
    fmt_7_mode = MODE_3;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if (vmode.compare("format7_mode4") == 0)
  {
    fmt_7_mode = MODE_4;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else  // Something not supported was asked of us, drop down into the most compatible mode
  {
    vmode = "640x480_mono8";
    vmode_out = VIDEOMODE_640x480Y8;
    ret_val &= false;
  }

  return ret_val;
}

bool PointGreyCamera::getFormat7PixelFormatFromString(std::string &sformat, FlyCapture2::PixelFormat &fmt_7_pix_fmt)
{
  // return true if we can set values as desired.
  bool ret_val = true;

  // Get camera info to check if color or black and white camera
  CameraInfo c_info;
  Error error = cam_.GetCameraInfo(&c_info);
  PointGreyCamera::handleError("PointGreyCamera::getFormat7PixelFormatFromString  Failed to get camera info.", error);

  if (c_info.isColorCamera)
  {
    if (sformat.compare("raw8") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_RAW8;
    }
    else if (sformat.compare("raw16") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_RAW16;
    }
    else if (sformat.compare("mono8") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_MONO8;
    }
    else if (sformat.compare("mono16") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_MONO16;
    }
    else
    {
      sformat = "raw8";
      fmt_7_pix_fmt = PIXEL_FORMAT_RAW8;
      ret_val &= false;
    }
  }
  else  // Is black and white
  {
    if (sformat.compare("mono8") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_MONO8;
    }
    else if (sformat.compare("mono16") == 0)
    {
      fmt_7_pix_fmt = PIXEL_FORMAT_MONO16;
    }
    else
    {
      sformat = "mono8";
      fmt_7_pix_fmt = PIXEL_FORMAT_MONO8;
      ret_val &= false;
    }
  }

  return ret_val;
}

bool PointGreyCamera::setProperty(const FlyCapture2::PropertyType &type, const bool &auto_set, unsigned int &value_a,
                                  unsigned int &value_b)
{
  // return true if we can set values as desired.
  bool ret_val = true;

  PropertyInfo p_info;
  p_info.type = type;
  Error error = cam_.GetPropertyInfo(&p_info);
  PointGreyCamera::handleError("PointGreyCamera::setProperty Could not get property info.", error);

  if (p_info.present)
  {
    Property prop;
    prop.type = type;
    prop.autoManualMode = (auto_set && p_info.autoSupported);
    prop.absControl = false;
    prop.onOff = p_info.onOffSupported;

    if (value_a < p_info.min)
    {
      value_a = p_info.min;
      ret_val &= false;
    }
    else if (value_a > p_info.max)
    {
      value_a = p_info.max;
      ret_val &= false;
    }
    if (value_b < p_info.min)
    {
      value_b = p_info.min;
      ret_val &= false;
    }
    else if (value_b > p_info.max)
    {
      value_b = p_info.max;
      ret_val &= false;
    }
    prop.valueA = value_a;
    prop.valueB = value_b;
    error = cam_.SetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to set property ",
                                 error); /** @todo say which property? */

    // Read back setting to confirm
    error = cam_.GetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to confirm property ",
                                 error); /** @todo say which property? */
    if (!prop.autoManualMode)
    {
      value_a = prop.valueA;
      value_b = prop.valueB;
    }
  }
  else  // Not supported
  {
    value_a = 0;
    value_b = 0;
  }

  return ret_val;
}

bool PointGreyCamera::setProperty(const FlyCapture2::PropertyType &type, const bool &auto_set, double &value)
{
  // return true if we can set values as desired.
  bool ret_val = true;

  PropertyInfo p_info;
  p_info.type = type;
  Error error = cam_.GetPropertyInfo(&p_info);
  PointGreyCamera::handleError("PointGreyCamera::setProperty Could not get property info.", error);

  if (p_info.present)
  {
    Property prop;
    prop.type = type;
    prop.autoManualMode = (auto_set && p_info.autoSupported);
    prop.absControl = p_info.absValSupported;
    prop.onOff = p_info.onOffSupported;

    if (value < p_info.absMin)
    {
      value = p_info.absMin;
      ret_val &= false;
    }
    else if (value > p_info.absMax)
    {
      value = p_info.absMax;
      ret_val &= false;
    }
    prop.absValue = value;
    error = cam_.SetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to set property ",
                                 error); /** @todo say which property? */

    // Read back setting to confirm
    error = cam_.GetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to confirm property ",
                                 error); /** @todo say which property? */
    if (!prop.autoManualMode)
    {
      value = prop.absValue;
    }
  }
  else  // Not supported
  {
    value = 0.0;
  }
  return ret_val;
}

bool PointGreyCamera::setWhiteBalance(bool &auto_white_balance, uint16_t &blue, uint16_t &red)
{
  // Get camera info to check if color or black and white chameleon
  CameraInfo c_info;
  Error error = cam_.GetCameraInfo(&c_info);
  handleError("PointGreyCamera::setWhiteBalance  Failed to get camera info.", error);

  if (!c_info.isColorCamera)
  {
    // Not a color camera, does not support auto white balance
    auto_white_balance = false;
    red = 0;
    blue = 0;
    return false;
  }

  unsigned white_balance_addr = 0x80c;
  unsigned enable = 1 << 31;
  unsigned value = 1 << 25;

  if (auto_white_balance)
  {
    PropertyInfo prop_info;
    prop_info.type = WHITE_BALANCE;
    error = cam_.GetPropertyInfo(&prop_info);
    handleError("PointGreyCamera::setWhiteBalance  Failed to get property info.", error);
    if (!prop_info.autoSupported)
    {
      // This is typically because a color camera is in mono mode, so we set
      // the red and blue to some reasonable value for later use
      auto_white_balance = false;
      blue = 800;
      red = 550;
      return false;
    }
    // Auto white balance is supported
    error = cam_.WriteRegister(white_balance_addr, enable);
    handleError("PointGreyCamera::setWhiteBalance  Failed to write to register.", error);
    value |= 1 << 24;
  }
  else
  {
    // Manual mode
    value |= blue << 12 | red;
  }
  error = cam_.WriteRegister(white_balance_addr, value);
  handleError("PointGreyCamera::setWhiteBalance  Failed to write to register.", error);
  return true;
}

void PointGreyCamera::setTimeout(const double &timeout)
{
  FC2Config p_config;
  Error error = cam_.GetConfiguration(&p_config);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not get camera configuration", error);
  p_config.grabTimeout = (int)(1000.0 * timeout);  // Needs to be in ms
  if (p_config.grabTimeout < 0.00001)
  {
    p_config.grabTimeout = -1;  // Default - no timeout
  }
  error = cam_.SetConfiguration(&p_config);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not set camera configuration", error);
}

float PointGreyCamera::getCameraTemperature()
{
  Property t_prop;
  t_prop.type = TEMPERATURE;
  Error error = cam_.GetProperty(&t_prop);
  PointGreyCamera::handleError("PointGreyCamera::getCameraTemperature Could not get property.", error);
  return t_prop.valueA / 10.0f - 273.15f;  // It returns values of 10 * K
}

float PointGreyCamera::getCameraFrameRate()
{
  Property f_prop;
  f_prop.type = FRAME_RATE;
  Error error = cam_.GetProperty(&f_prop);
  PointGreyCamera::handleError("PointGreyCamera::getCameraFrameRate Could not get property.", error);
  std::cout << "Frame Rate is: " << f_prop.absValue << std::endl;
  return f_prop.absValue;
}

static int sourceNumberFromGpioName(const std::string s)
{
  if (s.compare("gpio0") == 0)
  {
    return 0;
  }
  else if (s.compare("gpio1") == 0)
  {
    return 1;
  }
  else if (s.compare("gpio2") == 0)
  {
    return 2;
  }
  else if (s.compare("gpio3") == 0)
  {
    return 3;
  }
  else
  {
    // Unrecognized pin
    return -1;
  }
}

bool PointGreyCamera::setExternalStrobe(bool &enable, const std::string &dest, double &duration, double &delay,
                                        bool &polarity_high)
{
  // return true if we can set values as desired.
  bool ret_val = true;

  // Check strobe source
  int pin;
  pin = sourceNumberFromGpioName(dest);
  if (pin < 0)
  {
    // Unrecognized source
    return false;
  }
  // Check for external trigger support
  StrobeInfo strobe_info;
  strobe_info.source = pin;
  Error error = cam_.GetStrobeInfo(&strobe_info);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not check external strobe support.", error);
  if (strobe_info.present != true)
  {
    // Camera doesn't support external strobes on this pin, so set enable_strobe to false
    enable = false;
    return false;
  }

  StrobeControl strobe_control;
  strobe_control.source = pin;
  error = cam_.GetStrobe(&strobe_control);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not get strobe control.", error);
  strobe_control.duration = duration;
  strobe_control.delay = delay;
  strobe_control.onOff = enable;
  strobe_control.polarity = polarity_high;

  error = cam_.SetStrobe(&strobe_control);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not set strobe control.", error);
  error = cam_.GetStrobe(&strobe_control);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not get strobe control.", error);
  delay = strobe_control.delay;
  enable = strobe_control.onOff;
  polarity_high = strobe_control.polarity;

  return ret_val;
}

bool PointGreyCamera::setExternalTrigger(bool &enable, std::string &mode, std::string &source, int32_t &parameter,
                                         double &delay, bool &polarityHigh)
{
  // return true if we can set values as desired.
  bool ret_val = true;
  // Check for external trigger support
  TriggerModeInfo trigger_mode_info;
  Error error = cam_.GetTriggerModeInfo(&trigger_mode_info);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not check external trigger support.", error);
  if (trigger_mode_info.present != true)
  {
    // Camera doesn't support external triggering, so set enable_trigger to false
    enable = false;
    return false;
  }

  TriggerMode trigger_mode;
  error = cam_.GetTriggerMode(&trigger_mode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger mode.", error);
  trigger_mode.onOff = enable;

  // Set trigger mode
  std::string tmode = mode;
  if (tmode.compare("mode0") == 0)
  {
    trigger_mode.mode = 0;
  }
  else if (tmode.compare("mode1") == 0)
  {
    trigger_mode.mode = 1;
  }
  else if (tmode.compare("mode3") == 0)
  {
    trigger_mode.mode = 3;
  }
  else if (tmode.compare("mode14") == 0)
  {
    trigger_mode.mode = 14;
  }
  else
  {
    // Unrecognized mode
    trigger_mode.mode = 0;
    mode = "mode0";
    ret_val &= false;
  }

  // Parameter is used for mode3 (return one out of every N frames).  So if N is two, it returns every other frame.
  trigger_mode.parameter = parameter;

  // Set trigger source
  std::string tsource = source;
  int pin = sourceNumberFromGpioName(tsource);
  if (pin < 0)
  {
    // Unrecognized source
    trigger_mode.source = 0;
    source = "gpio0";
    ret_val &= false;
  }
  else
  {
    trigger_mode.source = pin;
  }

  trigger_mode.polarity = polarityHigh;

  error = cam_.SetTriggerMode(&trigger_mode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not set trigger mode.", error);
  error = cam_.GetTriggerMode(&trigger_mode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger mode.", error);
  enable = trigger_mode.onOff;
  std::stringstream buff;
  buff << "mode" << trigger_mode.mode;
  mode = buff.str();

  /** @todo, check delay min and max values */

  // Set trigger delay
  TriggerDelay trigger_delay;
  trigger_delay.type = TRIGGER_DELAY;
  trigger_delay.absControl = true;
  trigger_delay.absValue = delay;
  trigger_delay.onOff = true;
  error = cam_.SetTriggerDelay(&trigger_delay);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not set trigger delay.", error);
  error = cam_.GetTriggerDelay(&trigger_delay);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger delay.", error);
  delay = trigger_delay.absValue;

  return ret_val;
}

void PointGreyCamera::handleError(const std::string &prefix, const FlyCapture2::Error &error)
{
  if (error == PGRERROR_TIMEOUT)
  {
    throw CameraTimeoutException("PointGreyCamera: Failed to retrieve buffer within timeout.");
  }
  else if (error !=
           PGRERROR_OK)  // If there is actually an error (PGRERROR_OK means the function worked as intended...)
  {
    std::string start(" | FlyCapture2::ErrorType ");
    std::stringstream out;
    out << error.GetType();
    std::string desc(error.GetDescription());
    throw std::runtime_error(prefix + start + out.str() + " " + desc);
  }
}

}  // namespace caros