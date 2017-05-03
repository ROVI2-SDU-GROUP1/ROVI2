#include <caros/basler_camera.h>

#include <string>
#include <vector>
#include <algorithm>

namespace caros
{
BaslerCamera::BaslerCamera() : CameraInterface()
{
}

BaslerCamera::BaslerCamera(std::string serial) : CameraInterface(serial)
{
  gige_camera_initialized_ = false;
  Pylon::PylonAutoInitTerm auto_init_term;  // automatically calls PylonInitialize and PylonTerminate
  // Get the transport layer factory
  Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
  // Get all attached GigE cameras
  Pylon::DeviceInfoList_t devices;
  gige_cam_list_ = devices;
  nr_of_gige_cameras_ = TlFactory.EnumerateDevices(devices);
}

BaslerCamera::~BaslerCamera()
{
}

bool BaslerCamera::init()
{
  /* Attach Pylon device from serial */
  Pylon::CDeviceInfo info;
  info.SetSerialNumber(serial_.c_str());
  gige_camera_.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(info));

  /* Setup parameters */
  GenApi::INodeMap& nodemap = setup(gige_camera_);

  if (gige_camera_.IsPylonDeviceAttached())
  {
    gige_camera_initialized_ = true;
    return true;
  }
  else
    return false;
}

void BaslerCamera::start()
{
  if (gige_camera_initialized_)
  {
    gige_camera_.StartGrabbing();
  }
}

bool BaslerCamera::shutdown()
{
  gige_camera_.DestroyDevice();
}

void BaslerCamera::stop()
{
  if (gige_camera_initialized_)
    gige_camera_.StopGrabbing();
}

bool BaslerCamera::isRunning() const
{
  if (gige_camera_initialized_)
    if (gige_camera_.IsGrabbing())
      return true;
    else
      return false;
  else
    return false;
}

std::string BaslerCamera::getName() const
{
  return gige_camera_.GetDeviceInfo().GetModelName().c_str();
}

bool BaslerCamera::getRawImage(sensor_msgs::ImagePtr& img, uint64_t& timestamp)
{
  // Wait for an image and then retrieve it, with timeout [ms]
  Pylon::CGrabResultPtr ptr_grab_result;
  gige_camera_.RetrieveResult(1000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);

  // Image grabbed successfully?
  if (ptr_grab_result->GrabSucceeded())
  {
    // Access the image data.
    const uchar* buf = reinterpret_cast<uchar*>(ptr_grab_result->GetBuffer());
    const size_t rows = ptr_grab_result->GetHeight();
    const size_t cols = ptr_grab_result->GetWidth();

    uchar copy[rows * cols];
    std::copy(buf, buf + rows * cols, copy);

    // Create the output image
    //cv::Mat frame(cv::Size(cols, rows), CV_8UC1, copy, cv::Mat::AUTO_STEP);

    cv::Mat temp(rows, cols, CV_8UC1, copy);
    timestamp = ptr_grab_result->GetTimeStamp();
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    img = cv_bridge::CvImage(header, "bayer_rggb8", temp).toImageMsg();
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Error: " << ptr_grab_result->GetErrorCode() << " " << ptr_grab_result->GetErrorDescription());
    return false;
  }
}

// Set any type of parameter
template <typename T>
bool setParam(GenApi::INodeMap& nodemap, const std::string& name, const std::string& value)
{
  GenApi::CPointer<T> param(nodemap.GetNode(name.c_str()));
  if (IsWritable(param))
  {
    param->FromString(value.c_str());
    ROS_DEBUG_STREAM("\t" << name << ": " << param->GetValue());
  }
  else
  {
    ROS_DEBUG_STREAM("Failed to set camera parameter '" << name << "' to '" << value << "'!");
    return false;
  }

  return true;
}

// Special case: IEnumeration, where we make an extra check for the existence of the enum value
template <>
bool setParam<GenApi::IEnumeration>(GenApi::INodeMap& nodemap, const std::string& name, const std::string& value)
{
  GenApi::CPointer<GenApi::IEnumeration> param(nodemap.GetNode(name.c_str()));
  if (GenApi::IsWritable(param) && GenApi::IsAvailable(param->GetEntryByName(value.c_str())))
  {  // Extra check here
    param->FromString(value.c_str());
    ROS_DEBUG_STREAM("\t" << name << ": " << param->GetCurrentEntry()->GetSymbolic());
  }
  else
  {
    ROS_DEBUG_STREAM("Failed to set camera parameter '" << name << "' to '" << value << "'!");
    return false;
  }

  return true;
}

GenApi::INodeMap& BaslerCamera::setup(Pylon::CInstantCamera& camera)
{
  ROS_DEBUG_STREAM("Setting parameters for: " << camera.GetDeviceInfo().GetModelName() << " ("
                                              << camera.GetDeviceInfo().GetSerialNumber() << ")");

  if (camera.IsOpen())
  {
    ROS_DEBUG("Camera is already open! Closing...");
    camera.Close();
  }

  camera.Open();

  GenApi::INodeMap& nodemap = camera.GetNodeMap();
  GenApi::INodeMap& tl_nodemap = camera.GetTLNodeMap();

  // Set the pixel format to Mono8
  //    setParam<IEnumeration>(nodemap, "PixelFormat", "Mono8");

  /*
   * Auto parameters for avoiding saturation
   *   - Set the AOI to the region where stuff actually is
   */
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIWidth", "400");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIHeight", "400");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIOffsetX", "400");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIOffsetY", "400");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIWidth", "600");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIHeight", "600");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIOffsetX", "400");
  //    setParam<IInteger>(nodemap, "AutoFunctionAOIOffsetY", "400");
  setParam<GenApi::IEnumeration>(nodemap, "GainAuto", "Continuous");
  setParam<GenApi::IEnumeration>(nodemap, "ExposureAuto", "Continuous");
  setParam<GenApi::IEnumeration>(nodemap, "BalanceWhiteAuto", "Continuous");
  //    setParam<IEnumeration>(nodemap, "GainAuto", "Once");
  //    setParam<IEnumeration>(nodemap, "ExposureAuto", "Once");
  //    setParam<IInteger>(nodemap, "AutoTargetValue", "50");
  //    setParam<IBoolean>(nodemap, "GammaEnable", "True");
  //    setParam<IFloat>(nodemap, "Gamma", "2");

  /*
   * Set heartbeat parameter
   */

  setParam<GenApi::IInteger>(tl_nodemap, "HeartbeatTimeout", std::to_string(3 * 1000));  // set heartbeat to 3 seconds.

  /*
   * Set some extra bandwidth parameters
   */
  setParam<GenApi::IInteger>(nodemap, "GevSCPSPacketSize", "1500");  // MTU in bytes ( use jumbo frames)
  setParam<GenApi::IInteger>(nodemap, "GevSCPD", "100");              // Inter-packet delay in ticks
  camera.MaxNumBuffer = 1;                                           // Avoid lags when running at < 30 Hz

  return nodemap;
}


// Adjust value to make it comply with range and increment passed.
//
// The parameter's minimum and maximum are always considered as valid values.
// If the increment is larger than one, the returned value will be: min + (n * inc).
// If the value doesn't meet these criteria, it will be rounded down to ensure compliance.
int64_t adjust(int64_t val, int64_t minimum, int64_t maximum, int64_t inc)
{
  // Check the input parameters.
  if (inc <= 0)
  {
    // Negative increments are invalid.
    throw LOGICAL_ERROR_EXCEPTION("Unexpected increment %d", inc);
  }
  if (minimum > maximum)
  {
    // Minimum must not be bigger than or equal to the maximum.
    throw LOGICAL_ERROR_EXCEPTION("minimum bigger than maximum.");
  }
  // Check the lower bound.
  if (val < minimum)
  {
    return minimum;
  }
  // Check the upper bound.
  if (val > maximum)
  {
    return maximum;
  }
  // Check the increment.
  if (inc == 1)
  {
    // Special case: all values are valid.
    return val;
  }
  else
  {
    // The value must be min + (n * inc).
    // Due to the integer division, the value will be rounded down.
    return minimum + ( ((val - minimum) / inc) * inc );
  }
}


bool BaslerCamera::setupParameters(std::unordered_map<std::string, std::string> parameters,
                                   std::map<std::string, bool> bool_parameters,
                                   std::map<std::string, int> int_parameters,
                                   std::map<std::string, double> double_parameters)
{
  GenApi::INodeMap& nodemap = gige_camera_.GetNodeMap();

  // Set the pixel format
  gige_camera_.StopGrabbing();
  setParam<GenApi::IEnumeration>(nodemap, "PixelFormat", parameters["PixelFormat"]);
  gige_camera_.StartGrabbing();

  /*
   * Auto parameters for avoiding saturation
   */
  setParam<GenApi::IEnumeration>(nodemap, "GainAuto", parameters["GainAuto"]);
  setParam<GenApi::IEnumeration>(nodemap, "ExposureAuto", parameters["ExposureAuto"]);
  setParam<GenApi::IEnumeration>(nodemap, "BalanceWhiteAuto", parameters["BalanceWhiteAuto"]);
  //    setParam<IEnumeration>(nodemap, "GainAuto", "Once");
  //    setParam<IEnumeration>(nodemap, "ExposureAuto", "Once");
  //    setParam<IInteger>(nodemap, "AutoTargetValue", "50");
  //    setParam<IBoolean>(nodemap, "GammaEnable", "True");
  //    setParam<IFloat>(nodemap, "Gamma", "2");

  // Access the GainRaw integer type node. This node is available for Firewire and GigE Devices.
  GenApi::CIntegerPtr gain_raw(nodemap.GetNode("GainRaw"));
  if ( gain_raw.IsValid() )
  {
    int64_t new_gain_raw = gain_raw->GetMin() + int_parameters["GainRaw"]; //((gain_raw->GetMax() - gain_raw->GetMin()) / 2)
    // Make sure the calculated value is valid.
    new_gain_raw = adjust(new_gain_raw, gain_raw->GetMin(), gain_raw->GetMax(), gain_raw->GetInc());
    gain_raw->SetValue(new_gain_raw);
    std::cout << "Gain       : " << gain_raw->GetValue() << " (Min: " << gain_raw->GetMin() << "; Max: " <<
    gain_raw->GetMax() << "; Inc: " << gain_raw->GetInc() << ")" << std::endl;
  }

  // Access the ExposureTimeRaw integer type node.
  GenApi::CIntegerPtr exposure_time_raw(nodemap.GetNode("ExposureTimeRaw"));
  if ( exposure_time_raw.IsValid() )
  {
    int64_t new_exposure_time_raw = exposure_time_raw->GetMin() + int_parameters["ExposureTimeRaw"];
    // Make sure the calculated value is valid.
    new_exposure_time_raw = adjust(new_exposure_time_raw, exposure_time_raw->GetMin(), exposure_time_raw->GetMax(),
                          exposure_time_raw->GetInc());
    if (IsWritable(exposure_time_raw))
    {
      exposure_time_raw->SetValue(new_exposure_time_raw);
      std::cout << "Exposure Time Raw       : " << gain_raw->GetValue() << " (Min: " << gain_raw->GetMin() <<
      "; Max: " <<
      gain_raw->GetMax() << "; Inc: " << gain_raw->GetInc() << ")" << std::endl;
    }
  }

  GenApi::CBooleanPtr gamma_enable(nodemap.GetNode("GammaEnable"));

  if(bool_parameters["GammaEnable"])
  {
    gamma_enable->SetValue(true);

    // Access the Gamma float type node.
    GenApi::CFloatPtr gamma( nodemap.GetNode("Gamma"));
    if ( gamma.IsValid() )
    {
      double new_gamma = double_parameters["Gamma"];
      if (new_gamma < gamma->GetMax() && new_gamma > gamma->GetMin())
        gamma->SetValue(new_gamma);
      else
        ROS_WARN_STREAM("Will not accept the specified gamma value" << new_gamma << " the value must be between: "
                        << gamma->GetMin() << " and " << gamma->GetMax());
    }
  }
  else
  {
    gamma_enable->SetValue(false);
  }

  GenApi::CBooleanPtr acquisition_framerate_enable(nodemap.GetNode("AcquisitionFrameRateEnable"));

  if(bool_parameters["AcquisitionFrameRateEnable"])
  {
    acquisition_framerate_enable->SetValue(true);

    // Access the AcquisitionFrameRateAbs float type node.
    GenApi::CFloatPtr acquisition_framerate( nodemap.GetNode("AcquisitionFrameRateAbs"));
    if ( acquisition_framerate.IsValid() )
    {
      double new_acquisition_framerate = double_parameters["AcquisitionFrameRateAbs"];
      if (new_acquisition_framerate < acquisition_framerate->GetMax()
          && new_acquisition_framerate > acquisition_framerate->GetMin())
        acquisition_framerate->SetValue(new_acquisition_framerate);
      else
        ROS_WARN_STREAM("Will not accept the specified Acquisition Frame Rate" << new_acquisition_framerate
                        << " the value must be between: "
                        << acquisition_framerate->GetMin() << " and " << acquisition_framerate->GetMax());
    }
  }
  else
  {
    acquisition_framerate_enable->SetValue(false);
  }

  /*
   * Bandwidth parameters
   */
  setParam<GenApi::IInteger>(nodemap, "GevSCPSPacketSize",
                             parameters["GevSCPSPacketSize"]);            // MTU in bytes
  setParam<GenApi::IInteger>(nodemap, "GevSCPD", parameters["GevSCPD"]);  // Inter-packet delay in ticks

  return true;
}
}  // namespace caros
