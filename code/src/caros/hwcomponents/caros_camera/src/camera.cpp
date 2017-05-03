#include <caros/camera.h>
#include <ros/service_client.h>

#include <string>
#include <ros/assert.h>

namespace caros
{
Camera::Camera(const ros::NodeHandle& nodehandle)
    : CarosNodeServiceInterface(nodehandle, -1.0), CameraServiceInterface(nodehandle), nodehandle_(nodehandle)
{
  /* Currently nothing specific should happen */
}

Camera::~Camera()
{
  if (camera_manager_ != nullptr && camera_interface_ != nullptr)
  {
    if (camera_manager_->getCameraType() == (caros::CameraManager::ENSENSO || caros::CameraManager::BUMBLEBEE))
    {
      camera_interface_->shutdown();
    }
  }
}

void Camera::pointGreyCallback(caros_camera::PointGreyConfig& config, uint32_t level)
{
  if (camera_interface_ != nullptr)
  {
    // Stereo is only active in this mode (16 bits, 8 for each image)
    config.video_mode = "format7_mode3";

    try
    {
      ROS_DEBUG_STREAM("Dynamic reconfigure callback with level: " << level);
#ifdef BUMBLEBEE_SUPPORTED
      camera_interface_->setNewConfiguration(config, level);
#endif
    }
    catch (std::runtime_error& e)
    {
      ROS_DEBUG("Reconfigure Callback failed with error: %s", e.what());
    }
  }
  else
  {
    CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
  }
}

void Camera::baslerCallback(caros_camera::BaslerConfig& config, uint32_t level)
{
  ROS_INFO("Basler Reconfigure Request: ");
  ROS_INFO_STREAM("\t"
                  << "PixelFormat"
                  << ": " << config.PixelFormat.c_str());
  ROS_INFO_STREAM("\t"
                  << "GevSCPSPacketSize"
                  << ": " << config.GevSCPSPacketSize.c_str());
  ROS_INFO_STREAM("\t"
                  << "GevSCPD"
                  << ": " << config.GevSCPD.c_str());
  ROS_INFO_STREAM("\t"
                  << "ExposureAuto"
                  << ": " << config.ExposureAuto.c_str());
  ROS_INFO_STREAM("\t"
                  << "GainAuto"
                  << ": " << config.GainAuto.c_str());
  ROS_INFO_STREAM("\t"
                  << "BalanceWhiteAuto"
                  << ": " << config.BalanceWhiteAuto.c_str());
  ROS_INFO_STREAM("\t"
                  << "GainRaw"
                  << ": " << config.GainRaw);
  ROS_INFO_STREAM("\t"
                  << "ExposureTimeRaw"
                  << ": " << config.ExposureTimeRaw);
  ROS_INFO_STREAM("\t"
                  << "BlackLevelRaw"
                  << ": " << config.BlackLevelRaw);
  ROS_INFO_STREAM("\t"
                  << "Width"
                  << ": " << config.Width);
  ROS_INFO_STREAM("\t"
                  << "Height"
                  << ": " << config.Height);
  ROS_INFO_STREAM("\t"
                  << "AutoFunctionAOIWidth"
                  << ": " << config.AutoFunctionAOIWidth);
  ROS_INFO_STREAM("\t"
                  << "AutoFunctionAOIHeight"
                  << ": " << config.AutoFunctionAOIHeight);
  ROS_INFO_STREAM("\t"
                  << "GammaEnable: "
                  << ": " << std::boolalpha << config.GammaEnable);
  ROS_INFO_STREAM("\t"
                  << "AcquisitionFrameRateEnable: "
                  << ": " << std::boolalpha << config.AcquisitionFrameRateEnable);
  ROS_INFO_STREAM("\t"
                  << "Gamma"
                  << ": " << config.Gamma);
  ROS_INFO_STREAM("\t"
                  << "AcquisitionFrameRateAbs"
                  << ": " << config.AcquisitionFrameRateAbs);

  if (camera_interface_ != nullptr)
  {
    std::unordered_map<std::string, std::string> parameters;
    std::map<std::string, bool> bool_parameters;
    std::map<std::string, int> int_parameters;
    std::map<std::string, double> double_parameters;
    parameters["PixelFormat"] = config.PixelFormat;
    parameters["GevSCPSPacketSize"] = config.GevSCPSPacketSize;
    parameters["GevSCPD"] = config.GevSCPD;
    parameters["ExposureAuto"] = config.ExposureAuto;
    parameters["GainAuto"] = config.GainAuto;
    parameters["BalanceWhiteAuto"] = config.BalanceWhiteAuto;
    int_parameters["GainRaw"] = config.GainRaw;
    int_parameters["ExposureTimeRaw"] = config.ExposureTimeRaw;
    int_parameters["BlackLevelRaw"] = config.BlackLevelRaw;
    int_parameters["Width"] = config.Width;
    int_parameters["Height"] = config.Height;
    int_parameters["AutoFunctionAOIWidth"] = config.AutoFunctionAOIWidth;
    int_parameters["AutoFunctionAOIHeight"] = config.AutoFunctionAOIHeight;
    bool_parameters["GammaEnable"] = config.GammaEnable;
    bool_parameters["AcquisitionFrameRateEnable"] = config.AcquisitionFrameRateEnable;
    double_parameters["Gamma"] = config.Gamma;
    double_parameters["AcquisitionFrameRateAbs"] = config.AcquisitionFrameRateAbs;

    camera_interface_->setupParameters(parameters, bool_parameters, int_parameters, double_parameters);
  }
  else
  {
    CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
  }
}

void Camera::ensensoCallback(caros_camera::EnsensoConfig& config, uint32_t level)
{
  if (camera_interface_ != nullptr)
  {
    std::map<std::string, bool> bool_parameters;
    std::map<std::string, int> int_parameters;
    std::map<std::string, double> double_parameters;

    ROS_INFO("---");
    ROS_INFO("Capture Parameters");
    ROS_INFO_STREAM("AutoBlackLevel: " << std::boolalpha << config.AutoBlackLevel);
    ROS_INFO_STREAM("AutoExposure: " << std::boolalpha << config.AutoExposure);
    ROS_INFO_STREAM("AutoGain: " << std::boolalpha << config.AutoGain);
    ROS_INFO_STREAM("Binning: " << config.Binning);
    ROS_INFO_STREAM("BlackLevelOffset: " << config.BlackLevelOffset);
    ROS_INFO_STREAM("Exposure: " << config.Exposure);
    ROS_INFO_STREAM("FrontLight: " << std::boolalpha << config.FrontLight);
    ROS_INFO_STREAM("Gain: " << config.Gain);
    ROS_INFO_STREAM("GainBoost: " << std::boolalpha << config.GainBoost);
    ROS_INFO_STREAM("HardwareGamma: " << std::boolalpha << config.HardwareGamma);
    ROS_INFO_STREAM("Hdr: " << std::boolalpha << config.HDR);
    ROS_INFO_STREAM("PixelClock: " << config.PixelClock);
    ROS_INFO_STREAM("Projector: " << std::boolalpha << config.Projector);
    ROS_INFO_STREAM("TargetBrightness: " << config.TargetBrightness);
    ROS_INFO_STREAM("DisparityMapAOI: " << std::boolalpha << config.DisparityMapAOI);

    // Capture parameters
    bool_parameters["AutoBlackLevel"] = config.AutoBlackLevel;
    bool_parameters["AutoExposure"] = config.AutoExposure;
    bool_parameters["AutoGain"] = config.AutoGain;
    bool_parameters["FrontLight"] = config.FrontLight;
    bool_parameters["GainBoost"] = config.GainBoost;
    bool_parameters["HardwareGamma"] = config.HardwareGamma;
    bool_parameters["Projector"] = config.Projector;
    bool_parameters["DisparityMapAOI"] = config.DisparityMapAOI;
    bool_parameters["HDR"] = config.HDR;
    double_parameters["BlackLevelOffset"] = config.BlackLevelOffset;
    double_parameters["Exposure"] = config.Exposure;
    double_parameters["Gain"] = config.Gain;
    int_parameters["PixelClock"] = config.PixelClock;
    int_parameters["TriggerMode"] = config.TriggerMode;
    // Binning only works in 'Software' trigger mode and with the projector on
    if (config.TriggerMode == 0 && config.Projector)
      int_parameters["Binning"] = config.Binning;

    // Stereo parameters
    int_parameters["MinimumDisparity"] = config.MinimumDisparity;
    int_parameters["NumberOfDisparities"] = config.NumberOfDisparities;
    int_parameters["OptimizationProfile"] = config.OptimizationProfile;
    int_parameters["DepthChangeCost"] = config.DepthChangeCost;
    int_parameters["DepthStepCost"] = config.DepthStepCost;
    int_parameters["ShadowingThreshold"] = config.ShadowingThreshold;
    double_parameters["Scaling"] = config.Scaling;

    // Post-processing parameters
    int_parameters["UniquenessRatio"] = config.UniquenessRatio;
    int_parameters["MedianFilterRadius"] = config.MedianFilterRadius;
    int_parameters["SpeckleComponentThreshold"] = config.SpeckleComponentThreshold;
    int_parameters["SpeckleRegionSize"] = config.SpeckleRegionSize;
    int_parameters["FillBorderSpread"] = config.FillBorderSpread;
    int_parameters["FillRegionSize"] = config.FillRegionSize;

    // Streaming parameters
    publish_cloud_ = config.Cloud;
    publish_images_ = config.Images;

    // Setup parameters
    camera_interface_->setupParameters(bool_parameters, int_parameters, double_parameters);
  }
  else
  {
    CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
  }
}

bool Camera::activateHook()
{
  /* This hook should be used to establish connections to the hardware and activate the hardware.
   * It is also here that other interfaces should be initialised
   */

  /* Get a bool indicating if a stereo camera is used */
  nodehandle_.param<bool>("stereo", stereo_, false);

  /* Create nodehandle from namespace parameter */
  std::string cam;
  nodehandle_.param<std::string>("namespace", cam, "left");
  ros::NodeHandle nh(nodehandle_, cam);
  std::string cam_right = "right";
  ros::NodeHandle rnh(nodehandle_, cam_right);

  if (!CameraServiceInterface::configureInterface(cam, stereo_))
  {
    CAROS_FATALERROR("The CAROS CameraServiceInterface could not be configured correctly.",
                     CAROS_CAMERA_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  /* Get a serial number through ros */
  int serial_param;
  nodehandle_.param<int>("basler_serial", serial_param, 0);
  basler_serial_ = static_cast<uint32_t>(serial_param);

  nodehandle_.param<int>("bumblebee_serial", serial_param, 0);
  bumblebee_serial_ = static_cast<uint32_t>(serial_param);

  nodehandle_.param<int>("ensenso_serial", serial_param, 0);
  ensenso_serial_ = static_cast<uint32_t>(serial_param);

  camera_manager_ = std::make_shared<CameraManager>();
  uint64_t guid;

  if (basler_serial_ != 0)
  {
#ifndef BASLER_SUPPORTED
    CAROS_ERROR("A basler camera was specified, but the required driver was not found", REQUIRED_DRIVER_NOT_FOUND);
    return false;
#endif
    ROS_INFO_STREAM("Trying to find Basler camera ...");
    camera_interface_ = camera_manager_->getCamera(basler_serial_, caros::CameraManager::BASLER);

    if (camera_interface_ != nullptr)
    {
      if (camera_interface_->init())
      {
        ROS_INFO_STREAM("Allocated a Basler " << camera_interface_->getName() << " (" << basler_serial_ << ")");
      }
      else
      {
        CAROS_ERROR("Could not initialize a Basler camera with serial: " << basler_serial_,
                    NO_CAMERA_WITH_GIVEN_SERIAL);
        return false;
      }

      camera_interface_->start();

      if (camera_interface_->isRunning())
        ROS_INFO("The basler camera is grabbing!");
      cinfo_name_ << basler_serial_;

      /* Start up the dynamic_reconfigure service for Basler */
      basler_srv_ = std::make_shared<dynamic_reconfigure::Server<caros_camera::BaslerConfig>>(nodehandle_);
      dynamic_reconfigure::Server<caros_camera::BaslerConfig>::CallbackType f =
          boost::bind(&Camera::baslerCallback, this, _1, _2);
      basler_srv_->setCallback(f);
    }
    else
    {
      CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
      return false;
    }
  }
  else if (bumblebee_serial_ != 0)
  {
#ifndef BUMBLEBEE_SUPPORTED
    CAROS_ERROR("A Bumblebee camera was specified, but the required driver was not found", REQUIRED_DRIVER_NOT_FOUND);
    return false;
#endif
    ROS_INFO_STREAM("Trying to find Bumblebee camera ...");
    camera_interface_ = camera_manager_->getCamera(bumblebee_serial_, caros::CameraManager::BUMBLEBEE);
    if (camera_interface_ != nullptr)
    {
      if (camera_interface_->init())
      {
        ROS_INFO_STREAM("Allocated a Bumblebee 2 "
                        << " (" << bumblebee_serial_ << ")");
      }
      else
      {
        CAROS_ERROR("No Bumblebee camera with the given serial number: " << bumblebee_serial_ << " was found!",
                    NO_CAMERA_WITH_GIVEN_SERIAL);
        return false;
      }
      ROS_INFO("Starting camera capture");
      camera_interface_->start();

      cinfo_name_ << bumblebee_serial_;
      rcinfo_name_ << bumblebee_serial_;

      /* Start up the dynamic_reconfigure service for PointGrey / Bumblebee */
      pointgrey_srv_ = std::make_shared<dynamic_reconfigure::Server<caros_camera::PointGreyConfig>>(nodehandle_);
      dynamic_reconfigure::Server<caros_camera::PointGreyConfig>::CallbackType f =
          boost::bind(&Camera::pointGreyCallback, this, _1, _2);
      pointgrey_srv_->setCallback(f);
    }
    else
    {
      CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
      return false;
    }
  }
  else if (ensenso_serial_ != 0)
  {
#ifndef ENSENSO_SUPPORTED
    CAROS_ERROR("An Ensenso camera was specified, but the required driver was not found", REQUIRED_DRIVER_NOT_FOUND);
    return false;
#endif
    ROS_INFO_STREAM("Trying to find Ensenso camera ...");
    camera_interface_ = camera_manager_->getCamera(ensenso_serial_, caros::CameraManager::ENSENSO);
    if (camera_interface_ != nullptr)
    {
      if (camera_interface_->init())
      {
        ROS_INFO_STREAM("Allocated an Ensenso " << camera_interface_->getName() << " (" << ensenso_serial_ << ")");
      }
      else
      {
        CAROS_ERROR("Could not initialize a Ensenso camera with serial: " << ensenso_serial_,
                    NO_CAMERA_WITH_GIVEN_SERIAL);
        return false;
      }

      publish_cloud_ = false;
      publish_images_ = true;

      ROS_INFO("Using default parameters, starting camera");
      camera_interface_->start();

      if (camera_interface_->isRunning())
        ROS_INFO("The Ensenso camera is running!");

      cinfo_name_ << ensenso_serial_;
      rcinfo_name_ << ensenso_serial_;

      /* Start up the dynamic_reconfigure service for Ensenso */
      ensenso_srv_ = std::make_shared<dynamic_reconfigure::Server<caros_camera::EnsensoConfig>>(nodehandle_);
      dynamic_reconfigure::Server<caros_camera::EnsensoConfig>::CallbackType f =
          boost::bind(&Camera::ensensoCallback, this, _1, _2);
      ensenso_srv_->setCallback(f);
    }
    else
    {
      CAROS_FATALERROR("Camera interface failed to initialize, returns nullptr", FAILED_TO_INIT_CAMERA_INTERFACE);
      return false;
    }
  }
  else
  {
    // TODO(prier): make possible to run any available camera
    CAROS_ERROR("No camera parameters have been passed", NO_CAMERA_PARAMETERS_PASSED);
    return false;
  }

  /*  Get camera config yamls based on camera type */
  std::string camera_info_url;
  std::string second_info_url;

  if (stereo_)
  {
    if (camera_manager_->getCameraType() == caros::CameraManager::BUMBLEBEE)
    {
      nodehandle_.param<std::string>("bumblebee_info_url", camera_info_url, "");
      nodehandle_.param<std::string>("bumblebee_right_info_url", second_info_url, "");
    }
    else if (camera_manager_->getCameraType() == caros::CameraManager::ENSENSO)
    {
      nodehandle_.param<std::string>("ensenso_info_url", camera_info_url, "");
      nodehandle_.param<std::string>("ensenso_right_info_url", second_info_url, "");
    }
    else
    {
      CAROS_ERROR("Could not retrieve camera information based on the provided camera type", UNKNOWN_CAMERA_TYPE);
      return false;
    }
  }
  else
  {
    if (camera_manager_->getCameraType() == caros::CameraManager::BASLER)
    {
      nodehandle_.param<std::string>("basler_info_url", camera_info_url, "");
    }
    else if (camera_manager_->getCameraType() == caros::CameraManager::BUMBLEBEE)
    {
      CAROS_ERROR("Stereo camera specified, but it does not match the camera type!", WRONG_CAMERA_TYPE_FOR_STEREO);
      return false;
    }
    else
    {
      CAROS_ERROR("Could not retrieve camera information based on the provided camera type", UNKNOWN_CAMERA_TYPE);
      return false;
    }
  }

  /* Start the camera info manager and attempt to load any configurations */
  cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name_.str(), camera_info_url));
  if (stereo_)
    rcinfo_.reset(new camera_info_manager::CameraInfoManager(rnh, rcinfo_name_.str(), second_info_url));

  if (camera_manager_->getCameraType() == caros::CameraManager::ENSENSO)
  {
    if (!cinfo_->isCalibrated() && !rcinfo_->isCalibrated())
    {
      /* If this is an uncalibrated Ensenso camera, use the internal calibration instead */
      ROS_INFO("Could not find a calibration file for the Ensenso camera, using the internal calibration instead");
      sensor_msgs::CameraInfo linfo, rinfo;
      camera_interface_->getCameraInfo("Left", linfo);
      camera_interface_->getCameraInfo("Right", rinfo);
      cinfo_->setCameraInfo(linfo);
      rcinfo_->setCameraInfo(rinfo);
    }
  }

  /* Get the frame_id parameter */
  nodehandle_.param<std::string>("frame_id", camera_frame_name_, "");

  /* Get and set camera info from the camera info manager */
  ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci_->header.frame_id = camera_frame_name_;
  if (stereo_)
  {
    rci_.reset(new sensor_msgs::CameraInfo(rcinfo_->getCameraInfo()));
    rci_->header.frame_id = camera_frame_name_;
  }

  img_input_ros_image_ = boost::make_shared<sensor_msgs::Image>();
  img_input_r_ros_image_ = boost::make_shared<sensor_msgs::Image>();
  cloud_ = boost::make_shared<sensor_msgs::PointCloud2>();

  img_input_ros_image_->header.frame_id = camera_frame_name_;
  img_input_r_ros_image_->header.frame_id = camera_frame_name_;

  return true;
}

bool Camera::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  bool resolved = false;

  /* Switch on error codes */
  switch (error_code)
  {
    case UNABLE_TO_AQUIRE_IMAGE:
      ros::Duration(5.0).sleep();
      resolved = true;
      break;
    default:
      resolved = false;
  }

  return resolved;
}

void Camera::runLoopHook()
{
  if (stereo_)
  {
    if (camera_manager_->getCameraType() == caros::CameraManager::ENSENSO)
    {
      if (publish_cloud_)
      {
        if (!camera_interface_->grabSingleCloud(cloud_))
        {
          CAROS_ERROR("Unable to acquire PointCloud!", UNABLE_TO_AQUIRE_POINTCLOUD);
        }
      }
      if (publish_images_)
      {
        if (!camera_interface_->getRawImagePair(img_input_ros_image_, img_input_r_ros_image_, timestamp_cam_))
        {
          CAROS_ERROR("Unable to acquire image!", UNABLE_TO_AQUIRE_IMAGE);
        }
      }
    }
    else if (!camera_interface_->getRawImagePair(img_input_ros_image_, img_input_r_ros_image_, timestamp_cam_))
    {
      CAROS_ERROR("Unable to acquire image!", UNABLE_TO_AQUIRE_IMAGE);
    }
  }
  else
  {
    if (!camera_interface_->getRawImage(img_input_ros_image_, timestamp_cam_))
      CAROS_ERROR("Unable to acquire image!", UNABLE_TO_AQUIRE_IMAGE);
  }

  ros::Time capture_time = ros::Time(static_cast<double>(timestamp_cam_ / 1000000.0));
  ci_->header.stamp = capture_time;
  img_input_ros_image_->header.stamp = capture_time;
  if (stereo_)
  {
    rci_->header.stamp = capture_time;
    img_input_r_ros_image_->header.stamp = capture_time;
  }

  if (stereo_)
  {
    /* Publish images and cloud through CameraServiceInterface */
    if (camera_manager_->getCameraType() == caros::CameraManager::ENSENSO)
    {
      if (publish_cloud_)
      {
        cloud_->header.frame_id = camera_frame_name_;
        cloud_->header.stamp = capture_time;
        /* Publish PointCloud through CameraServiceInterface */
        publishCloud(*cloud_);
      }
      if (publish_images_)
      {
        publishImages(*img_input_ros_image_, *img_input_r_ros_image_, *ci_, *rci_);
      }
    }
    else
    {
      publishImages(*img_input_ros_image_, *img_input_r_ros_image_, *ci_, *rci_);
    }
  }
  else
  {
    /* Publish image through CameraServiceInterface */
    publish(*img_input_ros_image_, *ci_);
  }
}

void Camera::errorLoopHook()
{
  /* Not implemented yet */
}

void Camera::fatalErrorLoopHook()
{
  /* Not implemented yet */
}

}  // namespace caros
