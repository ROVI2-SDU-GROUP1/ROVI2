#include <caros/ensenso_camera.h>

#include <string>
#include <vector>
#include <map>
#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl
{
void ensensoExceptionHandling(const NxLibException &ex, std::string func_nam)
{
  PCL_ERROR("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str(), ex.getErrorText().c_str(),
            ex.getErrorCode(), ex.getItemPath().c_str());
  if (ex.getErrorCode() == NxLibExecutionFailed)
  {
    NxLibCommand cmd("");
    PCL_WARN("\n%s\n", cmd.result().asJson(true, 4, false).c_str());
  }
}

EnsensoCamera::EnsensoCamera() : caros::CameraInterface()
{
}

EnsensoCamera::EnsensoCamera(std::string serial)
    : caros::CameraInterface(serial), device_open_(false), tcp_open_(false), running_(false)
{
  ROS_INFO("Initialising nxLib");

  try
  {
    nxLibInitialize();
    root_.reset(new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION(pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

EnsensoCamera::~EnsensoCamera()
{
  try
  {
    stop();
    root_.reset();

    if (tcp_open_)
      closeTcpPort();
    nxLibFinalize();
  }
  catch (...)
  {
    // destructor never throws
  }
}

int EnsensoCamera::enumDevices() const
{
  int camera_count = 0;

  try
  {
    NxLibItem cams = NxLibItem("/Cameras/BySerialNo");
    camera_count = cams.count();

    // Print information for all cameras in the tree
    PCL_INFO("Number of connected cameras: %d\n", camera_count);
    PCL_INFO("Serial No    Model   Status\n");

    for (int n = 0; n < cams.count(); ++n)
    {
      PCL_INFO("%s   %s   %s\n", cams[n][itmSerialNumber].asString().c_str(), cams[n][itmModelName].asString().c_str(),
               cams[n][itmStatus].asString().c_str());
    }
    PCL_INFO("\n");
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "enumDevices");
  }
  return (camera_count);
}

bool EnsensoCamera::openDevice(std::string serial_no)
{
  if (device_open_)
    PCL_THROW_EXCEPTION(pcl::IOException, "Cannot open multiple devices!");
  PCL_INFO("Opening Ensenso stereo camera S/N: %s\n", serial_no.c_str());
  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][serial_no];

    if (!camera_.exists() || camera_[itmType] != valStereo)
    {
      PCL_THROW_EXCEPTION(pcl::IOException, "Please connect a single stereo camera to your computer!");
    }

    NxLibCommand open(cmdOpen);
    open.parameters()[itmCameras] = camera_[itmSerialNumber].asString();
    open.execute();
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "openDevice");
    return (false);
  }

  device_open_ = true;
  return (true);
}

bool EnsensoCamera::closeDevice()
{
  if (!device_open_)
    return (false);

  stop();
  PCL_INFO("Closing Ensenso stereo camera\n");

  try
  {
    NxLibCommand(cmdClose).execute();
    device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "closeDevice");
    return (false);
  }
  return (true);
}

void EnsensoCamera::start()
{
  if (isRunning())
    return;

  if (!device_open_)
    openDevice(0);

  frequency_.reset();
  running_ = true;
}

void EnsensoCamera::stop()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback
  }
}

bool EnsensoCamera::isRunning() const
{
  return (running_);
}

bool EnsensoCamera::isTcpPortOpen() const
{
  return (tcp_open_);
}

std::string EnsensoCamera::getName() const
{
  return ("EnsensoCamera");
}

bool stringToBool(const std::string str)
{
  if (str == "true" || str == "True" || str == "TRUE")
    return (true);
  else if (str == "false" || str == "False" || str == "FALSE")
    return (false);
}

bool pcl::EnsensoCamera::setupParameters(std::map<std::string, bool> bool_parameters,
                                         std::map<std::string, int> int_parameters,
                                         std::map<std::string, double> double_parameters)
{
  if (!device_open_)
    return (false);

  try
  {
    // Capture parameters
    NxLibItem captureParams = camera_[itmParameters][itmCapture];
    captureParams[itmAutoBlackLevel].set(bool_parameters["AutoBlackLevel"]);
    captureParams[itmAutoExposure].set(bool_parameters["AutoExposure"]);
    captureParams[itmAutoGain].set(bool_parameters["AutoGain"]);
    captureParams[itmBinning].set(int_parameters["Binning"]);
    captureParams[itmBlackLevelOffset].set(double_parameters["BlackLevelOffset"]);
    captureParams[itmExposure].set(double_parameters["Exposure"]);
    captureParams[itmFrontLight].set(bool_parameters["FrontLight"]);
    captureParams[itmGain].set(double_parameters["Gain"]);
    captureParams[itmGainBoost].set(bool_parameters["GainBoost"]);
    captureParams[itmHardwareGamma].set(bool_parameters["HardwareGamma"]);
    captureParams[itmHdr].set(bool_parameters["HDR"]);
    captureParams[itmPixelClock].set(int_parameters["PixelClock"]);
    captureParams[itmProjector].set(bool_parameters["Projector"]);
    captureParams[itmTargetBrightness].set(int_parameters["TargetBrightness"]);
    std::string trigger_mode;
    switch (int_parameters["TriggerMode"])
    {
      case 0:
        trigger_mode = "Software";
        break;
      case 1:
        trigger_mode = "FallingEdge";
        break;
      case 2:
        trigger_mode = "RisingEdge";
        break;
      default:
        trigger_mode = "Software";
    }
    captureParams[itmTriggerMode].set(trigger_mode);
    captureParams[itmUseDisparityMapAreaOfInterest].set(bool_parameters["DisparityMapAOI"]);

    // Stereo parameters
    NxLibItem stereoParams = camera_[itmParameters][itmDisparityMap][itmStereoMatching];
    stereoParams[itmMinimumDisparity].set(int_parameters["MinimumDisparity"]);
    stereoParams[itmNumberOfDisparities].set(int_parameters["NumberOfDisparities"]);
    std::string profile;
    switch (int_parameters["OptimizationProfile"])
    {
      case 0:
        profile = "Aligned";
        break;
      case 1:
        profile = "Diagonal";
        break;
      case 2:
        profile = "AlignedAndDiagonal";
        break;
      default:
        profile = "AlignedAndDiagonal";
    }
    stereoParams[itmOptimizationProfile].set(profile);
    stereoParams[itmDepthChangeCost].set(int_parameters["DepthChangeCost"]);
    stereoParams[itmDepthStepCost].set(int_parameters["DepthStepCost"]);
    stereoParams[itmShadowingThreshold].set(int_parameters["ShadowingThreshold"]);
    camera_[itmParameters][itmDisparityMap][itmScaling].set(double_parameters["Scaling"]);

    // Post-processing parameters
    NxLibItem postProcessParams = camera_[itmParameters][itmDisparityMap][itmPostProcessing];
    postProcessParams[itmUniquenessRatio].set(int_parameters["UniquenessRatio"]);
    postProcessParams[itmMedianFilterRadius].set(int_parameters["MedianFilterRadius"]);
    postProcessParams[itmSpeckleRemoval][itmComponentThreshold].set(int_parameters["SpeckleComponentThreshold"]);
    postProcessParams[itmSpeckleRemoval][itmRegionSize].set(int_parameters["SpeckleRegionSize"]);
    postProcessParams[itmFilling][itmBorderSpread].set(int_parameters["FillBorderSpread"]);
    postProcessParams[itmFilling][itmRegionSize].set(int_parameters["FillRegionSize"]);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "setupParameters");
    return (false);
  }
  return (true);
}

bool EnsensoCamera::grabSingleCloud(sensor_msgs::PointCloud2Ptr &cloud)
{
  if (!device_open_)
    return (false);

  // if (running_)
  //  return (false);

  try
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    NxLibCommand(cmdCapture).execute();
    // Stereo matching task
    NxLibCommand(cmdComputeDisparityMap).execute();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand(cmdComputePointMap).execute();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo(0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData(pointMap, 0);
    // Copy point pcl_cloud and convert in meters
    pcl_cloud.header.stamp = getPCLStamp(timestamp);
    pcl_cloud.resize(height * width);
    pcl_cloud.width = width;
    pcl_cloud.height = height;
    pcl_cloud.is_dense = false;
    // Copy data in point pcl_cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size(); i += 3)
    {
      pcl_cloud.points[i / 3].x = pointMap[i] / 1000.0;
      pcl_cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      pcl_cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    pcl::toROSMsg(pcl_cloud, *cloud);

    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "grabSingleCloud");
    return (false);
  }
}

bool EnsensoCamera::clearCalibrationPatternBuffer() const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);
  try
  {
    NxLibCommand(cmdDiscardPatterns).execute();
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "clearCalibrationPatternBuffer");
    return (false);
  }
  return (true);
}

int EnsensoCamera::captureCalibrationPattern() const
{
  if (!device_open_)
    return (-1);

  if (running_)
    return (-1);

  try
  {
    NxLibCommand(cmdCapture).execute();
    NxLibCommand collect_pattern(cmdCollectPattern);
    collect_pattern.parameters()[itmBuffer].set(true);  // Store the pattern into the buffer
    collect_pattern.execute();
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "captureCalibrationPattern");
    return (-1);
  }

  return ((*root_)[itmParameters][itmPatternCount].asInt());
}

bool EnsensoCamera::estimateCalibrationPatternPose(Eigen::Affine3d &pattern_pose, const bool average) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand estimate_pattern_pose(cmdEstimatePatternPose);
    estimate_pattern_pose.parameters()[itmAverage].set(average);

    estimate_pattern_pose.execute();
    NxLibItem tf = estimate_pattern_pose.result()[itmPatternPose];
    // Convert tf into a matrix
    if (!jsonTransformationToMatrix(tf.asJson(), pattern_pose))
      return (false);
    pattern_pose.translation() /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}

bool EnsensoCamera::computeCalibrationMatrix(
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> &robot_poses, std::string &json,
    int &iterations, double &reprojection_error, const std::string setup, const std::string target,
    const Eigen::Affine3d &guess_tf, const bool pretty_format) const
{
  if ((*root_)[itmVersion][itmMajor] < 2 && (*root_)[itmVersion][itmMinor] < 3)
    PCL_WARN(
        "EnsensoSDK 1.3.x fixes bugs into extrinsic calibration matrix optimization, please update your SDK!\n"
        "http://www.ensenso.de/support/sdk-download/\n");

  NxLibCommand calibrate(cmdCalibrateHandEye);
  try
  {
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> robot_poses_mm(robot_poses);
    std::vector<std::string> robot_poses_json;
    robot_poses_json.resize(robot_poses.size());
    for (uint i = 0; i < robot_poses_json.size(); ++i)
    {
      robot_poses_mm[i].translation() *= 1000.0;  // Convert meters in millimeters
      if (!matrixTransformationToJson(robot_poses_mm[i], robot_poses_json[i]))
        return (false);
    }
    // Set Hand-Eye calibration parameters
    if (boost::iequals(setup, "Fixed"))
      calibrate.parameters()[itmSetup].set(valFixed);
    else
      calibrate.parameters()[itmSetup].set(valMoving);
    calibrate.parameters()[itmTarget].set(target);

    // Set guess matrix
    if (guess_tf.matrix() != Eigen::Matrix4d::Identity())
    {
      // Matrix > JSON > Angle axis
      NxLibItem tf("/tmpTF");
      if (!matrixTransformationToJson(guess_tf, json))
        return (false);
      tf.setJson(json);

      // Rotation
      double theta = tf[itmRotation][itmAngle].asDouble();  // Angle of rotation
      double x = tf[itmRotation][itmAxis][0].asDouble();    // X component of Euler vector
      double y = tf[itmRotation][itmAxis][1].asDouble();    // Y component of Euler vector
      double z = tf[itmRotation][itmAxis][2].asDouble();    // Z component of Euler vector
      tf.erase();                                           // Delete tmpTF node

      calibrate.parameters()[itmLink][itmRotation][itmAngle].set(theta);
      calibrate.parameters()[itmLink][itmRotation][itmAxis][0].set(x);
      calibrate.parameters()[itmLink][itmRotation][itmAxis][1].set(y);
      calibrate.parameters()[itmLink][itmRotation][itmAxis][2].set(z);
      // Translation
      calibrate.parameters()[itmLink][itmTranslation][0].set(guess_tf.translation()[0] * 1000.0);
      calibrate.parameters()[itmLink][itmTranslation][1].set(guess_tf.translation()[1] * 1000.0);
      calibrate.parameters()[itmLink][itmTranslation][2].set(guess_tf.translation()[2] * 1000.0);
    }

    // Feed all robot poses into the calibration command
    for (uint i = 0; i < robot_poses_json.size(); ++i)
    {
      // Very weird behavior here:
      // If you modify this loop, check that all the transformations are still here in the [itmExecute][itmParameters]
      // node
      // because for an unknown reason sometimes the old transformations are erased in the tree ("null" in the tree)
      // Ensenso SDK 2.3.348: If not moved after guess calibration matrix, the vector is empty.
      calibrate.parameters()[itmTransformations][i].setJson(robot_poses_json[i], false);
    }

    calibrate.execute();  // Might take up to 120 sec (maximum allowed by Ensenso API)

    if (calibrate.successful())
    {
      json = calibrate.result()[itmLink].asJson(pretty_format);
      iterations = calibrate.result()[itmIterations].asInt();
      reprojection_error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_INFO("computeCalibrationMatrix succeeded. Iterations: %d, Reprojection error: %.2f", iterations,
               reprojection_error);
      ROS_INFO_STREAM("Result: " << std::endl << json);
      return (true);
    }
    else
    {
      json.clear();
      return (false);
    }
  }
  catch (NxLibException &ex)
  {
    try
    {
      int iters = calibrate.result()[itmIterations].asInt();
      double error = calibrate.result()[itmReprojectionError].asDouble();
      ROS_WARN("computeCalibrationMatrix Failed. Iterations: %d, Reprojection error: %.2f", iters, error);
      ROS_WARN_STREAM("Result: " << std::endl << calibrate.result()[itmLink].asJson(true));
      return (false);
    }
    catch (...)
    {
      pcl::ensensoExceptionHandling(ex, "computeCalibrationMatrix");
    }
  }
}

bool EnsensoCamera::storeEEPROMExtrinsicCalibration() const
{
  try
  {
    NxLibCommand store(cmdStoreCalibration);
    store.execute();
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "storeEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool EnsensoCamera::loadEEPROMExtrinsicCalibration() const
{
  try
  {
    NxLibCommand load(cmdLoadCalibration);
    load.parameters()[itmCameras] = camera_[itmSerialNumber].asString();
    load.execute();
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "loadEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool EnsensoCamera::clearEEPROMExtrinsicCalibration()
{
  try
  {
    setExtrinsicCalibration("");
    NxLibCommand store(cmdStoreCalibration);
    store.execute();
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "clearEEPROMExtrinsicCalibration");
    return (false);
  }
}

bool EnsensoCamera::setExtrinsicCalibration(const double euler_angle, Eigen::Vector3d &rotation_axis,
                                            const Eigen::Vector3d &translation, const std::string target)
{
  if (!device_open_)
    return (false);

  if (rotation_axis != Eigen::Vector3d(0, 0, 0))
    rotation_axis.normalize();

  try
  {
    NxLibItem calibParams = camera_[itmLink];
    calibParams[itmTarget].set(target);
    calibParams[itmRotation][itmAngle].set(euler_angle);

    calibParams[itmRotation][itmAxis][0].set(rotation_axis[0]);
    calibParams[itmRotation][itmAxis][1].set(rotation_axis[1]);
    calibParams[itmRotation][itmAxis][2].set(rotation_axis[2]);

    calibParams[itmTranslation][0].set(translation[0] * 1000.0);  // Convert in millimeters
    calibParams[itmTranslation][1].set(translation[1] * 1000.0);
    calibParams[itmTranslation][2].set(translation[2] * 1000.0);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "setExtrinsicCalibration");
    return (false);
  }
  return (true);
}

bool EnsensoCamera::setExtrinsicCalibration(const std::string target)
{
  if (!device_open_)
    return (false);

  Eigen::Vector3d rotation(Eigen::Vector3d::Zero());
  Eigen::Vector3d translation(Eigen::Vector3d::Zero());
  return (setExtrinsicCalibration(0.0, rotation, translation, target));
}

bool EnsensoCamera::setExtrinsicCalibration(const Eigen::Affine3d &transformation, const std::string target)
{
  std::string json;
  if (!matrixTransformationToJson(transformation, json))
    return (false);

  double euler_angle;
  Eigen::Vector3d rotation_axis;
  Eigen::Vector3d translation;

  if (!jsonTransformationToAngleAxis(json, euler_angle, rotation_axis, translation))
    return (false);

  return (setExtrinsicCalibration(euler_angle, rotation_axis, translation, target));
}

float EnsensoCamera::getFramesPerSecond() const
{
  boost::mutex::scoped_lock lock(fps_mutex_);
  return (frequency_.getFrequency());
}

bool EnsensoCamera::openTcpPort(const int port)
{
  try
  {
    nxLibOpenTcpPort(port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "openTcpPort");
    return (false);
  }
  return (true);
}

bool EnsensoCamera::closeTcpPort()
{
  try
  {
    nxLibCloseTcpPort();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

std::string EnsensoCamera::getTreeAsJson(const bool pretty_format) const
{
  try
  {
    return (root_->asJson(pretty_format));
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "getTreeAsJson");
    return ("");
  }
}

std::string EnsensoCamera::getResultAsJson(const bool pretty_format) const
{
  try
  {
    NxLibCommand cmd("");
    return (cmd.result().asJson(pretty_format));
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "getResultAsJson");
    return ("");
  }
}

double EnsensoCamera::getPatternGridSpacing() const
{
  if (!device_open_)
    return (-1);
  if (running_)
    return (-1);
  NxLibCommand collect_pattern(cmdCollectPattern);
  try
  {
    NxLibCommand(cmdCapture).execute();
    collect_pattern.parameters()[itmBuffer].set(false);
    collect_pattern.parameters()[itmDecodeData].set(true);
    collect_pattern.execute();
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "getPatternGridSpacing");
    return (-1);
  }

  return collect_pattern.result()[itmGridSpacing].asDouble();
}

bool EnsensoCamera::enableFrontLight(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmFrontLight].set(enable);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "enableFrontLight");
    return (false);
  }
  return (true);
}

bool EnsensoCamera::enableProjector(const bool enable) const
{
  try
  {
    camera_[itmParameters][itmCapture][itmProjector].set(enable);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "enableProjector");
    return (false);
  }
  return (true);
}

bool EnsensoCamera::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {
    cam_info.width = camera_[itmSensor][itmSize][0].asInt();
    cam_info.height = camera_[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors
    cam_info.D.resize(5);
    for (std::size_t i = 0; i < cam_info.D.size(); ++i)
      cam_info.D[i] = camera_[itmCalibration][itmMonocular][cam][itmDistortion][i].asDouble();
    // K and R matrices
    for (std::size_t i = 0; i < 3; ++i)
    {
      for (std::size_t j = 0; j < 3; ++j)
      {
        cam_info.K[3 * i + j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        cam_info.R[3 * i + j] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmRotation][j][i].asDouble();
      }
    }
    cam_info.P[0] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][0].asDouble();
    cam_info.P[1] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][0].asDouble();
    cam_info.P[2] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][0].asDouble();
    cam_info.P[3] = 0.0;
    cam_info.P[4] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][0][1].asDouble();
    cam_info.P[5] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][1][1].asDouble();
    cam_info.P[6] = camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera][2][1].asDouble();
    cam_info.P[7] = 0.0;
    cam_info.P[10] = 1.0;
    if (cam == "Right")
    {
      double B = camera_[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "getCameraInfo");
    return false;
  }
}

bool EnsensoCamera::jsonTransformationToEulerAngles(const std::string &json, double &x, double &y, double &z, double &w,
                                                    double &p, double &r) const
{
  try
  {
    NxLibCommand convert(cmdConvertTransformation);
    convert.parameters()[itmTransformation].setJson(json, false);
    convert.parameters()[itmSplitRotation].set(valXYZ);
    convert.execute();

    NxLibItem tf = convert.result()[itmTransformations];
    x = tf[0][itmTranslation][0].asDouble();
    y = tf[0][itmTranslation][1].asDouble();
    z = tf[0][itmTranslation][2].asDouble();
    r = tf[0][itmRotation][itmAngle].asDouble();  // Roll
    p = tf[1][itmRotation][itmAngle].asDouble();  // Pitch
    w = tf[2][itmRotation][itmAngle].asDouble();  // yaW
    return (true);
  }

  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "jsonTransformationToEulerAngles");
    return (false);
  }
}

bool EnsensoCamera::jsonTransformationToAngleAxis(const std::string json, double &alpha, Eigen::Vector3d &axis,
                                                  Eigen::Vector3d &translation) const
{
  try
  {
    NxLibItem tf("/tmpTF");
    tf.setJson(json);
    translation[0] = tf[itmTranslation][0].asDouble();
    translation[1] = tf[itmTranslation][1].asDouble();
    translation[2] = tf[itmTranslation][2].asDouble();

    alpha = tf[itmRotation][itmAngle].asDouble();      // Angle of rotation
    axis[0] = tf[itmRotation][itmAxis][0].asDouble();  // X component of Euler vector
    axis[1] = tf[itmRotation][itmAxis][1].asDouble();  // Y component of Euler vector
    axis[2] = tf[itmRotation][itmAxis][2].asDouble();  // Z component of Euler vector
    tf.erase();                                        // Delete tmpTF node
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "jsonTransformationToAngleAxis");
    return (false);
  }
}

bool EnsensoCamera::jsonTransformationToMatrix(const std::string transformation, Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert_transformation(cmdConvertTransformation);
    convert_transformation.parameters()[itmTransformation].setJson(transformation);
    convert_transformation.execute();
    Eigen::Affine3d tmp(Eigen::Affine3d::Identity());

    // Rotation
    tmp.linear().col(0) = Eigen::Vector3d(convert_transformation.result()[itmTransformation][0][0].asDouble(),
                                          convert_transformation.result()[itmTransformation][0][1].asDouble(),
                                          convert_transformation.result()[itmTransformation][0][2].asDouble());

    tmp.linear().col(1) = Eigen::Vector3d(convert_transformation.result()[itmTransformation][1][0].asDouble(),
                                          convert_transformation.result()[itmTransformation][1][1].asDouble(),
                                          convert_transformation.result()[itmTransformation][1][2].asDouble());

    tmp.linear().col(2) = Eigen::Vector3d(convert_transformation.result()[itmTransformation][2][0].asDouble(),
                                          convert_transformation.result()[itmTransformation][2][1].asDouble(),
                                          convert_transformation.result()[itmTransformation][2][2].asDouble());

    // Translation
    tmp.translation() = Eigen::Vector3d(convert_transformation.result()[itmTransformation][3][0].asDouble(),
                                        convert_transformation.result()[itmTransformation][3][1].asDouble(),
                                        convert_transformation.result()[itmTransformation][3][2].asDouble());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "jsonTransformationToMatrix");
    return (false);
  }
}

bool EnsensoCamera::eulerAnglesTransformationToJson(const double x, const double y, const double z, const double w,
                                                    const double p, const double r, std::string &json,
                                                    const bool pretty_format) const
{
  try
  {
    NxLibCommand chain(cmdChainTransformations);
    NxLibItem tf = chain.parameters()[itmTransformations];

    if (!angleAxisTransformationToJson(x, y, z, 0, 0, 1, r, json))
      return (false);
    tf[0].setJson(json, false);  // Roll

    if (!angleAxisTransformationToJson(0, 0, 0, 0, 1, 0, p, json))
      return (false);
    tf[1].setJson(json, false);  // Pitch

    if (!angleAxisTransformationToJson(0, 0, 0, 1, 0, 0, w, json))
      return (false);
    tf[2].setJson(json, false);  // yaW

    chain.execute();
    json = chain.result()[itmTransformation].asJson(pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "eulerAnglesTransformationToJson");
    return (false);
  }
}

bool EnsensoCamera::angleAxisTransformationToJson(const double x, const double y, const double z, const double rx,
                                                  const double ry, const double rz, const double alpha,
                                                  std::string &json, const bool pretty_format) const
{
  try
  {
    NxLibItem tf("/tmpTF");
    tf[itmTranslation][0].set(x);
    tf[itmTranslation][1].set(y);
    tf[itmTranslation][2].set(z);

    tf[itmRotation][itmAngle].set(alpha);  // Angle of rotation
    tf[itmRotation][itmAxis][0].set(rx);   // X component of Euler vector
    tf[itmRotation][itmAxis][1].set(ry);   // Y component of Euler vector
    tf[itmRotation][itmAxis][2].set(rz);   // Z component of Euler vector

    json = tf.asJson(pretty_format);
    tf.erase();
    return (true);
  }

  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "angleAxisTransformationToJson");
    return (false);
  }
}

bool EnsensoCamera::matrixTransformationToJson(const Eigen::Affine3d &matrix, std::string &json,
                                               const bool pretty_format) const
{
  try
  {
    NxLibCommand convert(cmdConvertTransformation);

    // Rotation
    convert.parameters()[itmTransformation][0][0].set(matrix.linear().col(0)[0]);
    convert.parameters()[itmTransformation][0][1].set(matrix.linear().col(0)[1]);
    convert.parameters()[itmTransformation][0][2].set(matrix.linear().col(0)[2]);
    convert.parameters()[itmTransformation][0][3].set(0.0);

    convert.parameters()[itmTransformation][1][0].set(matrix.linear().col(1)[0]);
    convert.parameters()[itmTransformation][1][1].set(matrix.linear().col(1)[1]);
    convert.parameters()[itmTransformation][1][2].set(matrix.linear().col(1)[2]);
    convert.parameters()[itmTransformation][1][3].set(0.0);

    convert.parameters()[itmTransformation][2][0].set(matrix.linear().col(2)[0]);
    convert.parameters()[itmTransformation][2][1].set(matrix.linear().col(2)[1]);
    convert.parameters()[itmTransformation][2][2].set(matrix.linear().col(2)[2]);
    convert.parameters()[itmTransformation][2][3].set(0.0);

    // Translation
    convert.parameters()[itmTransformation][3][0].set(matrix.translation()[0]);
    convert.parameters()[itmTransformation][3][1].set(matrix.translation()[1]);
    convert.parameters()[itmTransformation][3][2].set(matrix.translation()[2]);
    convert.parameters()[itmTransformation][3][3].set(1.0);

    convert.execute();
    json = convert.result()[itmTransformation].asJson(pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "matrixTransformationToJson");
    return (false);
  }
}

pcl::uint64_t EnsensoCamera::getPCLStamp(const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ((ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

std::string EnsensoCamera::getOpenCVType(const int channels, const int bpe, const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str(boost::format("CV_%i%cC%i") % bits % type % channels));
}

void EnsensoCamera::processGrabbing()
{
  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if (num_slots<sig_cb_ensenso_point_cloud>() > 0 || num_slots<sig_cb_ensenso_images>() > 0 ||
          num_slots<sig_cb_ensenso_point_cloud_images>() > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<PairOfImages> rawimages(new PairOfImages);
        boost::shared_ptr<PairOfImages> rectifiedimages(new PairOfImages);
        fps_mutex_.lock();
        frequency_.event();
        fps_mutex_.unlock();

        NxLibCommand(cmdCapture).execute();
        double timestamp;
        camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo(0, 0, 0, 0, 0, &timestamp);

        // Gather images
        if (num_slots<sig_cb_ensenso_images>() > 0 || num_slots<sig_cb_ensenso_point_cloud_images>() > 0)
        {
          // Rectify images
          NxLibCommand(cmdRectifyImages).execute();
          int width, height, channels, bpe;
          bool isFlt, collected_pattern = false;

          try  // Try to collect calibration pattern, if not possible, publish RAW images and Rectified images instead
          {
            NxLibCommand collect_pattern(cmdCollectPattern);
            collect_pattern.parameters()[itmBuffer].set(false);  // Do NOT store the pattern into the buffer!
            collect_pattern.execute();
            collected_pattern = true;
          }
          catch (const NxLibException &ex)
          {
            // We failed to collect the pattern but the RAW images are available!
          }

          if (collected_pattern)
          {
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp(timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize(width * height * sizeof(float));
            rawimages->second.data.resize(width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType(channels, bpe, isFlt);
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData(rawimages->first.data.data(),
                                                                      rawimages->first.data.size(), 0, 0);
            camera_[itmImages][itmWithOverlay][itmRight].getBinaryData(rawimages->second.data.data(),
                                                                       rawimages->second.data.size(), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize(width * height * sizeof(float));
            rectifiedimages->second.data.resize(width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType(channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData(rectifiedimages->first.data.data(),
                                                                    rectifiedimages->first.data.size(), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData(rectifiedimages->second.data.data(),
                                                                     rectifiedimages->second.data.size(), 0, 0);
          }
          else
          {
            camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
            rawimages->first.header.stamp = rawimages->second.header.stamp = getPCLStamp(timestamp);
            rawimages->first.width = rawimages->second.width = width;
            rawimages->first.height = rawimages->second.height = height;
            rawimages->first.data.resize(width * height * sizeof(float));
            rawimages->second.data.resize(width * height * sizeof(float));
            rawimages->first.encoding = rawimages->second.encoding = getOpenCVType(channels, bpe, isFlt);
            camera_[itmImages][itmRaw][itmLeft].getBinaryData(rawimages->first.data.data(),
                                                              rawimages->first.data.size(), 0, 0);
            camera_[itmImages][itmRaw][itmRight].getBinaryData(rawimages->second.data.data(),
                                                               rawimages->second.data.size(), 0, 0);
            // rectifiedimages
            camera_[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
            rectifiedimages->first.width = rectifiedimages->second.width = width;
            rectifiedimages->first.height = rectifiedimages->second.height = height;
            rectifiedimages->first.data.resize(width * height * sizeof(float));
            rectifiedimages->second.data.resize(width * height * sizeof(float));
            rectifiedimages->first.encoding = rectifiedimages->second.encoding = getOpenCVType(channels, bpe, isFlt);
            camera_[itmImages][itmRectified][itmLeft].getBinaryData(rectifiedimages->first.data.data(),
                                                                    rectifiedimages->first.data.size(), 0, 0);
            camera_[itmImages][itmRectified][itmRight].getBinaryData(rectifiedimages->second.data.data(),
                                                                     rectifiedimages->second.data.size(), 0, 0);
          }
        }

        // Gather point cloud
        if (num_slots<sig_cb_ensenso_point_cloud>() > 0 || num_slots<sig_cb_ensenso_point_cloud_images>() > 0)
        {
          // Stereo matching task
          NxLibCommand(cmdComputeDisparityMap).execute();
          // Convert disparity map into XYZ data for each pixel
          NxLibCommand(cmdComputePointMap).execute();
          // Get info about the computed point map and copy it into a std::vector
          std::vector<float> pointMap;
          int width, height;
          camera_[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData(pointMap, 0);
          // Copy point cloud and convert in meters
          cloud->header.stamp = getPCLStamp(timestamp);
          cloud->points.resize(height * width);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;
          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size(); i += 3)
          {
            cloud->points[i / 3].x = pointMap[i] / 1000.0;
            cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
            cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }
        }
      }
      continue_grabbing = running_;
    }
    catch (NxLibException &ex)
    {
      pcl::ensensoExceptionHandling(ex, "processGrabbing");
    }
  }
}

bool EnsensoCamera::init()
{
  ROS_INFO_STREAM("Init calling openDevice with serial: " << serial_);
  openDevice(serial_);
  openTcpPort();
  return true;
}

sensor_msgs::ImagePtr pclImgToOpenCV(pcl::PCLImage pcl_image)
{
  unsigned char *image_array = reinterpret_cast<unsigned char *>(&pcl_image.data[0]);
  int type(CV_8UC1);
  std::string encoding("mono8");
  if (pcl_image.encoding == "CV_8UC3")
  {
    type = CV_8UC3;
    encoding = "bgr8";
  }
  cv::Mat image_mat(pcl_image.height, pcl_image.width, type, image_array);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  return cv_bridge::CvImage(header, encoding, image_mat).toImageMsg();
}

bool EnsensoCamera::getRawImagePair(sensor_msgs::ImagePtr &img_left, sensor_msgs::ImagePtr &img_right,
                                    uint64_t &timestamp)
{
  try
  {
    // Publish images
    boost::shared_ptr<PairOfImages> raw_images(new PairOfImages);

    NxLibCommand(cmdCapture).execute();
    double time_stamp;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo(0, 0, 0, 0, 0, &time_stamp);

    // Gather images
    int width, height, channels, bpe;
    bool isFlt = false;

    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo(&width, &height, &channels, &bpe, &isFlt, 0);
    raw_images->first.header.stamp = raw_images->second.header.stamp = getPCLStamp(time_stamp);
    raw_images->first.width = raw_images->second.width = width;
    raw_images->first.height = raw_images->second.height = height;
    raw_images->first.data.resize(width * height * sizeof(float));
    raw_images->second.data.resize(width * height * sizeof(float));
    raw_images->first.encoding = raw_images->second.encoding = getOpenCVType(channels, bpe, isFlt);
    camera_[itmImages][itmRaw][itmLeft].getBinaryData(raw_images->first.data.data(), raw_images->first.data.size(), 0,
                                                      0);
    camera_[itmImages][itmRaw][itmRight].getBinaryData(raw_images->second.data.data(), raw_images->second.data.size(),
                                                       0, 0);

    img_left = pclImgToOpenCV(raw_images->first);
    img_right = pclImgToOpenCV(raw_images->second);
  }
  catch (NxLibException &ex)
  {
    pcl::ensensoExceptionHandling(ex, "getRawImagePair");
  }

  return true;
}

bool EnsensoCamera::shutdown()
{
  if (closeTcpPort())
  {
    if (closeDevice())
      return true;
  }
  return false;
}

}  // namespace pcl
