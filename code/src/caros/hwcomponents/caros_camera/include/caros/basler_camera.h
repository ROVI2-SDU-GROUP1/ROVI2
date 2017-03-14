#ifndef CAROS_BASLER_CAMERA_H
#define CAROS_BASLER_CAMERA_H

#include <caros/camera_interface.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// GigE (Pylon)
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/_BaslerGigECameraParams.h>

#include <string>

namespace caros
{
class BaslerCamera : public CameraInterface
{
 public:
  // constructors
  BaslerCamera();
  explicit BaslerCamera(std::string serial);
  ~BaslerCamera();

  GenApi::INodeMap& setup(Pylon::CInstantCamera& camera);

  bool setupParameters(std::unordered_map<std::string, std::string> parameters,
                       std::map<std::string, bool> bool_parameters,
                       std::map<std::string, int> int_parameters,
                       std::map<std::string, double> double_parameters);

  std::string getName() const;

  bool getRawImage(sensor_msgs::ImagePtr& img, uint64_t& timestamp);

  bool init();

  void start();

  void stop();

  bool shutdown();

  bool isRunning() const;

 protected:
  Pylon::CInstantCamera gige_camera_;
  bool gige_camera_initialized_;

  Pylon::DeviceInfoList_t gige_cam_list_;
  int nr_of_gige_cameras_;
};
}  // namespace caros

#endif  // CAROS_BASLER_CAMERA_H
