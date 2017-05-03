#ifndef CAROS_CAMERA_INTERFACE_H
#define CAROS_CAMERA_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <unordered_map>
#include <map>

#ifdef BUMBLEBEE_SUPPORTED
#include <caros_camera/PointGreyConfig.h>
#endif

namespace caros
{
class CameraInterface
{
 public:
  // constructors
  CameraInterface();
  // fw constructor
  explicit CameraInterface(unsigned int serial);
  // gige constructor
  explicit CameraInterface(const std::string& serial);

  virtual ~CameraInterface();

  virtual bool init() = 0;

  virtual void start() = 0;

  virtual bool shutdown() = 0;

  virtual std::string getName() const = 0;

  virtual void stop()
  {
  }
  virtual bool getRawImage(sensor_msgs::ImagePtr& img, uint64_t& timestamp)
  {
  }
  virtual bool getRawImagePair(sensor_msgs::ImagePtr& img_left, sensor_msgs::ImagePtr& img_right, uint64_t& timestamp)
  {
  }
  virtual bool grabSingleCloud(sensor_msgs::PointCloud2Ptr& cloud)
  {
  }
  virtual bool isRunning() const
  {
  }
  virtual bool setupParameters(std::unordered_map<std::string, std::string> parameters,
                               std::map<std::string, bool> bool_parameters, std::map<std::string, int> int_parameters,
                               std::map<std::string, double> double_parameters)
  {
  }
  virtual bool setupParameters(std::map<std::string, bool> bool_parameters, std::map<std::string, int> int_parameters,
                               std::map<std::string, double> double_parameters)
  {
  }
#ifdef BUMBLEBEE_SUPPORTED
  virtual bool setNewConfiguration(caros_camera::PointGreyConfig& config, const uint32_t& level)
  {
  }
#endif
  virtual bool getCameraInfo(std::string cam, sensor_msgs::CameraInfo& cam_info) const
  {
  }

 protected:
  // GigE
  std::string serial_;
  // FireWire
  unsigned int fw_serial_;
};
}  // namespace caros

#endif  // CAROS_CAMERA_INTERFACE_H
