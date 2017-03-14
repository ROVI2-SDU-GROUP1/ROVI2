#ifndef CAROS_CAMERA_H
#define CAROS_CAMERA_H

#include <caros/caros_node_service_interface.h>
#include <caros/camera_manager.h>
#include <caros/camera_service_interface.h>
#include <dynamic_reconfigure/server.h>
#include <caros_camera/BaslerConfig.h>
#include <caros_camera/EnsensoConfig.h>
#include <caros_camera/PointGreyConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <unordered_map>
#include <string>

namespace caros
{
class Camera : public caros::CarosNodeServiceInterface, public caros::CameraServiceInterface
{
 public:
  explicit Camera(const ros::NodeHandle& nodehandle);

  virtual ~Camera();

  enum CAMERANODE_ERRORCODE
  {
    NO_CAMERA_PARAMETERS_PASSED = 0,
    NO_CAMERA_WITH_GIVEN_SERIAL,
    UNABLE_TO_AQUIRE_IMAGE,
    UNABLE_TO_AQUIRE_POINTCLOUD,
    FAILED_TO_INIT_CAMERA,
    CAROS_CAMERA_SERVICE_CONFIGURE_FAIL,
    UNKNOWN_CAMERA_TYPE,
    WRONG_CAMERA_TYPE_FOR_STEREO,
    FAILED_TO_INIT_CAMERA_INTERFACE,
    UNABLE_TO_SET_PARAMETERS,
    REQUIRED_DRIVER_NOT_FOUND
  };

 protected:
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  void pointGreyCallback(caros_camera::PointGreyConfig& config, uint32_t level);
  void baslerCallback(caros_camera::BaslerConfig& config, uint32_t level);
  void ensensoCallback(caros_camera::EnsensoConfig& config, uint32_t level);

private:
  // Parameters
  uint32_t basler_serial_;
  uint32_t pike_serial_;
  uint32_t bumblebee_serial_;
  uint32_t ensenso_serial_;
  bool stereo_;
  bool publish_cloud_;
  bool publish_images_;

  ros::NodeHandle nodehandle_;
  std::shared_ptr<CameraInterface> camera_interface_;
  std::shared_ptr<CameraManager> camera_manager_;

  sensor_msgs::CameraInfoPtr ci_;
  sensor_msgs::CameraInfoPtr rci_;
  std::string camera_frame_name_;
  std::stringstream cinfo_name_;
  std::stringstream rcinfo_name_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> rcinfo_;
  std::shared_ptr<dynamic_reconfigure::Server<caros_camera::BaslerConfig>> basler_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<caros_camera::EnsensoConfig>> ensenso_srv_;
  std::shared_ptr<dynamic_reconfigure::Server<caros_camera::PointGreyConfig>> pointgrey_srv_;

  // Image and parameters
  int input_width_;
  int input_height_;
  sensor_msgs::ImagePtr img_input_ros_image_;
  sensor_msgs::ImagePtr img_input_r_ros_image_;
  sensor_msgs::PointCloud2Ptr cloud_;
  uint64_t timestamp_cam_;
};
}  // namespace caros

#endif  // CAROS_CAMERA_H
