#ifndef CAROS_CAMERA_SI_PROXY_H
#define CAROS_CAMERA_SI_PROXY_H

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <string>
#include <deque>

namespace caros
{
/**
 * @brief this class implements a cpp proxy to control and read data from
 * a CameraServiceInterface.
 *
 */
class CameraSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<CameraSIProxy> Ptr;

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] camera_topic The name of the camera topic
   * @param[in] use_persistent_connections Define usage of persistent connections
   */
  CameraSIProxy(ros::NodeHandle nodehandle, const std::string& camera_topic,
                const bool use_persistent_connections = true);

  //! destructor
  virtual ~CameraSIProxy();

  void shoot();

  void retrieveInfo();

  //! get image as OpenCV mat
  cv::Mat getImage();

  //! get camera projection matrix
  cv::Matx34d getProjectionMatrix();

  //! get images as a deque of OpenCV mat
  std::deque<cv::Mat> getImages();

 private:
  void handleImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
 protected:
  ros::NodeHandle nodehandle_;
  std::string ros_namespace_;
  ros::AsyncSpinner async_spinner_;
  ros::CallbackQueue callback_queue_;
  image_transport::CameraSubscriber img_sub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  // variables
  sensor_msgs::ImageConstPtr image_;
  std::deque<cv::Mat> images_;
  image_geometry::PinholeCameraModel cam_model_;
};
}  // namespace caros

#endif  // CAROS_CAMERA_SI_PROXY_H
