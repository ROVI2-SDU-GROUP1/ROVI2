#ifndef CAROS_CAMERA_SERVICE_INTERFACE_H
#define CAROS_CAMERA_SERVICE_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>

#include <memory>
#include <string>

#define CAMERA_PUBLISHER_QUEUE_SIZE 1

namespace caros
{
/**
 * @brief describe the minimum interface of a camera.
 */
class CameraServiceInterface
{
 public:
  //! pointer type
  typedef std::shared_ptr<CameraServiceInterface> Ptr;

  //! constructor
  explicit CameraServiceInterface(ros::NodeHandle nodehandle);

  //! destructor
  virtual ~CameraServiceInterface();

 protected:
  //! initialize ros interface, cam_namespace can be used to specify 'left' or 'right' camera
  bool configureInterface(const std::string &cam_namespace, bool stereo);

  //! publish the image
  void publish(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& ci_);

  //! publish the images
  void publishImages(const sensor_msgs::Image& img_left, const sensor_msgs::Image& img_right,
                     const sensor_msgs::CameraInfo& ci_, const sensor_msgs::CameraInfo& rci_);

  //! publish the pointcloud
  void publishCloud(const sensor_msgs::PointCloud2& pc);

 private:
  CameraServiceInterface()
  {
    /* Empty */
  }

  ros::NodeHandle nodehandle_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher img_raw_pub_;
  image_transport::CameraPublisher rimg_raw_pub_;
  ros::Publisher pc_pub_;
};
}  // namespace caros

#endif  // CAROS_CAMERA_SERVICE_INTERFACE_H
