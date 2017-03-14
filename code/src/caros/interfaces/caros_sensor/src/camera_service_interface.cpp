#include <caros/camera_service_interface.h>
#include <caros/common.h>

#include <string>
#include <cstddef>

using namespace caros;

CameraServiceInterface::CameraServiceInterface(ros::NodeHandle nodehandle) : nodehandle_(nodehandle)
{
  /* Nothing specific to do */
}

CameraServiceInterface::~CameraServiceInterface()
{
  /* Nothing specific to do */
}

bool CameraServiceInterface::configureInterface(const std::string& cam_namespace, bool stereo)
{
  it_.reset(new image_transport::ImageTransport(nodehandle_));

  if (stereo)
  {
    if (img_raw_pub_ and rimg_raw_pub_ and pc_pub_)
    {
      ROS_WARN_STREAM(
          "Reinitialising one or more CameraServiceInterface services or publishers. If this is not fully intended "
          "then this should be considered a bug!");
    }
  }
  else
  {
    if (img_raw_pub_)
    {
      ROS_WARN_STREAM(
          "Reinitialising one or more CameraServiceInterface services or publishers. If this is not fully intended "
          "then this should be considered a bug!");
    }
  }

  img_raw_pub_ = it_->advertiseCamera(cam_namespace + "/image_raw", CAMERA_PUBLISHER_QUEUE_SIZE);
  if (stereo)
  {
    rimg_raw_pub_ = it_->advertiseCamera("right/image_raw", CAMERA_PUBLISHER_QUEUE_SIZE);
    pc_pub_ =
        nodehandle_.advertise<sensor_msgs::PointCloud2>(cam_namespace + "/pointcloud", CAMERA_PUBLISHER_QUEUE_SIZE);
  }

  if (stereo)
  {
    ROS_ERROR_STREAM_COND(!img_raw_pub_, "The left image raw publisher is empty!");
    ROS_ERROR_STREAM_COND(!rimg_raw_pub_, "The right image raw publisher is empty!");
    ROS_WARN_STREAM_COND(!pc_pub_, "The pointcloud publisher is empty, however this might be intentional");
  }
  else
    ROS_ERROR_STREAM_COND(!img_raw_pub_, "The image raw publisher is empty!");

  if (stereo)
  {
    /* Verify that the various ROS services have actually been created properly */
    if (img_raw_pub_ and rimg_raw_pub_ and pc_pub_)
    {
      /* Everything seems to be properly initialised */
      ROS_DEBUG_STREAM("All CameraServiceInterface publishers and services appear to have been properly initialised");
    }
    else
    {
      ROS_ERROR_STREAM(
          "The camera service could not be properly initialised - one or more ros services or publishers failed "
          "to be "
          "properly initialised.");
      return false;
    }
  }
  else
  {
    /* Verify that the various ROS services have actually been created properly */
    if (img_raw_pub_)
    {
      /* Everything seems to be properly initialised */
      ROS_DEBUG_STREAM("All CameraServiceInterface publishers and services appear to have been properly initialised");
    }
    else
    {
      ROS_ERROR_STREAM(
          "The camera service could not be properly initialised - one or more ros services or publishers failed "
          "to be "
          "properly initialised.");
      return false;
    }
  }

  return true;
}

void CameraServiceInterface::publish(const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& ci_)
{
  img_raw_pub_.publish(image, ci_);
}

void CameraServiceInterface::publishImages(const sensor_msgs::Image& img_left, const sensor_msgs::Image& img_right,
                                           const sensor_msgs::CameraInfo& ci_, const sensor_msgs::CameraInfo& rci_)
{
  img_raw_pub_.publish(img_left, ci_);
  rimg_raw_pub_.publish(img_right, rci_);
}

void CameraServiceInterface::publishCloud(const sensor_msgs::PointCloud2& pc)
{
  pc_pub_.publish(pc);
}
