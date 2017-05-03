#include <caros/camera_si_proxy.h>

#include <string>
#include <deque>
#include <condition_variable>

using namespace caros;

CameraSIProxy::CameraSIProxy(ros::NodeHandle nodehandle, const std::string& camera_topic,
                             const bool use_persistent_connections)
    : nodehandle_(nodehandle), ros_namespace_(camera_topic), async_spinner_(1, &callback_queue_)
{
  nodehandle_.setCallbackQueue(&callback_queue_);
  async_spinner_.start();
}

CameraSIProxy::~CameraSIProxy()
{
}

void CameraSIProxy::handleImage(const sensor_msgs::ImageConstPtr& img_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(img_msg);
  images_.push_back(cv_image->image.clone());
  image_ = img_msg;
  cam_model_.fromCameraInfo(info_msg);
  // image received shutdown sub
  img_sub_.shutdown();
  cv_.notify_one();
}

void CameraSIProxy::shoot()
{
  std::unique_lock<std::mutex> lk(mutex_);
  images_.clear();
  image_transport::ImageTransport it(nodehandle_);
  img_sub_ = it.subscribeCamera(ros_namespace_ + "/image_rect_color", 1, &CameraSIProxy::handleImage, this);
  cv_.wait(lk, [this](){return image_ != nullptr; });
}

void CameraSIProxy::retrieveInfo()
{
  std::unique_lock<std::mutex> lk(mutex_);
  images_.clear();
  image_transport::ImageTransport it(nodehandle_);
  img_sub_ = it.subscribeCamera(ros_namespace_ + "/image_rect_color", 1, &CameraSIProxy::handleImage, this);
  cv_.wait(lk, [this](){return image_ != nullptr; });
}

cv::Matx34d CameraSIProxy::getProjectionMatrix()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Matx34d p_matrix = cam_model_.fullProjectionMatrix();
  return p_matrix;
}

cv::Mat CameraSIProxy::getImage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat image = images_.back().clone();
  return image;
}

std::deque<cv::Mat> CameraSIProxy::getImages()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::deque<cv::Mat> images;

  for (const auto& image : images_)
    images.push_back(image.clone());

  return images;
}
