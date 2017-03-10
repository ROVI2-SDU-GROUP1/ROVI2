#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter(std::string cameraName, std::string _windowName)
    : it_(nh_), windowName(_windowName)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(cameraName, 1, &ImageConverter::imageCb, this);
    cv::namedWindow(windowName);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(windowName);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(windowName, cv_ptr->image);
    cv::waitKey(3);
  }

  private:
	std::string windowName;
};

int main(int argc, char** argv)
{
  std::string windowLeft = "Image window left";
  std::string windowRight = "Image window right";

  ros::init(argc, argv, "image_converter");
  ImageConverter ic_l("/camera/left/image_color", windowLeft);
  ImageConverter ic_r("/camera/right/image_color", windowRight)

  ros::spin();
  return 0;
}

