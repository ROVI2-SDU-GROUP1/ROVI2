#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

ros::Subscriber sub;
ros::Publisher pub;

// state subscriber
void find2DPose(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;

  try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat image = cv_ptr->image.clone();

  cv::imshow("test", image);
  cv::waitKey(1);

  // Do image detection // TODO

  geometry_msgs::PointStamped point;
	pub.publish(point);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "image_detection");
	ros::NodeHandle nh;
	ros::Rate rate(20);

  sub = nh.subscribe<sensor_msgs::Image>("/camera/left/image_raw", 1, find2DPose);
  pub = nh.advertise<geometry_msgs::PointStamped>("/pose/2d_left", 1);

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
