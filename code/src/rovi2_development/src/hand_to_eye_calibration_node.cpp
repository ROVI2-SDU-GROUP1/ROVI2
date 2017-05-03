#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;
geometry_msgs::PointStamped pose3D;

cv::Mat imageLeft;
cv::Mat imageRight;

void calc3DPose(){
  std::cout << "TEST TEST TEST" << std::endl;
}

void CallBackFuncLeft(int event, int x, int y, int flags, void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DLeft.point.x = x;
    pose2DLeft.point.y = y;
    std::cout << "Left mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}

void CallBackFuncRight(int event, int x, int y, int flags, void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DRight.point.x = x;
    pose2DRight.point.y = y;
    std::cout << "Right mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}

void findPointsInImages(){
  cv::setMouseCallback("Left image", CallBackFuncLeft, NULL);
  cv::imshow("Left image", imageLeft);

  cv::setMouseCallback("Right image", CallBackFuncRight, NULL);
  cv::imshow("Right image", imageRight);

  calc3DPose();
}

void image_sync_callback(const sensor_msgs::Image::ConstPtr &image_left, const sensor_msgs::Image::ConstPtr &image_right){
  cv_bridge::CvImagePtr cv_ptr_left;
  cv_bridge::CvImagePtr cv_ptr_right;

  try{
    cv_ptr_left = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::BGR8);
    cv_ptr_right = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  imageLeft = cv_ptr_left->image.clone();
  imageRight = cv_ptr_right->image.clone();

  findPointsInImages();
}

int main(int argc, char **argv){
  cv::namedWindow("Left image", 1);
  cv::namedWindow("Right image", 1);

  ros::init(argc, argv, "hand_to_eye_calibration");
	ros::NodeHandle nh;
	ros::Rate rate(20);

  message_filters::Subscriber<sensor_msgs::Image> image_left(nh, "/camera/image_left/raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_right(nh,"/camera/image_right/raw", 1);

  //  TODO: Find msg type
  //message_filters::Subscriber<sensor_msgs::Image> q_state(nh,"/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_left, image_right, 10);
  sync.registerCallback(boost::bind(&image_sync_callback, _1, _2));

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
