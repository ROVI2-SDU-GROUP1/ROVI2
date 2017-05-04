#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <caros_control_msgs/RobotState.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>

#include "yaml-cpp/yaml.h"

#include <fstream>

geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;
caros_control_msgs::RobotState qState;

cv::Mat *cameraMatrixLeft;
cv::Mat *cameraMatrixRight;
cv::Mat *distCoeffsLeft;
cv::Mat *distCoeffsRight;

cv::Mat imageLeft;
cv::Mat imageRight;

ros::Publisher pub_point_left;
ros::Publisher pub_point_right;
ros::Publisher pub_q;

bool leftPressed = false;
bool rightPressed = false;

void CallBackFuncLeft(int event, int x, int y, int flags, void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DLeft.point.x = x;
    pose2DLeft.point.y = y;
    std::cout << "Left mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    leftPressed = true;
  }
}

void CallBackFuncRight(int event, int x, int y, int flags, void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DRight.point.x = x;
    pose2DRight.point.y = y;
    std::cout << "Right mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    rightPressed = true;
  }
}

cv::Mat Undistored(cv::Mat input, cv::Mat *cameraMatrix, cv::Mat *distCoeffs){
  cv::Mat undistorted;
  cv::undistort(input, undistorted, *cameraMatrix, *distCoeffs);
  return undistorted;
}

void image_sync_callback(const sensor_msgs::Image::ConstPtr &image_left, const sensor_msgs::Image::ConstPtr &image_right, const caros_control_msgs::RobotState::ConstPtr &q){
  pose2DLeft.header.stamp = image_left->header.stamp;
  pose2DRight.header.stamp = image_left->header.stamp;
  qState = *q;
  qState.header.stamp = image_left->header.stamp;

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

  imageLeft = Undistored(cv_ptr_left->image.clone(), cameraMatrixLeft, distCoeffsLeft);
  imageRight = Undistored(cv_ptr_right->image.clone(), cameraMatrixRight, distCoeffsRight);

  cv::imshow("Left image", imageLeft);
  cv::imshow("Right image", imageRight);
  cv::waitKey(1);

  while(!(leftPressed && rightPressed)){

  }

  pub_point_left.publish(pose2DLeft);
  pub_point_right.publish(pose2DRight);
  pub_q.publish(qState);

  leftPressed = false;
  rightPressed = false;
}

bool is_file_exist(std::string fileName){
    std::ifstream infile(fileName.c_str());
    return infile.good();
}

void loadYAMLparameters( std::string yaml_path, cv::Mat *cameraMatrix, cv::Mat *distCoeffs ){
  std::string abs_yaml_path = std::string(CALIBRATION_DIR) + yaml_path;
  std::cout << "Loading calib: " << abs_yaml_path << std::endl;
  if(!is_file_exist(abs_yaml_path)){
    ROS_WARN("File: %s does not exist\n", abs_yaml_path.c_str());
    exit(1);
  }

  YAML::Node calibration_yaml = YAML::LoadFile(abs_yaml_path);
  YAML::Node camera_matrix = calibration_yaml["camera_matrix"];
  YAML::Node distortion_coefficients = calibration_yaml["distortion_coefficients"];

  cameraMatrix = new cv::Mat(3,3, CV_64FC1);
  distCoeffs = new cv::Mat(1,5, CV_64FC1);

  for(unsigned int i = 0; i < 3; i++){
    for(unsigned int j = 0; j < 3; j++){
      cameraMatrix->at<double>(i,j) = camera_matrix["data"][i*3+j].as<double>();
    }
  }

  for(unsigned int i = 0; i < distortion_coefficients["data"].size();i++){
    distCoeffs->at<double>(i) = distortion_coefficients["data"][i].as<double>();
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "hand_to_eye_calibration");
	ros::NodeHandle nh;
	ros::Rate rate(20);

  cv::namedWindow("Left image", 1);
  cv::namedWindow("Right image", 1);

  cv::setMouseCallback("Left image", CallBackFuncLeft, NULL);
  cv::setMouseCallback("Right image", CallBackFuncRight, NULL);

  std::string param_yaml_path_left;
  std::string param_yaml_path_right;
  std::string param_image_left;
  std::string param_image_right;
  std::string param_point_left;
  std::string param_point_right;
  std::string param_robot_state_pub;
  std::string param_robot_state_sub;

  nh.param<std::string>("calibration_yaml_path_left", param_yaml_path_left, "default.yaml");
  nh.param<std::string>("calibration_yaml_path_right", param_yaml_path_right, "default.yaml");
  nh.param<std::string>("image_left", param_image_left, "/camera/left/image_raw");
  nh.param<std::string>("image_right", param_image_right, "/camera/right/image_raw");
  nh.param<std::string>("sub_robot_state", param_robot_state_sub, "/ur_simple_demo_node/caros_serial_device_service_interface/robot_state");
  nh.param<std::string>("pub_point_left", param_point_left, "/pose/2d_left");
  nh.param<std::string>("pub_point_right", param_point_right, "/pose/2d_right");
  nh.param<std::string>("pub_robot_state", param_robot_state_pub, "/q_state_processed");

  loadYAMLparameters(param_yaml_path_left, cameraMatrixLeft, distCoeffsLeft);
  loadYAMLparameters(param_yaml_path_right, cameraMatrixRight, distCoeffsRight);

  pub_point_left = nh.advertise<geometry_msgs::PointStamped>(param_point_left, 1);
  pub_point_right = nh.advertise<geometry_msgs::PointStamped>(param_point_right, 1);
  pub_q = nh.advertise<caros_control_msgs::RobotState>(param_robot_state_pub, 1);

  message_filters::Subscriber<sensor_msgs::Image> sub_image_left(nh, param_image_left, 0);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_right(nh, param_image_right, 0);
  message_filters::Subscriber<caros_control_msgs::RobotState> sub_q_state(nh, param_robot_state_sub, 0);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, caros_control_msgs::RobotState> sync(sub_image_left, sub_image_right, sub_q_state, 10);
  sync.registerCallback(boost::bind(&image_sync_callback, _1, _2, _3));

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
