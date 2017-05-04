#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <cassert>
#include "yaml-cpp/yaml.h"
#include <fstream>

int mx = 0;
int my = 0;

cv::Mat *cameraMatrix;
cv::Mat *distCoeffs;


ros::Subscriber image_sub;
ros::Publisher point_pub;

void closing(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_dilate = 10;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(src, dst, elementDilate);

  int erosion_size_erode = 7;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(dst, src, elementErode);
}

void opening(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_erode = 7;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(src, dst, elementErode);

  int erosion_size_dilate = 10;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(dst, src, elementDilate);
}

int abs(int x){
  if (x < 0) return -x;
  return x;
}


cv::Mat Undistored(cv::Mat input){
  cv::Mat undistorted;
  cv::undistort(input, undistorted, *cameraMatrix, *distCoeffs);

  return undistorted;
}

// state subscriber
void find2DPose(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  cv::Mat imageBGR = cv_ptr->image.clone();
  cv::Mat imageUndistorted = Undistored(imageBGR);


  cv::Mat imageHSV;
  cv::cvtColor(imageUndistorted, imageHSV, CV_BGR2HSV);


/*
  cv::Mat threshold;
  adaptive(imageHSV, threshold, cv::Scalar(0, 255, 255), 10);

  std::cout << imageHSV.at<cv::Vec3b>(my, mx) << std::endl;

*/
  cv::Mat thresholdLow;
  cv::Mat thresholdHigh;
  cv::inRange(imageHSV, cv::Scalar(0, 150, 50), cv::Scalar(10, 255, 255), thresholdLow);
  cv::inRange(imageHSV, cv::Scalar(215, 150, 50), cv::Scalar(255, 255, 255), thresholdHigh);

  cv::Mat threshold = thresholdLow + thresholdHigh;

  opening(threshold);
  closing(threshold);


	std::vector<std::vector<cv::Point> > cont;
	std::vector<cv::Vec4i> hier;
	cv::findContours(threshold, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  geometry_msgs::PointStamped point;

  if (!cont.size()){
    //cv::imshow("t2", imageBGR);
    //cv::waitKey(1);
    point_pub.publish(point);
    return;
  }

	cv::Moments moments = cv::moments(cont[0], false);
	cv::Point2f imgCoord = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

  point.point.x = imgCoord.x;
  point.point.y = imgCoord.y;
  point.header.stamp = msg->header.stamp;

  cv::circle(imageUndistorted, imgCoord, 10, cv::Scalar(0, 255, 0), 2);


//  cv::imshow("Distorted", imageBGR);
//  cv::imshow("Undistorted", imageUndistorted);
//  cv::imshow("t1", threshold);

  cv::waitKey(1);


  point_pub.publish(point);
}

bool is_file_exist(std::string fileName){
    std::ifstream infile(fileName.c_str());
    return infile.good();
}

int main(int argc, char **argv){
  std::string image_sub_name;
  std::string point_pub_name;
  std::string yaml_path;


  ros::init(argc, argv, "image_detection");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);

  nh.param<std::string>("image_sub", image_sub_name, "/camera/left/image_raw");
  nh.param<std::string>("point_pub", point_pub_name, "/pose/2d_left");
  nh.param<std::string>("calibration_yaml_path", yaml_path, "default.yaml");

  std::string abs_yaml_path = std::string(CALIBRATION_DIR) + yaml_path;
  std::cout << "Loading calib: " << abs_yaml_path << std::endl;
  if(!is_file_exist(abs_yaml_path)){
    ROS_WARN("File: %s does not exist\n", abs_yaml_path.c_str());
    exit(1);
  }

  // Read yaml parameters
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

  //cv::Mat distCoeffs;
  for(unsigned int i = 0; i < distortion_coefficients["data"].size();i++){
    distCoeffs->at<double>(i) = distortion_coefficients["data"][i].as<double>();
  }



  image_sub = nh.subscribe<sensor_msgs::Image>(image_sub_name, 1, find2DPose);
  point_pub = nh.advertise<geometry_msgs::PointStamped>(point_pub_name, 1);

  ros::Time last = ros::Time::now();

  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
