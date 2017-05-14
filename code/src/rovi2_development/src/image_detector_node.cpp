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
#include <HsvAdjust.hpp>
#include <mutex>
#include <thread>
#include <chrono>

int mx = 0;
int my = 0;

static cv::Mat *cameraMatrix;
static cv::Mat *distCoeffs;
static cv::Mat tmp_img;
static std::thread *faker_thread = nullptr;
//static HsvAdjust hsv_adjust;
static ros::Subscriber image_sub;
static ros::Publisher point_pub;
static ros::Publisher image_fake_pub;

void find2d_faker();


sensor_msgs::Image imageToMsg(Mat &img)
{
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;     // Message to be sent
    std_msgs::Header header; // empty header

    header.seq = 1; // user defined counter
    header.stamp = ros::Time::now(); // time

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img); //RGB8 bgr8
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    return img_msg;
}

void closing(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_dilate = 3;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(src, dst, elementDilate);

  int erosion_size_erode = 3;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(dst, src, elementErode);
}

void opening(cv::Mat &src){
  cv::Mat dst;

  int erosion_size_erode = 2;
  cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_erode + 1, 2 * erosion_size_erode + 1), cv::Point(erosion_size_erode, erosion_size_erode));
  cv::erode(src, dst, elementErode);

  int erosion_size_dilate = 2;
  cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size_dilate + 1, 2 * erosion_size_dilate + 1), cv::Point(erosion_size_dilate, erosion_size_dilate));
  cv::dilate(dst, src, elementDilate);
}

void Undistored(cv::Mat &input, cv::Mat &undistorted){
  cv::undistort(input, undistorted, *cameraMatrix, *distCoeffs);
}

std::vector<cv::Point2f> fast_undistort(std::vector<cv::Point2f> &input_points)
{
    std::vector<cv::Point2f> output_points(input_points.size());
    cv::undistortPoints(input_points, output_points, *cameraMatrix, *distCoeffs, cv::noArray(), *cameraMatrix);
    return output_points;
}
// state subscriber
void find2DPose_CV(cv::Mat &img, const sensor_msgs::Image::ConstPtr& msg, __attribute__((unused)) bool overwrite = true)
{
    //auto time_start = std::chrono::high_resolution_clock::now();
    cv::Mat &imageUndistorted = img;
    //Undistored(img,imageUndistorted);

    cv::Mat imageHSV;
    cv::cvtColor(imageUndistorted, imageHSV, CV_BGR2HSV);
    /*mtx.lock();
    std::cout << "test" << std::endl;
    if(overwrite) tmp_img = img;
    hsv_adjust.hsvSegmentation(imageUndistorted);
    mtx.unlock();
    cv::waitKey(1);

    return;*/
    cv::Mat thresholdLow;
    cv::Mat thresholdHigh;
    cv::inRange(imageHSV, cv::Scalar(0, 130, 20), cv::Scalar(10, 255, 255), thresholdLow);
    cv::inRange(imageHSV, cv::Scalar(130, 130, 20), cv::Scalar(255, 255, 255), thresholdHigh);

    cv::Mat threshold = thresholdLow + thresholdHigh;

    opening(threshold);
    closing(threshold);
    //cv::imshow("thres", threshold);

    std::vector<std::vector<cv::Point> > cont;
    std::vector<cv::Vec4i> hier;
    cv::findContours(threshold, cont, hier, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    geometry_msgs::PointStamped point;
    point.point.x = 0;
    point.point.y = 0;
    point.header.stamp = msg->header.stamp;
    if (!cont.size()){
      //cv::imshow("t2", imageBGR);
      //cv::waitKey(1);
      //point_pub.publish(point);
      return;
    }
    std::vector<cv::Point> largest_contour = cont[0];
    for(auto tmp : cont)
    {
        if(cv::contourArea(tmp) > cv::contourArea(largest_contour))
            largest_contour = tmp;
    }

    //Undistort the contour
    std::vector<cv::Point2f> contour;
    for(uint64_t i = 0; i < largest_contour.size(); i++)
    {
        contour.push_back(largest_contour[i]);
    }
    std::vector<cv::Point2f> undistorted_contour = fast_undistort(contour);
      cv::Moments moments = cv::moments(undistorted_contour, false);
      cv::Point2f imgCoord = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

    point.point.x = imgCoord.x;
    point.point.y = imgCoord.y;
    point.header.stamp = msg->header.stamp;

    cv::circle(imageUndistorted, imgCoord, 10, cv::Scalar(0, 255, 0), 2);

    //  cv::imshow("Distorted", imageBGR);
    //cv::imshow("t1", imageUndistorted);
    cv::waitKey(1);

    point_pub.publish(point);
    //std::cout <<  std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - time_start).count()  << std::endl;

}

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
  find2DPose_CV(imageBGR, msg);
  //if(faker_thread == nullptr) faker_thread = new std::thread(find2d_faker);
}


void find2DPose_fake(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat imageBGR = cv_ptr->image.clone();
  find2DPose_CV(imageBGR, msg, false);
  if(faker_thread == nullptr) faker_thread = new std::thread(find2d_faker);
}

bool is_file_exist(std::string fileName){
    std::ifstream infile(fileName.c_str());
    return infile.good();
}

void find2d_faker()
{
    while(1)
    {
        auto ros_img = imageToMsg(tmp_img);
        image_fake_pub.publish(ros_img);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}


void exit_tester()
{
    while(ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::exit(0);
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

  image_sub = nh.subscribe<sensor_msgs::Image>(image_sub_name, 5, find2DPose);
  point_pub = nh.advertise<geometry_msgs::PointStamped>(point_pub_name, 1);
  image_fake_pub = nh.advertise<sensor_msgs::Image>("/camera/left/image_raw_faker", 1);
  ros::Subscriber image_sub2 = nh.subscribe<sensor_msgs::Image>("/camera/left/image_raw_faker", 1, find2DPose_fake);

  ros::Time last = ros::Time::now();
  std::thread t1(exit_tester);
  while (ros::ok()){
    last = ros::Time::now();
    ros::spinOnce();
		rate.sleep();
  }

  return 0;
}
