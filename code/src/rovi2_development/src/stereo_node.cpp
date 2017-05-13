#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>
#include <fstream>

#include "yaml-cpp/yaml.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

ros::Time leftTime;
ros::Time rightTime;

ros::Publisher pub3D;

geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;
geometry_msgs::PointStamped pose3D;

template<typename _Matrix_Type_>_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()){
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

struct Intrinsic{
  int width;
  int height;
  Eigen::MatrixXd intrinsic;
  Eigen::MatrixXd distortion;

  Eigen::MatrixXd P;
  Eigen::MatrixXd PX;
  Eigen::MatrixXd px;
  Eigen::MatrixXd center;
};

Intrinsic calL;
Intrinsic calR;

bool is_file_exist(std::string fileName){
    std::ifstream infile(fileName.c_str());
    return infile.good();
}


Intrinsic loadCalibration(std::string fileName){
  std::string yaml_filename = std::string(CALIBRATION_DIR) + fileName;
  if(!is_file_exist(yaml_filename)){
    ROS_WARN("File: %s does not exist\n", yaml_filename.c_str());
    exit(1);
  } else {
    ROS_INFO("Calib file opened: %s\n", yaml_filename.c_str());

  }

  YAML::Node calibration_yaml = YAML::LoadFile(yaml_filename);

  Intrinsic cal;
  cal.intrinsic.resize(3, 3);
  YAML::Node tmp = calibration_yaml["camera_matrix"];
  for(unsigned int i = 0; i < tmp["data"].size();i+=3){
    cal.intrinsic.row(i/3) << tmp["data"][i].as<double>(), tmp["data"][i+1].as<double>(),tmp["data"][i+2].as<double>();
  }

  // P
  cal.P.resize(3,4);
  tmp = calibration_yaml["projection_matrix"];
  for(unsigned int i = 0; i < tmp["data"].size();i+=4){
    cal.P.row(i/4) << tmp["data"][i].as<double>(), tmp["data"][i+1].as<double>(),tmp["data"][i+2].as<double>(),tmp["data"][i+3].as<double>();
  }

  // PX
  cal.PX = cal.P.block(0, 0, 3, 3);

  // px
  cal.px = cal.P.block(0, 3, 3, 1);

  return cal;
}

void openCVMethod(){

  cv::Mat center_right(2, 1, CV_64FC1);
  cv::Mat center_left(2, 1, CV_64FC1);

  center_left.at<double>(0,0) = pose2DLeft.point.x;
  center_left.at<double>(0,1) = pose2DLeft.point.y;

  center_right.at<double>(0,0) = pose2DRight.point.x;
  center_right.at<double>(0,1) = pose2DRight.point.y;

  cv::Mat proj_left = ( cv::Mat_<float>(3, 4) << calL.P(0,0), calL.P(0,1), calL.P(0,2), calL.P(0,3), calL.P(1,0), calL.P(1,1), calL.P(1,2),calL.P(1,3), calL.P(2,0), calL.P(2,1), calL.P(2,2),calL.P(2,3) );
  cv::Mat proj_right = ( cv::Mat_<float>(3, 4) << calR.P(0,0), calR.P(0,1), calR.P(0,2), calR.P(0,3), calR.P(1,0), calR.P(1,1), calR.P(1,2),calR.P(1,3), calR.P(2,0), calR.P(2,1), calR.P(2,2),calR.P(2,3) );

  cv::Mat point4D;

  cv::triangulatePoints(proj_left, proj_right, center_left, center_right, point4D);

  pose3D.point.x = point4D.at<double>(0,0);
  pose3D.point.y = point4D.at<double>(1,0);
  pose3D.point.z = point4D.at<double>(2,0);

  pose3D.header.stamp = pose2DLeft.header.stamp;
  ROS_INFO("Point out");
  pub3D.publish(pose3D);
}

void linearSolv(){

  Eigen::MatrixXd A(4, 3);
  A.row(0) = -pose2DLeft.point.x * calL.PX.row(2) + calL.PX.row(0);
  A.row(1) = -pose2DLeft.point.y * calL.PX.row(2) + calL.PX.row(1);
  A.row(2) = -pose2DRight.point.x * calR.PX.row(2) + calR.PX.row(0);
  A.row(3) = -pose2DRight.point.y * calR.PX.row(2) + calR.PX.row(1);

  Eigen::MatrixXd B(4, 1);
  B.row(0) = pose2DLeft.point.x * calL.px.row(2) - calL.px.row(0);
  B.row(1) = pose2DLeft.point.y * calL.px.row(2) - calL.px.row(1);
  B.row(2) = pose2DRight.point.x * calR.px.row(2) - calR.px.row(0);
  B.row(3) = pose2DRight.point.y * calR.px.row(2) - calR.px.row(1);

  Eigen::MatrixXd invA(3, 4);
  invA = pseudoInverse(A);

  Eigen::MatrixXd x(3, 1);
  x = (invA * A).inverse() * (invA * B);

  pose3D.point.x = x(0, 0);
  pose3D.point.y = x(1, 0);
  pose3D.point.z = x(2, 0);

  pose3D.header.stamp = pose2DLeft.header.stamp;

  pub3D.publish(pose3D);
}

void calc3DPose(){
  //openCVMethod();
  linearSolv();
}

void image_sync_callback(const geometry_msgs::PointStamped::ConstPtr &image_left, const geometry_msgs::PointStamped::ConstPtr &image_right){
  pose2DLeft = *image_left;
  pose2DRight = *image_right;
  calc3DPose();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stereo_node");
  	ros::NodeHandle nh("~");

    std::string param_yaml_path_left;
    std::string param_yaml_path_right;

    nh.param<std::string>("calibration_yaml_path_left", param_yaml_path_left, "default.yaml");
    nh.param<std::string>("calibration_yaml_path_right", param_yaml_path_right, "default.yaml");

    calL = loadCalibration(param_yaml_path_left);
    calR = loadCalibration(param_yaml_path_right);

    pub3D = nh.advertise<geometry_msgs::PointStamped>("/pose/3d", 1);

    message_filters::Subscriber<geometry_msgs::PointStamped> image_left(nh, "/pose/2d_left", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> image_right(nh,"/pose/2d_right", 1);
    message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> sync(image_left, image_right, 10);
    sync.registerCallback(boost::bind(&image_sync_callback, _1, _2));

    ros::spin();
    return 0;
}
