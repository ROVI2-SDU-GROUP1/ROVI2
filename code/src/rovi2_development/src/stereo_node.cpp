#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>
#include <fstream>
#include "rw/math/RPY.hpp"

#include "yaml-cpp/yaml.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include "YAMLCalibration/YAMLCalibration.hpp"



static ros::Time leftTime;
static ros::Time rightTime;

static ros::Publisher pub3D;

static geometry_msgs::PointStamped pose2DLeft;
static geometry_msgs::PointStamped pose2DRight;
static geometry_msgs::PointStamped pose3D;
static rw::math::Transform3D<double> Trans_camera_in_base;

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
rw::math::Transform3D<double> get_cam_to_base(std::string filename)
{
    std::cout << "loading cam to base from " << filename << std::endl;
    YAMLCalibration hte_calibration( std::string(CALIBRATION_DIR) + filename);
    cv::Mat translation_vector = hte_calibration.get_translation_vector();
    cv::Mat rotation_matrix = hte_calibration.get_rotation_matrix();
    rw::math::Vector3D<double> trans(   translation_vector.at<double>(0,0),
                                        translation_vector.at<double>(1,0),
                                        translation_vector.at<double>(2,0));

    rw::math::Rotation3D<double> rot( rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
                                      rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
                                      rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));

    //std::cout << rot << std::endl << rotation_matrix << std::endl;
    std::cout << "Rot determinant: " << rot.e().determinant() << std::endl;
    rw::math::Transform3D<double> cam_to_base(trans, rot);

    return cam_to_base;

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

  rw::math::Vector3D<double> rw_x(x / 1000.);

  auto rpy = rw::math::RPY<>(Trans_camera_in_base.R());
  rpy[2] = -rpy[2];

  rw_x = rw::math::Transform3D<double>(Trans_camera_in_base.P(), rpy.toRotation3D()) * rw_x;
  std::cout << rw_x << std::endl;
  std::cout << x << std::endl;
  pose3D.point.x = rw_x(0);
  pose3D.point.y = rw_x(1);
  pose3D.point.z = rw_x(2);

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
    std::string param_hte_path;
    std::string param_point_sub_left;
    std::string param_point_sub_right;
    std::string param_pose_3d;


    nh.param<std::string>("calibration_yaml_path_left", param_yaml_path_left, "default.yaml");
    nh.param<std::string>("calibration_yaml_path_right", param_yaml_path_right, "default.yaml");
    nh.param<std::string>("point_sub_left", param_point_sub_left, "/pose/");
    nh.param<std::string>("point_sub_right", param_point_sub_right, "/pose/");
    nh.param<std::string>("param_hte_path", param_hte_path, "default.yaml");
    nh.param<std::string>("point_pub", param_pose_3d, "/pose/");



    calL = loadCalibration(param_yaml_path_left);
    calR = loadCalibration(param_yaml_path_right);
    Trans_camera_in_base = get_cam_to_base(param_hte_path);
    std::cout << Trans_camera_in_base << std::endl;
    pub3D = nh.advertise<geometry_msgs::PointStamped>(param_pose_3d, 1);

    message_filters::Subscriber<geometry_msgs::PointStamped> image_left(nh, param_point_sub_left, 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> image_right(nh,param_point_sub_right, 1);
    message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> sync(image_left, image_right, 10);
    sync.registerCallback(boost::bind(&image_sync_callback, _1, _2));

    ros::spin();
    return 0;
}
