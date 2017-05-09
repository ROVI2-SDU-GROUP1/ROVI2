#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>

#include "yaml-cpp/yaml.h"

#include <opencv2/core/mat.hpp>
//#include <opencv2/calib3d.hpp>

ros::Time leftTime;
ros::Time rightTime;

ros::Publisher pub3D;

geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;
geometry_msgs::PointStamped pose3D;

template<typename _Matrix_Type_>_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
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

#define CALIB_RIGHT_FRONT CALIBRATION_DIR"right_front.yaml"
#define CALIB_LEFT_FRONT CALIBRATION_DIR"left_front.yaml"

Intrinsic loadCalibration(std::string fileName){
  YAML::Node calibration_yaml = YAML::LoadFile(fileName);

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
  //std::cout << "\tLinear method" << std::endl << std::endl;

  //std::cout << "Left:" << std::endl;
  //std::cout << pose2DLeft.point.x << "\n" << pose2DLeft.point.y << std::endl;
  //std::cout << "Right:" << std::endl;
  //std::cout << pose2DRight.point.x << "\n" << pose2DRight.point.y << std::endl;

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

  // std::cout << "A:" << std::endl;
  // std::cout << A << std::endl;
  // std::cout << "invA:" << std::endl;
  // std::cout << invA << std::endl;
  // std::cout << "B:" << std::endl;
  // std::cout << B << std::endl;
  // std::cout << "x:" << std::endl;
  // std::cout << x << std::endl;
  //
  // std::cout << "x_norm:" << std::endl;
  // std::cout << x.norm() << std::endl;

  pose3D.point.x = x(0, 0);
  pose3D.point.y = x(1, 0);
  pose3D.point.z = x(2, 0);

  pose3D.header.stamp = pose2DLeft.header.stamp;

  pub3D.publish(pose3D);
}

void calc3DPose(){
  // Stereo proc        - http://wiki.ros.org/stereo_image_proc
  // Dense stereo ros
  // Q matrix
  //std::cout << std::endl << std::endl << "----Calc 3d pos!" << std::endl;

  linearSolv();
}

void image_sync_callback(const geometry_msgs::PointStamped::ConstPtr &image_left, const geometry_msgs::PointStamped::ConstPtr &image_right){
  pose2DLeft = *image_left;
  pose2DRight = *image_right;
  calc3DPose();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stereo");
  	ros::NodeHandle nh;
  	ros::Rate rate(20);

    calL = loadCalibration(CALIB_RIGHT_FRONT);
    calR = loadCalibration(CALIB_LEFT_FRONT);

    pub3D = nh.advertise<geometry_msgs::PointStamped>("/pose/3d", 1);

    message_filters::Subscriber<geometry_msgs::PointStamped> image_left(nh, "/pose/2d_left", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> image_right(nh,"/pose/2d_right", 1);
    message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> sync(image_left, image_right, 10);
    sync.registerCallback(boost::bind(&image_sync_callback, _1, _2));

    ros::Time last = ros::Time::now();

    while (ros::ok()){
      last = ros::Time::now();
      ros::spinOnce();
  		rate.sleep();
    }

    return 0;
}
