/*
  New terminal > bash > roscore
  New terminal > bash > rosbag play -l ...
  New terminal > bash > rosrun rovi2_development image_detector_left
  New terminal > bash > rosrun rovi2_development image_detector_right
  New terminal > bash > cd workspace/ROVI2/code > catkin_make && rosrun rovi2_development stereo
*/

//#include <mutex>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
//#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>
#include "yaml-cpp/yaml.h"


#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
//#include <opencv2/calib3d.hpp>

ros::Time leftTime;
ros::Time rightTime;

//using namespace message_filters;

ros::Subscriber sub2DLeft;
ros::Subscriber sub2DRight;

ros::Publisher pub3D;

//std::mutex mutex;

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

  //std::cout << calibration_front_left["projection_matrix"].as<std::string>() << std::endl;
  // for(YAML::const_iterator it=tmp["data"].begin();it!=tmp["data"].end();++it) {
  //   // std::cout << "key " << it->first.as<std::string>() << " value " << it->second.as<std::string>() << "\n";
  //   std::cout << " value " << it->as<double>() <<"\n";
  // }

  Intrinsic cal;

  cal.intrinsic.resize(3, 3);
  // std::cout << tmp["data"][0] << std::endl;
  // double num = tmp["data"][0].as<double>();
  // std::cout << "num: " << num << std::endl;
  //cal.intrinsic.row(0) <<num, num, num;
  YAML::Node tmp = calibration_yaml["camera_matrix"];
  for(unsigned int i = 0; i < tmp["data"].size();i+=3){
    cal.intrinsic.row(i/3) << tmp["data"][i].as<double>(), tmp["data"][i+1].as<double>(),tmp["data"][i+2].as<double>();
  }


  // cal.intrinsic.row(0) << 1348.764942, 0.000000, 533.749161;
  // cal.intrinsic.row(1) << 0.000000, 1349.118163, 354.434678;
  // cal.intrinsic.row(2) << 0.000000, 0.000000, 1.000000;


  // else        cal.intrinsic.row(0) << 1355.395640, 0.000000, 540.084150;
  // else        cal.intrinsic.row(1) << 0.000000, 1354.485656, 437.144524;
  // else        cal.intrinsic.row(2) << 0.000000, 0.000000, 1.000000;

  // // P
  cal.P.resize(3,4);
  tmp = calibration_yaml["projection_matrix"];
  for(unsigned int i = 0; i < tmp["data"].size();i+=4){
    cal.P.row(i/4) << tmp["data"][i].as<double>(), tmp["data"][i+1].as<double>(),tmp["data"][i+2].as<double>(),tmp["data"][i+3].as<double>();
  }

  // if (!index) cal.P.row(0) << 2030.399737, 0.000000, 382.195145, 0.000000;
  // if (!index) cal.P.row(1) << 0.000000, 2030.399737, 408.654493, 0.000000;
  // if (!index) cal.P.row(2) << 0.000000, 0.000000, 1.000000, 0.000000;


  // else        cal.P.row(0) << 2030.399737, 0.000000, 382.195145, -246.187430;
  // else        cal.P.row(1) << 0.000000, 2030.399737, 408.654493, 0.000000;
  // else        cal.P.row(2) << 0.000000, 0.000000, 1.000000, 0.000000;
  std::cout << cal.P << std::endl;

  // PX
  cal.PX = cal.P.block(0, 0, 3, 3);

  // px
  cal.px = cal.P.block(0, 3, 3, 1);

  return cal;
}

void linearSolv(){
  std::cout << "\tLinear method" << std::endl << std::endl;

  std::cout << "Left:" << std::endl;
  std::cout << pose2DLeft.point.x << "\n" << pose2DLeft.point.y << std::endl;
  std::cout << "Right:" << std::endl;
  std::cout << pose2DRight.point.x << "\n" << pose2DRight.point.y << std::endl;

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
  std::cout << "x:" << std::endl;
  std::cout << x << std::endl;

  std::cout << "x_norm:" << std::endl;
  std::cout << x.norm() << std::endl;

  pose3D.point.x = x(0, 0);
  pose3D.point.y = x(1, 0);
  pose3D.point.z = x(2, 0);

  pose3D.header.stamp = pose2DLeft.header.stamp;

  pub3D.publish(pose3D);
}

void openCvTriangulation(){
  //  OpenCV triangulation - http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  cv::Mat Pl = cv::Mat::ones(3,4,CV_32F);
  cv::Mat Pr = cv::Mat::ones(3,4,CV_32F);

  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 4; j++){
      Pl.at<float>(i,j) = calL.P(i,j);
      Pr.at<float>(i,j) = calR.P(i,j);
    }
  }

  cv::Mat pointL = cv::Mat::ones(2,1,CV_32F);
  cv::Mat pointR = cv::Mat::ones(2,1,CV_32F);
  pointL.at<float>(1,1) = pose2DLeft.point.x;
  pointL.at<float>(2,1) = pose2DLeft.point.y;

  pointR.at<float>(1,1) = pose2DRight.point.x;
  pointR.at<float>(2,1) = pose2DRight.point.y;

  //cv::sfm::triangulatePoints(Pl, Pr, pointL, pointR, OutputArray points4D)

  /*
  std::cout << Pl << std::endl;
  std::cout << Pr << std::endl;
  std::cout << pointL << std::endl;
  std::cout << pointR << std::endl;
  */

  cv::Mat point3D;
  std::cout << point3D << std::endl;
  //triangulatePoints(Pl, Pr, pointL, pointR, point3D);
  std::cout << point3D << std::endl;
}

void calc3DPose(){
  // Stereo proc        - http://wiki.ros.org/stereo_image_proc
  // Dense stereo ros
  // Q matrix
  std::cout << std::endl << std::endl << "----Calc 3d pos!" << std::endl;

  linearSolv();
  //openCvTriangulation();
}

// void receiveLeftImage(const geometry_msgs::PointStamped::ConstPtr &msg){
//   mutex.lock();
//   pose2DLeft = *msg;
//   leftTime = msg->header.stamp;
//   if (leftTime == rightTime)
//     calc3DPose();
//   else {
//     std::cout << "Left - times not equal left: " << leftTime << " right: " << rightTime << std::endl;
//   }
//   mutex.unlock();
// }
//
// void receiveRightImage(const geometry_msgs::PointStamped::ConstPtr &msg){
//   mutex.lock();
//   pose2DRight = *msg;
//   rightTime = msg->header.stamp;
//   if (leftTime == rightTime)
//       calc3DPose();
//     else {
//       std::cout << "Right - times not equal left: " << leftTime << " right: " << rightTime << std::endl;
//     }
//   mutex.unlock();
// }

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

    // sub2DLeft = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_left", 1, receiveLeftImage);
    // sub2DRight = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_right", 1, receiveRightImage);
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
      //calc3DPose();
    }

    return 0;
}
