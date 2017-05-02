/*
  New terminal > bash > roscore
  New terminal > bash > rosbag play -l ...
  New terminal > bash > rosrun rovi2_development image_detector_left
  New terminal > bash > rosrun rovi2_development image_detector_right
  New terminal > bash > cd workspace/ROVI2/code > catkin_make && rosrun rovi2_development stereo
*/

#include <mutex>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>

ros::Time leftTime;
ros::Time rightTime;

ros::Subscriber sub2DLeft;
ros::Subscriber sub2DRight;

ros::Publisher pub3D;

std::mutex mutex;

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
  Eigen::MatrixXd rotation;
  Eigen::MatrixXd translation;

  Eigen::MatrixXd transformation;
  Eigen::MatrixXd P;
  Eigen::MatrixXd PX;
  Eigen::MatrixXd px;
  Eigen::MatrixXd center;
};

Intrinsic calL;
Intrinsic calR;

Intrinsic loadCalibration(std::string fileName, int index){
  Intrinsic cal;
  std::ifstream file(fileName);
  //assert(file.is_open());
  std::string line;
  while (getline(file, line)){
    std::cout << line << std::endl;
  }

  cal.intrinsic.resize(3, 3);
  cal.intrinsic.row(0) << 1280, 0, 512;
  cal.intrinsic.row(1) << 0, 1280, 384;
  cal.intrinsic.row(2) << 0, 0, 1;

  if(false){  //  Calibration file from the ex.2 in vision (rectified)
    cal.rotation.resize(3, 3);
    cal.rotation.row(0) << -4.37113882867e-08, 1.0, 0.0;
    cal.rotation.row(1) << 0.173648133874, 7.59040119647e-09, -0.984807729721;
    cal.rotation.row(2) << -0.984807729721, -4.30473150459e-08, -0.173648133874;

    cal.translation.resize(3, 1);
    if (!index) cal.translation.col(0) << -0.499999552965, 0.725537955761, 10.2821979523;
    else        cal.translation.col(0) << 0.500000417233, 0.725538015366, 10.2821979523;
  }
  else if(true){  //  Our calibration
    cal.rotation.resize(3, 3);
    if (!index) cal.rotation.row(0) << 0.087155804038, 0.996194720268, 0.0;
    else        cal.rotation.row(0) << -0.0871557667851, 0.996194720268, 0.0;
    if (!index) cal.rotation.row(1) << 0.172987341881, -0.0151344416663, -0.984807729721;
    else        cal.rotation.row(1) << 0.172987341881, 0.0151344360784, -0.984807729721;
    if (!index) cal.rotation.row(2) << -0.981060266495, 0.0858317092061, -0.173648133874;
    else        cal.rotation.row(2) << -0.981060266495, -0.0858316794038, -0.173648133874;

    cal.translation.resize(3, 1);
    if (!index) cal.translation.col(0) << -0.373460680246, 0.724578678608, 10.2876386642;
    else        cal.translation.col(0) << 0.373460680246, 0.724578678608, 10.2876386642;
  }else{
    cal.rotation.resize(3, 3);
    cal.rotation.row(0) << 0, 1, 0;
    cal.rotation.row(1) << 0, 0, 1;
    cal.rotation.row(2) << 1, 0, 0;

    cal.translation.resize(3, 1);
    if (!index) cal.translation.col(0) << -5, 0, 0;
    else        cal.translation.col(0) << 5, 0, 0;
  }

  Eigen::MatrixXd temp1;
  Eigen::MatrixXd temp2;

  // Transformation
  temp1.resize(3, 4);
  temp2.resize(1, 4);
  temp1 << cal.rotation, cal.translation;
  temp2.row(0) << 0, 0, 0, 1;
  cal.transformation.resize(4, 4);
  cal.transformation << temp1, temp2;

  // P
  temp1.resize(3, 4);
  temp2.resize(3, 1);
  temp2.col(0) << 0, 0, 0;
  temp1 << cal.intrinsic, temp2;
  cal.P = temp1 * cal.transformation;

  // PX
  cal.PX = cal.P.block(0, 0, 3, 3);

  // px
  cal.px = cal.P.block(0, 3, 3, 1);

  // center
  temp1.resize(3, 1);
  temp1 = -cal.rotation.inverse() * cal.translation;
  cal.center.resize(4, 1);
  cal.center << temp1, 1;

  return cal;
}

void linearSolv(){
  std::cout << "\tLinear method" << std::endl << std::endl;

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

  std::cout << "A:" << std::endl;
  std::cout << A << std::endl;
  std::cout << "invA:" << std::endl;
  std::cout << invA << std::endl;
  std::cout << "B:" << std::endl;
  std::cout << B << std::endl;
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

void epiSolv(){
  std::cout << "\tEpi method" << std::endl << std::endl;

  //  Compute infinity points
  Eigen::MatrixXd ml(3,1);
  //ml.col(0) << pose2DLeft.point.x, pose2DLeft.point.y, 1;
  ml.col(0) << 700, 300, 1;
  Eigen::MatrixXd Minfl = calL.PX.inverse() * ml;

  Eigen::MatrixXd mr(3,1);
  //mr.col(0) << pose2DRight.point.x, pose2DRight.point.y, 1;
  mr.col(0) << 300, 300, 1;
  Eigen::MatrixXd Minfr = calR.PX.inverse() * mr;

  //  Compute plucker lines
  //    mu1 and nu1
  Eigen::Vector3d tmp1 = calL.center.block(0, 0, 3, 1);
  Eigen::Vector3d tmp2 = Minfl.block(0, 0, 3, 1);
  Eigen::Vector3d mu1 = tmp1.cross(tmp2) / tmp2.norm();
  Eigen::Vector3d nu1 = tmp2 / tmp2.norm();

  //    mu1 and nu1
  tmp1 = calR.center.block(0, 0, 3, 1);
  tmp2 = Minfr.block(0, 0, 3, 1);
  Eigen::Vector3d mu2 = tmp1.cross(tmp2) / tmp2.norm();
  Eigen::Vector3d nu2 = tmp2 / tmp2.norm();

  //  M1 and M2
  Eigen::Vector3d M1 = ( nu1 * nu2.cross(mu2).transpose() - (nu1 * nu2.transpose()) * nu1 * (nu2.cross(mu1)).transpose() ) / ( (nu1.cross(nu2)).norm() * (nu1.cross(nu2)).norm() ) * nu1 + nu1.cross(mu1);
  Eigen::Vector3d M2 = ( nu2 * nu1.cross(mu1).transpose() - (nu2 * nu1.transpose()) * nu2 * (nu1.cross(mu2)).transpose() ) / ( (nu2.cross(nu1)).norm() * (nu2.cross(nu1)).norm() ) * nu2 + nu2.cross(mu2);
  Eigen::Vector3d M = M1 + (M2 - M1)/2;

  std::cout << "M1:" << std::endl;
  std::cout << M1 << std::endl;
  std::cout << "M2:" << std::endl;
  std::cout << M2 << std::endl;
  std::cout << "M:" << std::endl;
  std::cout << M << std::endl;
  std::cout << "M_norm:" << std::endl;
  std::cout << M.norm() << std::endl;

  pose3D.point.x = M(0, 0);
  pose3D.point.y = M(1, 0);
  pose3D.point.z = M(2, 0);

  pose3D.header.stamp = pose2DLeft.header.stamp;

  pub3D.publish(pose3D);
}
/*
#include <opencv2/core/eigen.hpp>
cv::Mat_<float> a = Mat_<float>::ones(2,2);
Eigen::Matrix<float,Dynamic,Dynamic> b;
cv2eigen(a,b);
*/

void eig2mat( Eigen::MatrixXd &src, cv::Mat &dst ){

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
  //epiSolv();
  //openCvTriangulation();
}

void receiveLeftImage(const geometry_msgs::PointStamped::ConstPtr &msg){
  mutex.lock();
  pose2DLeft = *msg;
  leftTime = msg->header.stamp;
  if (leftTime == rightTime) calc3DPose();
  mutex.unlock();
}

void receiveRightImage(const geometry_msgs::PointStamped::ConstPtr &msg){
  mutex.lock();
  pose2DRight = *msg;
  rightTime = msg->header.stamp;
  if (leftTime == rightTime) calc3DPose();
  mutex.unlock();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "stereo");
  	ros::NodeHandle nh;
  	ros::Rate rate(20);

    calL = loadCalibration("calibration.txt", 1);
    calR = loadCalibration("calibration.txt", 0);

    sub2DLeft = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_left", 1, receiveLeftImage);
    sub2DRight = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_right", 1, receiveRightImage);
    pub3D = nh.advertise<geometry_msgs::PointStamped>("/pose/3d", 1);

    ros::Time last = ros::Time::now();

    while (ros::ok()){
      last = ros::Time::now();
      ros::spinOnce();
  		rate.sleep();
      //calc3DPose();
    }

    return 0;
}
