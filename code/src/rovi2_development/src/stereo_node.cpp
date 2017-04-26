#include <mutex>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cassert>

ros::Time leftTime;
ros::Time rightTime;

ros::Subscriber sub2DLeft;
ros::Subscriber sub2DRight;

ros::Publisher pub3D;

std::mutex mutex;

geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;

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
  assert(file.is_open());
  std::string line;
  while (getline(file, line)){
    std::cout << line << std::endl;
  }

  cal.intrinsic.resize(3, 3);
  cal.intrinsic.row(0) << 1280, 0, 512;
  cal.intrinsic.row(1) << 0, 1280, 384;
  cal.intrinsic.row(2) << 0, 0, 1;

  cal.rotation.resize(3, 3);
  cal.rotation.row(0) << 0, 1, 0;
  cal.rotation.row(1) << 0, 0, -1;
  cal.rotation.row(2) << -1, 0, 0;

  cal.translation.resize(3, 1);
  if (!index) cal.translation.col(0) << -0.373460680246, 0.724578678608, 10.2876386642;
  else cal.translation.col(0) << 0.373460680246, 0.724578678608, 10.2876386642;

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

void calc3DPose(){
  std::cout << "Calc 3d pos!" << std::endl;

  geometry_msgs::PointStamped pose3D;

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


  std::cout << "PX left:" << std::endl;
  std::cout << calL.PX << std::endl;
  std::cout << "px left:" << std::endl;
  std::cout << calL.px << std::endl;
  std::cout << "PX right:" << std::endl;
  std::cout << calR.PX << std::endl;
  std::cout << "px right:" << std::endl;
  std::cout << calR.px << std::endl;
  std::cout << "A:" << std::endl;
  std::cout << A << std::endl;
  std::cout << "B:" << std::endl;
  std::cout << B << std::endl;

  Eigen::MatrixXd x(3, 1);
  x = (pseudoInverse(A) * A).inverse() * pseudoInverse(A) * B;

  pose3D.point.x = x(0, 0);
  pose3D.point.y = x(1, 0);
  pose3D.point.z = x(2, 0);

  pose3D.header.stamp = pose2DLeft.header.stamp;

  pub3D.publish(pose3D);
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

    calL = loadCalibration("calibration.txt", 0);
    calR = loadCalibration("calibration.txt", 1);

    sub2DLeft = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_left", 1, receiveLeftImage);
    sub2DRight = nh.subscribe<geometry_msgs::PointStamped>("/pose/2d_right", 1, receiveRightImage);
    pub3D = nh.advertise<geometry_msgs::PointStamped>("/pose/3d", 1);

    ros::Time last = ros::Time::now();

    while (ros::ok()){
      last = ros::Time::now();
      ros::spinOnce();
  		rate.sleep();
    }

    return 0;
}
