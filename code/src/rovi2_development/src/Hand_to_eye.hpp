#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <caros_control_msgs/RobotState.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <limits>
#include "YAMLCalibration/YAMLCalibration.hpp"


#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>



void CallBackFuncLeft(__attribute__((unused))int event, int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata);
void CallBackFuncRight(__attribute__((unused)) int event, __attribute__((unused)) int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata);

class Hand_to_eye_node{
public:
  Hand_to_eye_node( int argc,  char *argv[]);
  void spin();



private:
  void QToTransform(caros_control_msgs::RobotState &Q_state);
  //void CallBackFuncLeft(__attribute__((unused))int event, int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata);
  //void CallBackFuncRight(__attribute__((unused)) int event, __attribute__((unused)) int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata);
  cv::Mat Undistored(cv::Mat input, cv::Mat cameraMatrix, cv::Mat distCoeffs);
  void initRectifyMatrix();
  void image_sync_callback(const sensor_msgs::Image::ConstPtr &image_left, const sensor_msgs::Image::ConstPtr &image_right);
  void robot_state_q_callback(const caros_control_msgs::RobotState::ConstPtr &q);

  caros_control_msgs::RobotState qState;

  cv::Mat cameraMatrixLeft;
  cv::Mat cameraMatrixRight;
  cv::Mat distCoeffsLeft;
  cv::Mat distCoeffsRight;
  cv::Mat rectMatrixLeft;
  cv::Mat rectMatrixRight;

  cv::Mat translationVectorRight;
  cv::Mat rotationMatrixRight;

  cv::Mat translationVectorLeft;
  cv::Mat rotationMatrixLeft;

  int image_width;
  int image_height;

  bool param_show_images;
  bool param_save_images;
  bool param_rectify_images;

  std::string param_save_images_path_left;
  std::string param_save_images_path_right;

  ros::Publisher pub_point_left;
  ros::Publisher pub_point_right;
  ros::Publisher pub_transform;


  ros::Subscriber sub_q_callback;
  message_filters::Subscriber<sensor_msgs::Image>* sub_image_left;
  message_filters::Subscriber<sensor_msgs::Image>* sub_image_right;


  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *sync_obj;

  rw::models::WorkCell::Ptr _wc;
  rw::models::Device::Ptr _device;
  rw::kinematics::State _state;
  rw::kinematics::MovableFrame* TcpFrame;

  cv::Mat map1Left, map2Left, map1Right, map2Right;

};
