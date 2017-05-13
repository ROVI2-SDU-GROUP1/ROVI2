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
#include "yaml-cpp/yaml.h"

#include <fstream>

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

#include <rw/math/Quaternion.hpp>

caros_control_msgs::RobotState qState;

//  Pub
geometry_msgs::PointStamped pose2DLeft;
geometry_msgs::PointStamped pose2DRight;

static cv::Mat *cameraMatrixLeft;
static cv::Mat *cameraMatrixRight;
static cv::Mat *distCoeffsLeft;
static cv::Mat *distCoeffsRight;
static cv::Mat *rectMatrixLeft;
static cv::Mat *rectMatrixRight;

static cv::Mat *translationVectorRight;
static cv::Mat *rotationMatrixRight;

static cv::Mat *translationVectorLeft;
static cv::Mat *rotationMatrixLeft;

int image_width;
int image_height;

ros::Publisher pub_point_left;
ros::Publisher pub_point_right;
ros::Publisher pub_transform;

bool leftPressed = false;
bool rightPressed = false;

rw::models::WorkCell::Ptr _wc;
rw::models::Device::Ptr _device;
rw::kinematics::State _state;
rw::kinematics::MovableFrame* TcpFrame;

void QToTransform(caros_control_msgs::RobotState &Q_state){
  rw::math::Q RW_Q_state(6, Q_state.q.data[0],Q_state.q.data[1],Q_state.q.data[2],Q_state.q.data[3],Q_state.q.data[4],Q_state.q.data[5] );
  _device->setQ(RW_Q_state, _state);

  ROS_INFO("Before baseTframe");

  auto tool_pos = _device->baseTframe(TcpFrame, _state).P();

  rw::math::Quaternion<double> TmpQuaternion = rw::math::Quaternion<double>(_device->baseTframe(TcpFrame, _state).R());

  ROS_INFO("After Quaternion");

  geometry_msgs::TransformStamped qStateTransformed;
  qStateTransformed.transform.translation.x = tool_pos[0];
  qStateTransformed.transform.translation.y = tool_pos[1];
  qStateTransformed.transform.translation.z = tool_pos[2];

  qStateTransformed.transform.rotation.x = TmpQuaternion.getQx();
  qStateTransformed.transform.rotation.y = TmpQuaternion.getQy();
  qStateTransformed.transform.rotation.z = TmpQuaternion.getQz();
  qStateTransformed.transform.rotation.w = TmpQuaternion.getQw();

  qStateTransformed.header.stamp = pose2DLeft.header.stamp;
  ROS_INFO("Publishing transform");
  pub_transform.publish(qStateTransformed);
}

void CallBackFuncLeft(__attribute__((unused))int event, int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DLeft.point.x = x;
    pose2DLeft.point.y = y;
    std::cout << "Left mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    leftPressed = true;
  }
}

void CallBackFuncRight(__attribute__((unused)) int event, __attribute__((unused)) int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2DRight.point.x = x;
    pose2DRight.point.y = y;
    std::cout << "Right mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    rightPressed = true;
  }
}
//
cv::Mat Undistored(cv::Mat input, cv::Mat *cameraMatrix, cv::Mat *distCoeffs){
  cv::Mat undistorted;
  cv::undistort(input, undistorted, *cameraMatrix, *distCoeffs);
  return undistorted;
}

// , const caros_control_msgs::RobotState::ConstPtr &q
void image_sync_callback(const sensor_msgs::Image::ConstPtr &image_left, const sensor_msgs::Image::ConstPtr &image_right){
  pose2DLeft.header.stamp = image_left->header.stamp;
  pose2DRight.header.stamp = image_left->header.stamp;

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
  cv::Mat tmp_l = cv_ptr_left->image.clone();
  cv::Mat tmp_r = cv_ptr_right->image.clone();

  cv::Rect *region_of_interest_left = new cv::Rect();
  cv::Rect *region_of_interest_right = new cv::Rect();

  cv::Size imageSize = tmp_r.size();
  //cv::Mat rectLeft, map1Left, map2Left, rectRight, map1Right, map2Right;


 tmp_l = Undistored(tmp_l, cameraMatrixLeft, distCoeffsLeft);
 tmp_r = Undistored(tmp_r, cameraMatrixRight, distCoeffsRight);


  // cv::Mat R1, R2, P1, P2, Q;
  // cv::Mat rectLeft, map1Left, map2Left, map1Right, map2Right;
  //
  // stereoRectify(*cameraMatrixLeft, *distCoeffsLeft, *cameraMatrixRight,*distCoeffsRight, imageSize, *rotationMatrixRight,*translationVectorRight, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0,imageSize,region_of_interest_left,region_of_interest_right);
  //
  // initUndistortRectifyMap(*cameraMatrixLeft, *distCoeffsLeft, R1, P1, imageSize, CV_16SC2, map1Left, map2Left);
  // initUndistortRectifyMap(*cameraMatrixRight, *distCoeffsRight, R2, P2, imageSize, CV_16SC2, map1Right, map2Right);
  //
  // cv::Mat img_l;
  // cv::Mat img_r;
  // remap(tmp_l, img_l, map1Left, map2Left, cv::INTER_LINEAR);
  // remap(tmp_r, img_r, map1Right, map2Right, cv::INTER_LINEAR);
  //
  //
  //
  // std::cout << "P1: " << P1 << std::endl;
  // std::cout << "P2: " << P2 << std::endl;
  //
  // std::cout << "R1: " << R1 << std::endl;
  // std::cout << "R2: " << R2 << std::endl;




  //
  //cv::rectangle(img_l, *region_of_interest_left, cv::Scalar(255,0,0));
  cv::imshow("Leftimage", tmp_l);
  //
  //cv::rectangle(img_r, *region_of_interest_right,cv::Scalar(255,0,0));
  cv::imshow("Rightimage", tmp_r );
  cv::waitKey(1);

  if (leftPressed and rightPressed) {
    pub_point_left.publish(pose2DLeft);
    pub_point_right.publish(pose2DRight);

    if(qState.q.data.size() > 0){
      QToTransform(qState);
    } else {
      ROS_INFO("No Q received");
    }

    leftPressed = false;
    rightPressed = false;
  }
  return ;
}

bool is_file_exist(std::string fileName){
    std::ifstream infile(fileName.c_str());
    return infile.good();
}

void loadYAMLparameters( std::string yaml_path, cv::Mat *&cameraMatrix, cv::Mat *&distCoeffs, cv::Mat *&rectMatrix , cv::Mat *& translationVector, cv::Mat *& rotationMatrix){
  std::string abs_yaml_path = std::string(CALIBRATION_DIR) + yaml_path;
  std::cout << "Loading calib: " << abs_yaml_path << std::endl;
  if(!is_file_exist(abs_yaml_path)){
    ROS_WARN("File: %s does not exist\n", abs_yaml_path.c_str());
    exit(1);
  }

  YAML::Node calibration_yaml = YAML::LoadFile(abs_yaml_path);
  YAML::Node camera_matrix = calibration_yaml["camera_matrix"];
  YAML::Node distortion_coefficients = calibration_yaml["distortion_coefficients"];
  YAML::Node translation_vector = calibration_yaml["translation_vector"];
  YAML::Node rotation_matrix = calibration_yaml["rotation_matrix"];
  YAML::Node rectification_matrix = calibration_yaml["rectification_matrix"];

  image_width =  calibration_yaml["image_width"].as<int>();
  image_height =  calibration_yaml["image_height"].as<int>();


  cameraMatrix = new cv::Mat(3,3, CV_64FC1);
  distCoeffs = new cv::Mat(1,5, CV_64FC1);
  rectMatrix = new cv::Mat(3,3, CV_64FC1);
  translationVector = new cv::Mat(3,1, CV_64FC1);
  rotationMatrix = new cv::Mat(3,3, CV_64FC1);



  for(unsigned int i = 0; i < 3; i++){
    for(unsigned int j = 0; j < 3; j++){
      cameraMatrix->at<double>(i,j) = camera_matrix["data"][i*3+j].as<double>();
    }
  }

  for(unsigned int i = 0; i < 3; i++){
    for(unsigned int j = 0; j < 3; j++){
      rectMatrix->at<double>(i,j) = rectification_matrix["data"][i*3+j].as<double>();
    }
  }

  for(unsigned int i = 0; i < distortion_coefficients["data"].size();i++){
    distCoeffs->at<double>(i) = distortion_coefficients["data"][i].as<double>();
  }


  for(unsigned int i = 0; i < translation_vector.size(); i++){
    translationVector->at<double>(i) = translation_vector["data"][i].as<double>();
  }

  for(unsigned int y = 0; y < 3; y++){
    for(unsigned int x = 0; x < 3; x++){
      rotationMatrix->at<double>(y,x) = rotation_matrix["data"][y*3+x].as<double>();
    }
  }
}

void robot_state_q_callback(const caros_control_msgs::RobotState::ConstPtr &q){
  qState = *q;
}

int main(__attribute__((unused)) int argc, __attribute__((unused)) char *argv[]) {
    printf("Wow, it works!\n");
    printf("Compile info: GCC %u.%u.%u\t", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    printf("Compile date: %s -- %s\n", __DATE__, __TIME__);

    _wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	_device = _wc->findDevice("UR1");

    if (_device == NULL){
      ROS_WARN("Unable to load device");
    }else {
      ROS_WARN("Loading device");
    }

    ros::init(argc, (char **)argv, "hand_to_eye_calibration");
    ros::NodeHandle nh("~");
    //ros::Rate rate(20);

    if (_wc == NULL){
     ROS_WARN("Unable to load workcell: %s", SCENE_FILE);
    } else {
     ROS_INFO("Loading workcell");
    }

    std::string FindFrame = "WSG50.TCP";
    TcpFrame = (rw::kinematics::MovableFrame*)_wc->findFrame(FindFrame);
    if(TcpFrame == NULL){
      ROS_WARN("Frame %s not found!",FindFrame);
    }

   _state = _wc->getDefaultState();

  cv::namedWindow("Leftimage", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Rightimage", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("rectified", cv::WINDOW_AUTOSIZE);
  // cv::namedWindow("RectLeft", cv::WINDOW_NORMAL);
  // cv::namedWindow("RectRight", cv::WINDOW_NORMAL);

  cv::setMouseCallback("Leftimage", CallBackFuncLeft, NULL);
  cv::setMouseCallback("Rightimage", CallBackFuncRight, NULL);

  std::string param_yaml_path_left;
  std::string param_yaml_path_right;
  std::string param_image_left;
  std::string param_image_right;
  std::string param_point_left;
  std::string param_point_right;
  std::string param_robot_transform;
  std::string param_robot_state_sub;

  nh.param<std::string>("calibration_yaml_path_left", param_yaml_path_left, "default.yaml");
  nh.param<std::string>("calibration_yaml_path_right", param_yaml_path_right, "default.yaml");
  nh.param<std::string>("image_left", param_image_left, "/camera/left/image_raw");
  nh.param<std::string>("image_right", param_image_right, "/camera/right/image_raw");
  nh.param<std::string>("sub_robot_state", param_robot_state_sub, "/ur_simple_demo_node/caros_serial_device_service_interface/robot_state");
  nh.param<std::string>("pub_point_left", param_point_left, "/pose/2d_left");
  nh.param<std::string>("pub_point_right", param_point_right, "/pose/2d_right");
  nh.param<std::string>("pub_robot_transform", param_robot_transform, "/robot_transform");

  loadYAMLparameters(param_yaml_path_left, cameraMatrixLeft, distCoeffsLeft, rectMatrixLeft, translationVectorLeft, rotationMatrixLeft);
  loadYAMLparameters(param_yaml_path_right, cameraMatrixRight, distCoeffsRight, rectMatrixRight, translationVectorRight, rotationMatrixRight);

  pub_point_left = nh.advertise<geometry_msgs::PointStamped>(param_point_left, 1);
  pub_point_right = nh.advertise<geometry_msgs::PointStamped>(param_point_right, 1);
  pub_transform = nh.advertise<geometry_msgs::TransformStamped>(param_robot_transform, 1);

  message_filters::Subscriber<sensor_msgs::Image> sub_image_left(nh, param_image_left, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_right(nh, param_image_right, 1);
  //message_filters::Subscriber<caros_control_msgs::RobotState> sub_q_state(nh, param_robot_state_sub, 0);
  ros::Subscriber sub = nh.subscribe(param_robot_state_sub, 1, robot_state_q_callback);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image_left, sub_image_right, 20);
  sync.registerCallback(boost::bind(&image_sync_callback, _1, _2));

  // while (ros::ok()){
  //   ros::spinOnce();
	// 	rate.sleep();
  // }
  ros::spin();

  return 0;
}
