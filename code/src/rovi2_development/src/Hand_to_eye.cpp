#include "Hand_to_eye.hpp"
//  Pub
geometry_msgs::PointStamped pose2D_Left;
geometry_msgs::PointStamped pose2D_Right;

bool leftPressed = false;
bool rightPressed = false;

Hand_to_eye_node::Hand_to_eye_node( int argc, char *argv[]){
    _wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
    _device = _wc->findDevice("UR1");

    if (_device == NULL){
      ROS_WARN("Unable to load device");
    }else {
      ROS_INFO("Loading device");
    }

    ros::init(argc, (char **)argv, "hand_to_eye_calibration");
    ros::NodeHandle nh("~");

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
    nh.param<bool>("show_images", param_show_images, false);
    nh.param<bool>("save_images", param_save_images, false);
    nh.param<bool>("rectify_images", param_rectify_images, false);
    nh.param<std::string>("save_images_path_left", param_save_images_path_left, "/tmp/");
    nh.param<std::string>("save_images_path_right", param_save_images_path_right, "/tmp/");

    ROS_INFO("Use rectification: %d", param_rectify_images);
    ROS_INFO("Save images: %d", param_save_images);
    ROS_INFO("Show images: %d", param_show_images);

    std::string abs_yaml_path_left = std::string(CALIBRATION_DIR) + param_yaml_path_left;
    std::string abs_yaml_path_right = std::string(CALIBRATION_DIR) + param_yaml_path_right;

    YAMLCalibration yaml_calib_left(abs_yaml_path_left);
    YAMLCalibration yaml_calib_right(abs_yaml_path_right);

    image_height = yaml_calib_left.get_height();
    image_width = yaml_calib_left.get_width();

    cameraMatrixLeft = yaml_calib_left.get_camera_matrix();
    distCoeffsLeft = yaml_calib_left.get_distortion_coeffs();
    rectMatrixLeft = yaml_calib_left.get_rect_matrix();
    translationVectorLeft = yaml_calib_left.get_translation_vector();
    rotationMatrixLeft = yaml_calib_left.get_rotation_matrix();

    cameraMatrixRight = yaml_calib_right.get_camera_matrix();
    distCoeffsRight = yaml_calib_right.get_distortion_coeffs();
    rectMatrixRight = yaml_calib_right.get_rect_matrix();
    translationVectorRight = yaml_calib_right.get_translation_vector();
    rotationMatrixRight = yaml_calib_right.get_rotation_matrix();

    if(param_rectify_images){
      initRectifyMatrix();
    }

    pub_point_left = nh.advertise<geometry_msgs::PointStamped>(param_point_left, 1);
    pub_point_right = nh.advertise<geometry_msgs::PointStamped>(param_point_right, 1);
    pub_transform = nh.advertise<geometry_msgs::TransformStamped>(param_robot_transform, 1);

    sub_image_left =  new message_filters::Subscriber<sensor_msgs::Image>(nh, param_image_left, 1);
    sub_image_right = new message_filters::Subscriber<sensor_msgs::Image>(nh, param_image_right, 1);

    sub_q_callback = nh.subscribe(param_robot_state_sub, 1, &Hand_to_eye_node::robot_state_q_callback, this);

    sync_obj = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*sub_image_left, *sub_image_right, 20);
    sync_obj->registerCallback(boost::bind(&Hand_to_eye_node::image_sync_callback,this, _1, _2));
}


void Hand_to_eye_node::QToTransform(caros_control_msgs::RobotState &Q_state){
  rw::math::Q RW_Q_state(6, Q_state.q.data[0],Q_state.q.data[1],Q_state.q.data[2],Q_state.q.data[3],Q_state.q.data[4],Q_state.q.data[5] );
  _device->setQ(RW_Q_state, _state);

  auto tool_pos = _device->baseTframe(TcpFrame, _state).P();

  rw::math::Quaternion<double> TmpQuaternion = rw::math::Quaternion<double>(_device->baseTframe(TcpFrame, _state).R());

  geometry_msgs::TransformStamped qStateTransformed;
  qStateTransformed.transform.translation.x = tool_pos[0];
  qStateTransformed.transform.translation.y = tool_pos[1];
  qStateTransformed.transform.translation.z = tool_pos[2];

  qStateTransformed.transform.rotation.x = TmpQuaternion.getQx();
  qStateTransformed.transform.rotation.y = TmpQuaternion.getQy();
  qStateTransformed.transform.rotation.z = TmpQuaternion.getQz();
  qStateTransformed.transform.rotation.w = TmpQuaternion.getQw();

  qStateTransformed.header.stamp = pose2D_Left.header.stamp;
  ROS_INFO("Publishing transform");
  pub_transform.publish(qStateTransformed);
}

void CallBackFuncLeft(__attribute__((unused))int event, int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2D_Left.point.x = x;
    pose2D_Left.point.y = y;
    std::cout << "Left mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    leftPressed = true;
  }
}

void CallBackFuncRight(__attribute__((unused)) int event, __attribute__((unused)) int x, int y, __attribute__((unused))int flags, __attribute__((unused))void* userdata){
  if ( flags == cv::EVENT_FLAG_LBUTTON ){
    pose2D_Right.point.x = x;
    pose2D_Right.point.y = y;
    std::cout << "Right mouse button is clicked - position (" << x << ", " << y << ")" << std::endl;
    rightPressed = true;
  }
}

cv::Mat Hand_to_eye_node::Undistored(cv::Mat input, cv::Mat cameraMatrix, cv::Mat distCoeffs){
  cv::Mat undistorted;
  cv::undistort(input, undistorted, cameraMatrix, distCoeffs);
  return undistorted;
}

void Hand_to_eye_node::initRectifyMatrix(){
  cv::Mat R1, R2, P1, P2, Q;
  cv::Size imageSize = cv::Size(image_width, image_height);

  cv::Rect *region_of_interest_left = new cv::Rect();
  cv::Rect *region_of_interest_right = new cv::Rect();

  stereoRectify(cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight,distCoeffsRight, imageSize, rotationMatrixRight,translationVectorRight, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0,imageSize,region_of_interest_left,region_of_interest_right);

  initUndistortRectifyMap(cameraMatrixLeft, distCoeffsLeft, R1, P1, imageSize, CV_16SC2, map1Left, map2Left);
  initUndistortRectifyMap(cameraMatrixRight, distCoeffsRight, R2, P2, imageSize, CV_16SC2, map1Right, map2Right);
}

void Hand_to_eye_node::image_sync_callback(const sensor_msgs::Image::ConstPtr &image_left, const sensor_msgs::Image::ConstPtr &image_right){
  static int i = 0;
  pose2D_Left.header.stamp = image_left->header.stamp;
  pose2D_Right.header.stamp = image_left->header.stamp;

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

  cv::Mat img_l;
  cv::Mat img_r;
  if(param_rectify_images){
    remap(tmp_l, img_l, map1Left, map2Left, cv::INTER_LINEAR);
    remap(tmp_r, img_r, map1Right, map2Right, cv::INTER_LINEAR);
  }

  // Left
  std::string filename = param_save_images_path_left + "/left-" + std::to_string(i) + ".png";
  if(param_show_images){
      img_l = Undistored(tmp_l, cameraMatrixLeft, distCoeffsLeft);
      cv::imshow("Leftimage", img_l);
  }
  if(param_save_images){
    cv::imwrite( filename.c_str(), img_l );
  }

  // Right
  filename = param_save_images_path_right + "/right-" + std::to_string(i++) + ".png";
  if(param_show_images){
      img_r = Undistored(tmp_r, cameraMatrixRight, distCoeffsRight);
      cv::imshow("Rightimage", img_r );
  }
  if(param_save_images){
    cv::imwrite( filename.c_str(), img_r );
  }else{

  }

  cv::waitKey(1);

  if (leftPressed and rightPressed) {
    pub_point_left.publish(pose2D_Left);
    pub_point_right.publish(pose2D_Right);

    if(qState.q.data.size() > 0){
      QToTransform(qState);
    } else {
      ROS_INFO("No Q received");
    }

    leftPressed = false;
    rightPressed = false;
  }
  return;
}

void Hand_to_eye_node::robot_state_q_callback(const caros_control_msgs::RobotState::ConstPtr &q){
  qState = *q;
}


void Hand_to_eye_node::spin(){
  ros::spin();
}
