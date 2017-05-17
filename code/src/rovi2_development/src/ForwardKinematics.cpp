#include "ForwardKinematics.hpp"


ForwardKinematics::ForwardKinematics( int argc, char *argv[]){
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

    std::string FindFrame = "WSG50.Marker";
    TcpFrame = (rw::kinematics::MovableFrame*)_wc->findFrame(FindFrame);
    if(TcpFrame == NULL){
      ROS_WARN("Frame %s not found!",FindFrame.c_str());
    }

    _state = _wc->getDefaultState();

    std::string param_robot_state_sub;
    std::string param_robot_transform_pub;

    nh.param<std::string>("param_robot_state_sub", param_robot_state_sub, "param_robot_state_sub");
    nh.param<std::string>("param_robot_transform_pub", param_robot_transform_pub, "param_robot_transform_pub");

    pub_transform = nh.advertise<geometry_msgs::TransformStamped>(param_robot_transform_pub, 1);
    sub_q = nh.subscribe(param_robot_state_sub, 1, &ForwardKinematics::robot_state_q_callback, this);
}


void ForwardKinematics::QToTransform(caros_control_msgs::RobotState Q_state){
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

  ROS_INFO("Publishing transform");
  pub_transform.publish(qStateTransformed);
}


void ForwardKinematics::robot_state_q_callback(const caros_control_msgs::RobotState::ConstPtr &q){
  QToTransform(*q);
}


void ForwardKinematics::spin(){
  ros::spin();
}
