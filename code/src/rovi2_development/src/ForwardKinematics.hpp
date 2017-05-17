#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <caros_control_msgs/RobotState.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <limits>

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

class ForwardKinematics{
public:
  ForwardKinematics( int argc,  char *argv[]);
  void robot_state_q_callback(const caros_control_msgs::RobotState::ConstPtr &q);
  void spin();



private:
  void QToTransform(caros_control_msgs::RobotState Q_state);

  ros::Publisher pub_transform;
  ros::Subscriber sub_q;

  rw::models::WorkCell::Ptr _wc;
  rw::models::Device::Ptr _device;
  rw::kinematics::State _state;
  rw::kinematics::MovableFrame* TcpFrame;
};
