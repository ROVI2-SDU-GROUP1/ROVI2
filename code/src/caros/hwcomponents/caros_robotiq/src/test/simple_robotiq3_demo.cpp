#include <caros/gripper_si_proxy.h>
#include <caros_common_msgs/CarosNodeState.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>

#include <string>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_robotiq3_demo");
  ros::NodeHandle n("~");

  const std::string node_under_test_name = "caros_robotiq3";

  const std::string info_prefix = ros::this_node::getName() + ": ";

  ROS_INFO_STREAM(ros::this_node::getName() << " started!");

  ROS_INFO_STREAM(info_prefix << "Setting up GripperSIProxy");
  caros::GripperSIProxy r3_test(n, node_under_test_name);
  ROS_INFO_STREAM(info_prefix << "GripperSIProxy setup. The hand should be initializing if it was not before!");

  const std::string node_stateTopicName = node_under_test_name + "/" + "caros_node/caros_node_state";
  bool is_running = false;
  while (!is_running)
  {
    auto node_state_message = ros::topic::waitForMessage<caros_common_msgs::CarosNodeState>(node_stateTopicName);
    is_running = node_state_message->state == "RUNNING";
  }

  ROS_INFO_STREAM(info_prefix << "The hand should be initialized and ready to use now!");

  ros::spinOnce();

  ROS_INFO_STREAM(info_prefix << "The hand is now at: " << r3_test.getQ());

  rw::math::Q target(4, 200, 200, 200, 100);
  ROS_INFO_STREAM(info_prefix << "Trying to move to: " << target);

  bool ret = r3_test.moveQ(target);

  if (!ret)
  {
    ROS_ERROR_STREAM(info_prefix << "Could not properly move the hand");
    return 1;
  }
  else
  {
    ROS_INFO_STREAM(info_prefix << "Movement command successfully accepted");
  }

  rw::math::Q current = r3_test.getQ();

  while (ros::ok() && current != target)
  {
    ros::Duration(1).sleep();  // In seconds
    ros::spinOnce();
    current = r3_test.getQ();
    ROS_INFO_STREAM(info_prefix << "The hand is now at: " << r3_test.getQ());
  }

  ROS_WARN_STREAM(info_prefix << "This node will now end. This is intended behavior. When used with the test script "
                                 "this will lead to an error message. Please ignore that.");

  return 0;
}
