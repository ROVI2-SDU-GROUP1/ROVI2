#include <ros/ros.h>
#include <caros/robotiq_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caros_robotiq3");

  ros::NodeHandle nh("~");
  caros::RobotiqNode hand(nh, caros::RobotiqNode::HandType::ROBOTIQ3);
  hand.start();

  return 0;
}
