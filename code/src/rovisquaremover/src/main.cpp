#include "squaremover_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotSq");
  MoveRobot moveRobot;
  ros::spin();
  return 0;
}
