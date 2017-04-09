#include "squaremover_node.hpp"
#include <chrono>         // std::chrono::seconds

int main(int argc, char** argv)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ros::init(argc, argv, "RobotSq");
  MoveRobot moveRobot(false, true);
  ros::spin();
  return 0;
}
