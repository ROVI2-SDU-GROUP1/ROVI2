#include <caros/trakstar_node.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caros_trakstar_node");

  caros::TrakstarNode tstar;
  tstar.start();

  return 0;
}
