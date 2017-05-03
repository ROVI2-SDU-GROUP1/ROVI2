#include <caros/schunkpg70_node.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "caros_schunkpg70");

  ros::NodeHandle nh("~");

  caros::SchunkPG70Node schunkpg70_node(nh);

  schunkpg70_node.start();

  return 0;
}
