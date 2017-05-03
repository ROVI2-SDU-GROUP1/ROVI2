#include <caros/netft_node.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caros_netft");
  ros::NodeHandle nh("~");
  caros::NetFTNode ftsensor(nh);

  ftsensor.start();

  return 0;
}
