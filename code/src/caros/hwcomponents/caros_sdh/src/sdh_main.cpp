#include <caros/sdh_node.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "caros_sdh");

  ros::NodeHandle nh("~");

  caros::SDHNode sdh_node(nh);

  sdh_node.start();

  return 0;
}
