#include <caros/camera.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "caros_camera");

  ros::NodeHandle nh("~");

  caros::Camera camera(nh);
  camera.start();

  return 0;
}
