#include <caros/pose_teleoperate.h>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caros_teleoperation");

  ros::NodeHandle nh("~");
  std::string name = nh.getNamespace();
  std::cout << "Namespace " << name << std::endl;
  caros::PoseTeleoperate platform(nh, name);
  platform.start();

  return 0;
}
