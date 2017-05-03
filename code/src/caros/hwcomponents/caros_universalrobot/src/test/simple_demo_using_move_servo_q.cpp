#include <caros/test/universal_robot_test.h>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_servo_q");

  const double q_change = 0.2;

  UrTest ur_test;
  bool ret = false;
  ret = ur_test.testMoveServoQ(q_change);

  if (!ret)
  {
    ROS_ERROR_STREAM("Could not properly do the testMoveServoQ");
    return 1;
  }

  return 0;
}
