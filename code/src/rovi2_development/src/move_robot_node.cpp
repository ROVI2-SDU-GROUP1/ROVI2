#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "caros_control_msgs/SerialDeviceMovePtp.h"
#include "caros_common_msgs/Q.h"

class MoveRobot {
	ros::NodeHandle nh_;
	ros::ServiceClient Robot;
public:

	MoveRobot(){
		Robot = nh_.serviceClient<caros_control_msgs::SerialDeviceMovePtp>("/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp");
	}

	void SendQ(){
		//beginner_tutorials::AddTwoInts srv;
		caros_control_msgs::SerialDeviceMovePtp srv;

		caros_common_msgs::Q q;
		q.data.push_back(0);
		q.data.push_back(0);
		q.data.push_back(0);
		q.data.push_back(0);
		q.data.push_back(0);
		q.data.push_back(0);


		srv.request.targets.push_back(q);

		srv.request.speeds.push_back(0.1);

		srv.request.blends.push_back(0.1);

		if (Robot.call(srv)){
			ROS_INFO("Sum: %ld", (long int)srv.response.success);
		} else	{
			ROS_ERROR("Failed to call service add_two_ints");
		}
	}

  ~ MoveRobot(){
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotMover");
  MoveRobot moveRobot;
  moveRobot.SendQ();
  ros::spin();
  return 0;
}

