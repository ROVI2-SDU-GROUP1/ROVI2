#include <RT_RRT_Star.hpp>
#include <ros/ros.h>
#include <rovi2_development/Trajectory3D.h>
#include "caros_control_msgs/SerialDeviceMovePtp.h"
#include "caros_common_msgs/Q.h"
#include "caros_control_msgs/RobotState.h"


void rob_state_callback(const caros_control_msgs::RobotState::ConstPtr& data)
{
    std::cout << data << std::endl;
}

void trajectory_callback(const rovi2_development::Trajectory3D &parameters)
{
    std::cout << parameters << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rovi2_pathplanner");
    ros::NodeHandle nh;
    //Subscribe on kalman filter result.
    ros::Subscriber sub_filtered = nh.subscribe("/pose/parameter",1, &trajectory_callback);
    //Publish to ur ptp
    ros::ServiceClient ptp_publisher = nh.serviceClient<caros_control_msgs::SerialDeviceMovePtp>("/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp");
    //Subscribe on the robot state
    ros::Subscriber sub_robot_state = nh.subscribe("/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1, &rob_state_callback);

    ros::spin();
    return 0;
}
