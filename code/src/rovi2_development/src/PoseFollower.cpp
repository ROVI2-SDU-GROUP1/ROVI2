#include <ros/ros.h>
#include "caros_control_msgs/SerialDeviceMoveServoQ.h"
#include "caros_common_msgs/Q.h"
#include "caros_control_msgs/RobotState.h"
#include <iostream>
#include <rw/rw.hpp>
#include <geometry_msgs/PointStamped.h>

#include "rw/kinematics/Kinematics.hpp"
#include "rw/math/MetricFactory.hpp"
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>

#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include <limits>
#include <cstdlib>
#include <cmath>
#include <mutex>


static std::mutex mtx;
static rw::models::WorkCell::Ptr wc;
static rw::models::Device::Ptr device;
static rw::kinematics::State state;
static ros::ServiceClient ur_service;
void SendQ(rw::math::Q q);


void pose_callback( __attribute__((unused)) const geometry_msgs::PointStamped::ConstPtr &msg)
{
  static rw::invkin::JacobianIKSolver *ik_solver = new rw::invkin::JacobianIKSolver(device, state);

  geometry_msgs::PointStamped this_pt = *msg;
  rw::math::Vector3D<double> goal(this_pt.point.x, this_pt.point.y, this_pt.point.z);
  rw::math::Rotation3D<double> rot( 0,-1,0, 0,0,-1, 1,0,0 );
  rw::math::Transform3D<double> NewToolPosition(goal, rot);

  std::cout << "trying to solve inverse kinematics" << std::endl;
  std::vector<rw::math::Q> solutions = ik_solver->solve(NewToolPosition, state);
  if(solutions.size() > 0)
  {
     SendQ(solutions[0]);
  }

}


void SendQ(rw::math::Q q)
{
    caros_control_msgs::SerialDeviceMoveServoQ srv;
    caros_common_msgs::Q Q;
    Q.data.push_back( q[0] );
    Q.data.push_back( q[1] );
    Q.data.push_back( q[2] );
    Q.data.push_back( q[3] );
    Q.data.push_back( q[4] );
    Q.data.push_back( q[5] );
    srv.request.targets.push_back(Q);
    srv.request.speeds.push_back(0.001);

	if (ur_service.call(srv)){
		ROS_INFO("CAROS ROSPONSE: %d", srv.response.success);
	} else	{
		ROS_ERROR("ERROR IN CAROS REQUEST");
	}
}


void rob_state_callback(const caros_control_msgs::RobotState::ConstPtr& data)
{
    rw::math::Q current_q(6, data->q.data[0], data->q.data[1],data->q.data[2], data->q.data[3],data->q.data[4], data->q.data[5]);
    device->setQ(current_q, state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_follower");
    ros::NodeHandle nh;


    wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
    device = wc->findDevice("UR1");
    state = wc->getDefaultState();

    ur_service = nh.serviceClient<caros_control_msgs::SerialDeviceMoveServoQ>("/ur_simple_demo_node/caros_serial_device_service_interface/move_servo_q");
    ros::Subscriber sub_robot_state(nh.subscribe("/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1, rob_state_callback));
    ros::Subscriber sub_pose(nh.subscribe<geometry_msgs::PointStamped>("/pose/3d",5, pose_callback));
    ros::spin();
}
