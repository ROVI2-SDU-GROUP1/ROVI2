
#include <ros/ros.h>
#include "caros_control_msgs/SerialDeviceMovePtp.h"
#include "caros_common_msgs/Q.h"
#include "caros_control_msgs/RobotState.h"

/* RW */
#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <rw/math.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>


class MoveRobot {
	ros::NodeHandle nh_;
	ros::ServiceClient Robot;
  ros::Subscriber sub_robotFeedback;

	// Houseclearning
	bool firstRun = true;


  int i = 0;
  std::vector<rw::math::Vector3D<double>> toolPositions;

  rw::models::WorkCell::Ptr _wc;
  rw::models::Device::Ptr _device;
  rw::kinematics::State _state;
	rw::invkin::JacobianIKSolver * ik_solver_;


public:

	MoveRobot(){
    toolPositions.push_back(rw::math::Vector3D<double>(0,-0.2, 0.858));
    toolPositions.push_back(rw::math::Vector3D<double>(0,-0.191, 1.00));

		Robot = nh_.serviceClient<caros_control_msgs::SerialDeviceMovePtp>("/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp");
    sub_robotFeedback = nh_.subscribe("/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1, &MoveRobot::RobotFeedbackCallback, this);


    // Auto load workcell
    std::cout << "Loading cell: " << SCENE_FILE << std::endl;
    _wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
  	_device = _wc->findDevice("UR1");
  	_state = _wc->getDefaultState();


		ik_solver_ = new rw::invkin::JacobianIKSolver(_device, _state);
		ik_solver_->setEnableInterpolation(true);
		ik_solver_->setInterpolatorStep(0.001);

	}

  void RobotFeedbackCallback(const caros_control_msgs::RobotState::ConstPtr& data){
    rw::math::Q qcurrent_(6, data->q.data[0], data->q.data[1],data->q.data[2], data->q.data[3],data->q.data[4], data->q.data[5]);
    _device->setQ(qcurrent_, _state);
    auto diff = (_device.get()->baseTend(_state).P() - toolPositions[i]).norm2(); // Difference between actual and desired state
		std::cout << "i: " << i <<  " diff: " << diff << " current: " << _device.get()->baseTend(_state).P() << " , desired: " << toolPositions[i] << std::endl;

		if(data->header.seq %6 == 0){

			if(firstRun){
					firstRun = false;
					rw::math::Q tmp(6, 0, -1.6, 0, -1.5708, 0, 0);
					_device->setQ(tmp, _state);
					SendQ(tmp);
					return;
			}

	    if(diff < 0.05){ // 1 cm
	      i++;
	    }

	    if(i >= toolPositions.size()){
	      i = 0;
	    }

	  	rw::math::Transform3D<double> NewToolPosition(toolPositions[i],_device.get()->baseTend(_state).R());

	  	std::vector<rw::math::Q> solutions = ik_solver_->solve(NewToolPosition, _state);


	  	std::cout << "Number of solutions found: " <<  solutions.size() << std::endl;
	  	if(solutions.size() > 0){
	  		rw::math::Q closest = solutions.front();
	  		SendQ(closest);
	  	}
		}
  }

	void SendQ(rw::math::Q q){
		//beginner_tutorials::AddTwoInts srv;
		caros_control_msgs::SerialDeviceMovePtp srv;

		caros_common_msgs::Q Q;
    Q.data.push_back( q[0] );
    Q.data.push_back( q[1] );
    Q.data.push_back( q[2] );
    Q.data.push_back( q[3] );
    Q.data.push_back( q[4] );
    Q.data.push_back( q[5] );


		srv.request.targets.push_back(Q);

		srv.request.speeds.push_back(0.1);

		srv.request.blends.push_back(0.1);

		if (Robot.call(srv)){
			ROS_INFO("Sum: %ld", (long int)srv.response.success);
		} else	{
			ROS_ERROR("Failed to call service add_two_ints");
		}
	}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotSq");
  MoveRobot moveRobot;
  ros::spin();
  return 0;
}
