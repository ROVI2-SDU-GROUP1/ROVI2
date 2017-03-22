#pragma once

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
#include <rw/math/Vector3D.hpp>
#include <thread>         // std::thread
#include <mutex>

class MoveRobot
{
    public:
        MoveRobot();
        ~MoveRobot();
        void RobotFeedbackCallback(const caros_control_msgs::RobotState::ConstPtr& data);
        void SendQ(rw::math::Q q);

    private:
        bool stop = false;
        void runner();
        std::mutex state_position_lock;
        std::thread *reverse_kin_thread = nullptr;
    	ros::NodeHandle nh_;
    	ros::ServiceClient Robot;
        ros::Subscriber sub_robotFeedback;
        rw::math::Q q_desired; //the currently desired Q.
        bool in_position = false;


        size_t pos_counter = 0;
        std::vector<rw::math::Vector3D<double>> toolPositions;

        rw::models::WorkCell::Ptr _wc;
        rw::models::Device::Ptr _device;
        rw::kinematics::State _state;
    	rw::invkin::JacobianIKSolver * ik_solver_;
};
