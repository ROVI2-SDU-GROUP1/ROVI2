#include "squaremover_node.hpp"
#include <rw/math/Transform3D.hpp>
#include <chrono>         // std::chrono::seconds
#include <cstdlib>
#include <ctime>
#include <cmath>

MoveRobot::MoveRobot(bool shape, bool _preprocess)
: q_desired(6, 0, -1.6, 0, -1.5708, 0, 0), preprocess(_preprocess) //Start in upright position
{
    //init randomness
    srand(time(NULL));
	// Square motion
	/*
	UL:
		x: -0.201
		y: -0.191
		z:  0.933
	UR:
		x: 0.099
		y: -0.191
		z: 0.933
	DR:
		x: 0.099
		y: -0.191
		z:0.783
	DL:
		x: -0.201
		y: -0.191
		z: 0.783
	*/
    if(shape == false) //We move in a square
    {
    	this->toolPositions.push_back(rw::math::Vector3D<double>(-0.191,-0.201, 0.933)); // y,x,z - totalt retarderet...
    	this->toolPositions.push_back(rw::math::Vector3D<double>(-0.191, 0.099, 0.933)); // y,x,z - totalt retarderet...
    	this->toolPositions.push_back(rw::math::Vector3D<double>(-0.191,0.099, 0.783)); // y,x,z - totalt retarderet...
    	this->toolPositions.push_back(rw::math::Vector3D<double>(-0.191,-0.201, 0.783)); // y,x,z - totalt retarderet...
    }
    else //We Move in a circle
    {
        float resolution = 0.01; //Stepsize
        float radius = 0.15;
        //We are centered in 0,0,0.6
        for(float x = -radius; x <= radius; x+= resolution) //first half of circle
        {
            float y = sqrt(radius * radius - x * x);
            this->toolPositions.push_back(rw::math::Vector3D<double>(-0.25, y, x + 0.6)); // y,x,z
        }
        for(float x = radius; x >= -radius; x-= resolution) //second half of circle
        {
            float y = -sqrt(radius * radius - x * x);
            this->toolPositions.push_back(rw::math::Vector3D<double>(-0.25, y, x + 0.6)); // y,x,z
        }
        //Start with the bottom of the circle
    }
	this->Robot = nh_.serviceClient<caros_control_msgs::SerialDeviceMovePtp>("/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp");
    this->sub_robotFeedback = nh_.subscribe("/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1, &MoveRobot::RobotFeedbackCallback, this);


    // Auto load workcell
    std::cout << "Loading cell: " << SCENE_FILE << std::endl;
    this->_wc = rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE);
	this->_device = _wc->findDevice("UR1");
	this->_state = _wc->getDefaultState();


	this->ik_solver_ = new rw::invkin::JacobianIKSolver(_device, _state);
	this->ik_solver_->setEnableInterpolation(true);
	this->ik_solver_->setInterpolatorStep(0.001);

    //Set the inital robot state
    this->_device->setQ(this->q_desired, _state);
    SendQ(this->q_desired);

    if(this->preprocess)
    {
        for(size_t i = 0; i < this->toolPositions.size(); i++)
        {
            bool found_solution = false;
            while(found_solution == false)
            {
                rw::math::Transform3D<double> NewToolPosition(this->toolPositions[i], this->_device.get()->baseTend( this->_state).R());
              	std::vector<rw::math::Q> solutions = this->ik_solver_->solve(NewToolPosition, this->_state);
                std::cout << "Number of solutions found: " <<  solutions.size() << std::endl;
              	if(solutions.size() > 0)
                {
                    found_solution = true;
              		Q_positions.push_back(solutions.front());
                    this->_device->setQ(solutions.front(), this->_state);

              	}
                else
                { //if we can't find a solution, add a small random displacement to all joints.
                  //We don't output this on the robot, we just use it to solve the inverse kinematics.
                  //This is probably NOT a good way to fix this...
                  rw::math::Q cur_q  = this->_device->getQ(this->_state);
                  for(uint8_t i = 0; i < cur_q.size(); i++)
                  {
                      cur_q[i] += (rand() % 1000) / 1000000.;
                  }
                  this->_device->setQ(cur_q, this->_state);
                }
            }
        }
    }
    else
    {
        this->reverse_kin_thread = new std::thread(&MoveRobot::runner, this);
    }
}


MoveRobot::~MoveRobot()
{
    this->stop = true;
    if(this->reverse_kin_thread)
        this->reverse_kin_thread->join();
}

void MoveRobot::runner()
{
    while(!this->stop)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        this->state_position_lock.lock();
        if(this->in_position == false)
        {   //If we are not in the desired state yet, don't try to find the next
            this->state_position_lock.unlock();
            continue;
        }
        std::cout << "Finding a solution for position " << this->pos_counter << std::endl;
        while(this->in_position == true)
        {
            rw::math::Transform3D<double> NewToolPosition(this->toolPositions[this->pos_counter], this->_device.get()->baseTend( this->_state).R());
          	std::vector<rw::math::Q> solutions = this->ik_solver_->solve(NewToolPosition, this->_state);
            std::cout << "Number of solutions found: " <<  solutions.size() << std::endl;
          	if(solutions.size() > 0)
            {
                this->in_position = false;
          		this->q_desired = solutions.front();
          		SendQ(this->q_desired);
          	}
            else
            { //if we can't find a solution, add a small random displacement to all joints.
              //We don't output this on the robot, we just use it to solve the inverse kinematics.
              //This is probably NOT a good way to fix this...
              rw::math::Q cur_q  = this->_device->getQ(this->_state);
              for(uint8_t i = 0; i < cur_q.size(); i++)
              {
                  cur_q[i] += (rand() % 1000) / 1000000.;
              }
              this->_device->setQ(cur_q, this->_state);

            }
        }
        this->state_position_lock.unlock();
    }
}

void MoveRobot::RobotFeedbackCallback(const caros_control_msgs::RobotState::ConstPtr& data){
    //Try to obtain the lock.
    //If this is not possible, simply abort, as this means that the runner thread is not done calculating yet.
    if(this->state_position_lock.try_lock() == false) return;

    rw::math::Q qcurrent_(6, data->q.data[0], data->q.data[1],data->q.data[2], data->q.data[3],data->q.data[4], data->q.data[5]);
    auto diff = (qcurrent_ - this->q_desired).norm2(); // Difference between actual and desired state
    if(diff < 0.05)
    {   //pretty close..., maybe need to be more liberal on the real robot
        if(this->in_position == false) //Only increase counter if we were not in position beforehand.
            this->pos_counter = (this->pos_counter + 1) % this->toolPositions.size();
        this->_device->setQ(qcurrent_, this->_state);
        std::cout << "current position: " << this->_device.get()->baseTend(this->_state).P() << std::endl;
        if(this->preprocess)
        {   //If preprocessed, we set the new wanted configuration here
            this->q_desired = this->Q_positions[this->pos_counter];
            std::cout << this->pos_counter << " " << this->q_desired << std::endl;
            SendQ(this->q_desired);
        }
        else
        {
            this->in_position = true;
        }
    }
    this->state_position_lock.unlock(); //lose the lock.
}

void MoveRobot::SendQ(rw::math::Q q)
{
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

	srv.request.speeds.push_back(0.5);

	srv.request.blends.push_back(0.1);

	if (Robot.call(srv)){
		ROS_INFO("Sum: %ld", (long int)srv.response.success);
	} else	{
		ROS_ERROR("Failed to call service add_two_ints");
	}
}
