#include <RT_RRT_Star.hpp>
#include <ros/ros.h>
#include <rovi2_development/Trajectory3D.h>
#include "caros_control_msgs/SerialDeviceMovePtp.h"
#include "caros_common_msgs/Q.h"
#include "caros_control_msgs/RobotState.h"
#include "RT_RRT_Star.hpp"
#include <iostream>
#include <rw/rw.hpp>

#include "rw/kinematics/Kinematics.hpp"
#include "rw/math/MetricFactory.hpp"
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>

#include "rwlibs/proximitystrategies/ProximityStrategyFactory.hpp"
#include <limits>
#include <cstdlib>
#include <cmath>
#include <ElipsisSampler.hpp>
#include <thread>
#include <mutex>
#include <limits>

struct Plane3d
{
    public:
    Eigen::Vector3d norm_vec;
    Eigen::Vector3d point;
};

struct Trajectory3d
{
    public:
    Eigen::Vector3d acceleration;
    Eigen::Vector3d velocity;
    Eigen::Vector3d position;
};

int sign(double n)
{
    if(n > 0) return 1;
    if(n < 0) return -1;
    return 0;
}
double distance_to_plane(const Plane3d &plane, const Eigen::Vector3d &point)
{
    return plane.norm_vec.cwiseProduct(point - plane.point).sum()  / plane.norm_vec.norm();
}

double find_plane_parobola_interception(const Plane3d &plane, const Trajectory3d &traj, double t_min = 0,
    double t_max = 10, double stepsize = 1, double min_stepsize = 0.0000000001)
{
    double t_best = 0;
    double min_distance = distance_to_plane(plane, traj.position);

    for(double t = t_min; t < t_max; t += stepsize)
    {
        Eigen::Vector3d p = 0.5 * traj.acceleration * t * t + traj.velocity * t + traj.position;
        double distance = distance_to_plane(plane, p);
        std::cout << "distance " << distance << "\t" << sign(distance) << std::endl;
        double tmp_min = min_distance;
        if(std::fabs(tmp_min) > std::fabs(distance))
        {
            min_distance = distance;
            t_best = t;
        }
        if(sign(tmp_min) != sign(distance))
        {
            if(stepsize <= min_stepsize)
            {
                return t_best;
            }
            return find_plane_parobola_interception(plane, traj, t - stepsize, t, stepsize / 10, min_stepsize);
        }
    }
    return t_best;
}

rw::math::Q get_point_on_line(rw::math::Q a, rw::math::Q b, rw::math::Q p)
{
    auto ap = (p - a).e();
    auto ab = (b - a).e();
    auto new_point = rw::math::Q(a.e() + ap.dot(ab) / ab.dot(ab)  *ab);
    if((b - a).norm2() < (new_point - a).norm2()) return b; //cap at b
    return new_point;
}

std::vector<rw::math::Q> force_find_ik(rw::invkin::JacobianIKSolver *ik_solver, rw::math::Transform3D<double> goal_state,
    rw::kinematics::State *state, rw::models::Device::Ptr device)
{
    rw::math::Q tmp_q  = device->getQ(*state);

    bool found_solution = false;
    while(found_solution == false)
    {
        std::vector<rw::math::Q> solutions = ik_solver->solve(goal_state, *state);
        if(solutions.size() > 0)
        {
            device->setQ(tmp_q, *state);
            return solutions;
        }
        else
        { //if we can't find a solution, add a small random displacement to all joints.
          //We don't output this on the robot, we just use it to solve the inverse kinematics.
          //This is probably NOT a good way to fix this...
          rw::math::Q cur_q  = device->getQ(*state);
          for(uint8_t i = 0; i < cur_q.size(); i++)
          {
              cur_q[i] += (rand() % 1000) / 1000000.;
          }
          device->setQ(cur_q, *state);
        }
    }
    return std::vector<rw::math::Q>();
}

class RobotPlanner
{
    public:
        RobotPlanner();
        ~RobotPlanner() {};
    private:
        rw::math::Q find_new_goal(Plane3d intersect_plane, Trajectory3d traj, bool *found_new_goal);
        void rob_state_callback(const caros_control_msgs::RobotState::ConstPtr& data);
        void trajectory_callback(const rovi2_development::Trajectory3D &parameters);
        void rt_rrt_runner();
        rw::models::WorkCell::Ptr wc;
        rw::models::Device::Ptr device;
        rw::math::Metric<rw::math::Q>::Ptr metric;
        rw::proximity::CollisionDetector *detector = nullptr;
        rw::kinematics::State state;
        rw::pathplanning::QConstraint::Ptr constraint;
        rw::math::QMetric::Ptr norm_metric;
        rw::pathplanning::QEdgeConstraint::Ptr edge_constraint;
        rw::pathplanning::PlannerConstraint p_constraint;
        rw::pathplanning::QSampler::Ptr sampler;
        rw::invkin::JacobianIKSolver *ik_solver;

        ros::NodeHandle nh;
        ros::Subscriber sub_filtered;
        ros::ServiceClient ptp_publisher;
        ros::Subscriber sub_robot_state;
        std::thread *rt_rrt_thread;
        std::mutex cur_q_lock;
        RT_RRT_Star *rt_rrt_star_planner = nullptr;
        bool update_agent = false;
        bool update_goal = false;
        rw::math::Q current_q;
        rw::math::Q next_goal;
        std::pair<RT_Node *, RT_Node *> current_edge;
        std::vector<RT_Node *> cur_path;
        Eigen::Vector3d last_goal_point;
};

RobotPlanner::RobotPlanner()
//Robwork stuff (wtf)
:
wc(rw::loaders::WorkCellLoader::Factory::load(SCENE_FILE)),
device(this->wc->findDevice("UR1")),
metric(rw::math::MetricFactory::makeEuclidean<rw::math::Q>()),
detector(new rw::proximity::CollisionDetector(this->wc,
rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy())),
state( this->wc->getDefaultState() ),
constraint(rw::pathplanning::QConstraint::make(this->detector, this->device, this->state)),
norm_metric(rw::pathplanning::PlannerUtil::normalizingInfinityMetric(this->device->getBounds())),
edge_constraint(rw::pathplanning::QEdgeConstraint::make(this->constraint, this->norm_metric, 0.02)),
p_constraint(rw::pathplanning::PlannerConstraint::make(constraint, edge_constraint)),
sampler(rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::QSampler::makeUniform(this->device), this->p_constraint.getQConstraintPtr())),

//ROS stuff
sub_filtered(nh.subscribe("/pose/parameter",1, &RobotPlanner::trajectory_callback, this)),
ptp_publisher(nh.serviceClient<caros_control_msgs::SerialDeviceMovePtp>("/ur_simple_demo_node/caros_serial_device_service_interface/move_ptp")),
sub_robot_state(nh.subscribe("/ur_simple_demo_node/caros_serial_device_service_interface/robot_state", 1, &RobotPlanner::rob_state_callback, this)),

//Other stuff
last_goal_point(0,0,0)
{
    this->ik_solver = new rw::invkin::JacobianIKSolver(this->device, this->state);
    this->ik_solver->setEnableInterpolation(true);
	this->ik_solver->setInterpolatorStep(0.001);
    this->current_q = device->getQ(this->state);
    this->next_goal = device->getQ(this->state);
    //We have not found a trajectory for the ball yet, start building a tree to 0,0,0.40 (x,y,z)
    rw::math::Vector3D<double> init_goal(0,-0.2, 0.6);
    rw::math::Rotation3D<double> init_rotation( 0,-1,0, 0,0,-1, 1,0,0 );
    rw::math::Transform3D<double> NewToolPosition(init_goal, init_rotation);

    std::cout << "trying to solve inverse kinematics" << std::endl;
    std::vector<rw::math::Q> solutions = force_find_ik(this->ik_solver, NewToolPosition, &this->state, this->device);
    std::cout << solutions[0] << std::endl;
    std::cout << "solved" << std::endl;

    assert(solutions.size());
    this->rt_rrt_star_planner = new RT_RRT_Star(this->device->getQ(this->state), solutions[0], this->p_constraint, sampler, metric, device);

    this->rt_rrt_thread = new std::thread(&RobotPlanner::rt_rrt_runner, this);

}

void RobotPlanner::rt_rrt_runner(void)
{
    while(true)
    {
        //If we have not found a trajectory for the ball yet, start building a tree to 0,0,40 (x,y,z)
        //this->rt_rrt_star_planner(q_1, q_2, p_constraint, sampler, metric, device);
        //We start the rrt now!
        std::chrono::milliseconds time_to_solve{1000};
        if(this->update_goal == true)
        {
            this->cur_q_lock.lock();
            rw::math::Q new_goal = this->next_goal;
            this->rt_rrt_star_planner->set_new_goal(new_goal);
            this->cur_q_lock.unlock();
        }
        if(this->update_agent)
        {
            this->cur_q_lock.lock();
            auto cur_copy = this->current_q;
            rw::math::Q new_node = get_point_on_line(this->current_edge.first->getValue(), this->current_edge.second->getValue(), cur_copy);
            this->cur_q_lock.unlock();
            std::cout << "New: " << new_node << std::endl;
            auto new_agent = this->rt_rrt_star_planner->split_edge_with_point(new_node,
                this->current_edge.first, this->current_edge.second);
            this->rt_rrt_star_planner->move_agent(new_agent);
        }
        auto new_path = this->rt_rrt_star_planner->find_next_path(time_to_solve, true);
        for(auto node : new_path)
        {
            std::cout << node->getValue() << std::endl;
        }
        //Simulate update of q
        this->cur_path = new_path;
        this->current_edge = std::pair<RT_Node *, RT_Node *>(new_path[0], new_path[1]);
        this->update_agent = true;
        this->current_q = LineSampler::get_instance(new_path[0]->getValue(), new_path[1]->getValue())->doSample() * 0.5;
        std::cout << new_path[0]->getValue() << "\t" << new_path[1]->getValue() << "\t" << this->current_q << std::endl;
        std::cout << "tree size: " << this->rt_rrt_star_planner->get_size()  <<  std::endl;
    }
}

void RobotPlanner::rob_state_callback(const caros_control_msgs::RobotState::ConstPtr& data)
{
    this->cur_q_lock.lock();
    this->current_q = rw::math::Q(6, data->q.data[0], data->q.data[1],data->q.data[2], data->q.data[3],data->q.data[4], data->q.data[5]);
    this->cur_q_lock.unlock();

    std::cout << data << std::endl;
}

void RobotPlanner::trajectory_callback(const rovi2_development::Trajectory3D &parameters)
{
    //The plane we want to intercept the ball in.
    Eigen::Vector3d plane_normal(0,1,1);
    Eigen::Vector3d plane_point(0,1,1);
    Plane3d plane = {plane_normal, plane_point};

    //The trajectory as eigen matrix
    Eigen::Vector3d traj_acc(parameters.acc.x, parameters.acc.y, parameters.acc.z);
    Eigen::Vector3d traj_vel(parameters.vel.x, parameters.vel.y, parameters.vel.z);
    Eigen::Vector3d traj_point(parameters.pos.x, parameters.pos.y, parameters.pos.z);
    Trajectory3d trajectory = {traj_acc, traj_vel, traj_point};
    bool new_goal = false;
    rw::math::Q _next_goal = this->find_new_goal(plane, trajectory, &new_goal);
    if(new_goal)
    {
        //Update the goal, agent, and force a replanning of the path
        this->cur_q_lock.lock();
        this->next_goal = _next_goal;
        this->update_goal = true;
        this->update_agent = true;
        this->rt_rrt_star_planner->force_stop();
        this->cur_q_lock.unlock();
    }
}
rw::math::Q RobotPlanner::find_new_goal(Plane3d intersect_plane, Trajectory3d traj, bool *found_new_goal)
{
    *found_new_goal = false;
    //Find the best plane intersection.
    double cath_time = find_plane_parobola_interception(intersect_plane, traj);
    std::cout << "We need to catch the ball in " << cath_time << " seconds!" << std::endl;

    Eigen::Vector3d p = 0.5 * traj.acceleration * cath_time * cath_time + traj.velocity * cath_time + traj.position;
    if( (p - this->last_goal_point).norm() < 10) return rw::math::Q(); //Return if goal have not changed by much.
    //Find the point closest to this where there is a collision free poisition.
    for(double t_offset = 0; t_offset < 2; t_offset += 0.01)
    {
        double t = cath_time + t_offset;
        p = 0.5 * traj.acceleration * t * t + traj.velocity * t + traj.position;
        rw::math::Vector3D<double> goal(p);
        rw::math::Rotation3D<double> rotation( 0,-1,0, 0,0,-1, 1,0,0 ); //It would be nice to rotate the tool so we the "bat" is perpendicular to the p'(t)
        rw::math::Transform3D<double> NewToolPosition(goal, rotation);
        std::vector<rw::math::Q> solutions = ik_solver->solve(NewToolPosition, this->state);
        if(solutions.size())
        {
            *found_new_goal = true;
            return solutions[0];
        }
        t = cath_time - t_offset;
        p = 0.5 * traj.acceleration * t * t + traj.velocity * t + traj.position;
        goal = rw::math::Vector3D<double>(p);
        rotation =rw::math::Rotation3D<double>( 0,-1,0, 0,0,-1, 1,0,0 ); //It would be nice to rotate the tool so we the "bat" is perpendicular to the p'(t)
        NewToolPosition = rw::math::Transform3D<double>(goal, rotation);
        solutions = ik_solver->solve(NewToolPosition, this->state);
        if(solutions.size())
        {
            *found_new_goal = true;
            return solutions[0];
        }
    }
    return rw::math::Q();
}

int main(int argc, char **argv) {
    rw::math::Math::seed(time(NULL)); //seed robwork with current time
    ros::init(argc, argv, "rovi2_pathplanner");
    RobotPlanner planner;
    ros::spin();
    return 0;
}
