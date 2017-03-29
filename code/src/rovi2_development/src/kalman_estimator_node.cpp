#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <rovi2_development/Trajectory3D.h>
#include <kalman_estimator_node.hpp>


tf2::Stamped<Eigen::Vector3d> pointstamped_to_vector3d(geometry_msgs::PointStamped &in)
{
    tf2::Stamped<Eigen::Vector3d> out;
    out.stamp_ = in.header.stamp;
    out.x() = in.point.x;
    out.y() = in.point.y;
    out.z() = in.point.z;
    return out;
}

geometry_msgs::Point vector3d_to_point(Eigen::Vector3d &in)
{
    geometry_msgs::Point out;
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
    return out;
}


void Kalman_Estimator::pose_callback( __attribute__((unused)) const geometry_msgs::PointStamped::ConstPtr &msg)
{
  //Propegate the array of last positions
  geometry_msgs::PointStamped this_pt = *msg;
  this->prev_points[2] = std::move(this->prev_points[1]);
  this->prev_points[1] = std::move(this->prev_points[0]);
  this->prev_points[0] = pointstamped_to_vector3d(this_pt);

  Eigen::Vector3d cur_speed = (this->prev_points[1] - this->prev_points[0]) / (this->prev_points[1].stamp_ - this->prev_points[0].stamp_).toSec();
  Eigen::Vector3d last_speed = (this->prev_points[2] - this->prev_points[1]) / (this->prev_points[2].stamp_ - this->prev_points[1].stamp_).toSec();
  Eigen::Vector3d acc = (last_speed - cur_speed) / (this->prev_points[1].stamp_ - this->prev_points[0].stamp_).toSec();;
  Eigen::Vector3d pos = this->prev_points[0];


  //  Do the parameter estimation // TODO
  rovi2_development::Trajectory3D traj;
  traj.header.stamp = this_pt.header.stamp;
  traj.acc = vector3d_to_point(acc);
  traj.vel = vector3d_to_point(cur_speed);
  traj.pos = vector3d_to_point(pos);
  traj.t0 = this_pt.header.stamp;
  this->pub.publish(traj);

}

Kalman_Estimator::Kalman_Estimator()
: sub(nh.subscribe<geometry_msgs::PointStamped>("/pose/3d",1, &Kalman_Estimator::pose_callback, this)),
  pub(nh.advertise<rovi2_development::Trajectory3D>("/pose/parameter",1))
{

}

Kalman_Estimator::~Kalman_Estimator() {}


int main(int argc, char **argv){
  ros::init(argc, argv, "kalman_estimator");
  Kalman_Estimator k_e;
  ros::spin();

  return 0;
}
