#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <rovi2_development/Trajectory3D.h>
#include <kalman_estimator_node.hpp>
#include <chrono>
#include <std_msgs/Bool.h>

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

void Kalman_Estimator::do_predict_step()
{
    //Perform a state estimate
    this->cur_state = this->transition_matrix * this->cur_state;
    //Perform a covariance estimate
    this->cur_covariance = this->transition_matrix * this->cur_covariance * this->transition_matrix.transpose();
}

void Kalman_Estimator::do_update_step(Eigen::VectorXd measured_state)
{
    //Get difference between measurement and estimate
    Eigen::VectorXd y_k = measured_state - this->measurement_matrix * this->cur_state;

    //"innovation" covariance step.
    //Maybe we could get an estimate of R_k from the vision stuff by somehow evaluating how good our detection
    //Of the balls position was last time. Just pick 0.1 in the diagonal for now. We could also try to use ALS??
    Eigen::Matrix<double, 9, 9> R_k = Eigen::MatrixXd::Identity(this->transition_matrix.rows(), this->transition_matrix.cols());
    R_k(0,0) = 10; //Ok sure about the actual points.
    R_k(1,1) = 10; //Ok sure about the actual points.
    R_k(2,2) = 10; //Ok sure about the actual points.
    R_k(3,3) = 1000; //Not too sure about velocity.
    R_k(4,4) = 1000; //Not too sure about velocity.
    R_k(5,5) = 1000; //Not too sure about velocity.
    R_k(6,6) = 10000; //Very unsure about acceleration.
    R_k(7,7) = 10000; //Very unsure about acceleration.
    R_k(8,8) = 10000; //Very unsure about acceleration.

    Eigen::Matrix<double, 9, 9> S_k = this->measurement_matrix * this->cur_covariance * this->measurement_matrix.transpose() + R_k;
    //Compute the optimal kalman gain from the different covariance matrix'


    Eigen::MatrixXd K_gain = this->cur_covariance * this->measurement_matrix.transpose() * S_k.inverse();
    //Update state estimate.
    //std::cout << K_gain << " ... \n" << y_k << std::endl;

    this->cur_state = this->cur_state + K_gain * y_k;
    this->cur_covariance = (Eigen::MatrixXd::Identity(this->transition_matrix.rows(), this->transition_matrix.cols()) - K_gain * this->measurement_matrix) *
        this->cur_covariance;
}

void Kalman_Estimator::update_transition_matrix(double time_step)
{
    //at2 means 0.5 * dt * dt
    //The matrix will be:
    //[1,   0,   0,  dt,   0,   0, at2,   0,   0]
    //[0,   1,   0,   0,  dt,   0,   0, at2,   0]
    //[0,   0,   1,   0,   0,  dt,   0,   0, at2]
    //[0,   0,   0,   1,   0,   0,  dt,   0,   0]
    //[0,   0,   0,   0,   1,   0,   0,  dt,   0]
    //[0,   0,   0,   0,   0,   1,   0,   0,  dt]
    //[0,   0,   0,   0,   0,   0,   1,   0,   0]
    //[0,   0,   0,   0,   0,   0,   0,   1,   0]
    //[0,   0,   0,   0,   0,   0,   0,   0,   1]

    //Set the dt fields
    uint8_t y = 0;
    for(uint8_t x = 3; x <  this->transition_matrix.cols(); x++, y++)
    {
        this->transition_matrix(y, x) = time_step;
    }
    //Set the at2 fields
    y = 0;
    for(uint8_t x = 6; x <  this->transition_matrix.cols(); x++, y++)
    {
        this->transition_matrix(y, x) = 0.5 * time_step * time_step ;
    }

}

bool Kalman_Estimator::test_reset(Eigen::Vector3d &cur_speed, Eigen::Vector3d &acc)
{   //This function tries to guestimatte if it is time for resetting the filter state.
    if(this->prev_points[0].stamp_ < this->prev_points[1].stamp_)
    {   //Reset if we have performed timetravel
        return true;
    }
    if((this->prev_points[0].stamp_ - this->prev_points[1].stamp_).toSec() > 1)
    {   //If there have been more than 1 second between the detections, we reset.
        return true;
    }
    if((this->prev_points[0] -  this->prev_points[1]).norm() > 1.5)
    {   //If the ball have moved more than 1.5 meter inbetween frames, give up
        return true;
    }
    if(acc.norm() > 50) //If we have to violent acceration, reset
    {   //If the ball have moved more than 1.5 meter inbetween frames, give up
        return true;
    }
    return false;
}
void Kalman_Estimator::pose_callback( __attribute__((unused)) const geometry_msgs::PointStamped::ConstPtr &msg)
{
  this->position_count++;
  //Propegate the array of last positions
  geometry_msgs::PointStamped this_pt = *msg;
  this->prev_points[2] = std::move(this->prev_points[1]);
  this->prev_points[1] = std::move(this->prev_points[0]);
  this->prev_points[0] = pointstamped_to_vector3d(this_pt);
  this->prev_points[0](1) += 0.5;
  if(position_count < 3) return; //We don't have enough positions yet to estimate the parameters.
                                 //We could probably do a fallback after two samples to some default acceleration parameters

  //Compute speeds and accelerations
  Eigen::Vector3d cur_speed = (this->prev_points[0] - this->prev_points[1]) / (this->prev_points[0].stamp_ - this->prev_points[1].stamp_).toSec();
  Eigen::Vector3d last_speed = (this->prev_points[1] - this->prev_points[2]) / (this->prev_points[1].stamp_ - this->prev_points[2].stamp_).toSec();
  Eigen::Vector3d acc = (cur_speed - last_speed) / (this->prev_points[0].stamp_ - this->prev_points[1].stamp_).toSec();;
  Eigen::Vector3d pos = this->prev_points[0];
  Eigen::VectorXd measured_state(9);
  std::cout << "speed " << cur_speed.norm() << "\tmoved distance " <<  (this->prev_points[0] -  this->prev_points[1]).norm()  << "total acceleration " << acc.norm() <<  std::endl;
  std::cout << acc << std::endl;
  if(this->test_reset(cur_speed, acc) /*and false*/)
  {
      this->position_count = 1;
      std_msgs::Bool reset_msg;
      reset_msg.data = true;
      this->pub_reset.publish(reset_msg);
      this->reset();
      std::cout << "kalman state have been reset!" << std::endl;
      return;
  }
  measured_state(0) = pos(0);
  measured_state(1) = pos(1);
  measured_state(2) = pos(2);
  measured_state(3) = cur_speed(0);
  measured_state(4) = cur_speed(1);
  measured_state(5) = cur_speed(2);
  measured_state(6) = acc(0);
  measured_state(7) = acc(1);
  measured_state(8) = acc(2);

  //measured_state(6) = 0;
  //measured_state(7) = 0;
  //measured_state(8) = -9.82;
  rovi2_development::Trajectory3D traj;
  traj.header.stamp = this_pt.header.stamp;
  traj.t0 = this_pt.header.stamp;
  traj.acc = vector3d_to_point(acc);
  traj.vel = vector3d_to_point(cur_speed);
  traj.pos = vector3d_to_point(pos);
  this->pub_raw.publish(traj); //Publish nonfiltered parameters


  this->update_transition_matrix((this->prev_points[0].stamp_ - this->prev_points[1].stamp_).toSec());
  this->do_predict_step();
  this->do_update_step(measured_state);

  //Set acc, pos and cur_speed to the new found values
  pos(0)        = this->cur_state(0);
  pos(1)        = this->cur_state(1);
  pos(2)        = this->cur_state(2);
  cur_speed(0)  = this->cur_state(3);
  cur_speed(1)  = this->cur_state(4);
  cur_speed(2)  = this->cur_state(5);
  acc(0)        = this->cur_state(6);
  acc(1)        = this->cur_state(7);
  acc(2)        = this->cur_state(8);
  traj.acc = vector3d_to_point(acc);
  traj.vel = vector3d_to_point(cur_speed);
  traj.pos = vector3d_to_point(pos);
  this->pub_filtered.publish(traj); //publish filtered parameters.
}

Kalman_Estimator::Kalman_Estimator()
: sub(nh.subscribe<geometry_msgs::PointStamped>("/pose/3d",5, &Kalman_Estimator::pose_callback, this)),
  pub_filtered(nh.advertise<rovi2_development::Trajectory3D>("/pose/parameter",5)),
  pub_raw(nh.advertise<rovi2_development::Trajectory3D>("/pose/parameter_raw",5)),
  pub_reset(nh.advertise<std_msgs::Bool>("kalman/reset", 5)),

  cur_state(9)
{
    this->reset();
}

void Kalman_Estimator::reset()
{
    //Initialise current state as 0 for all.
    this->cur_state.setZero();
    this->cur_state(6) = 0;
    this->cur_state(7) = 0;
    this->cur_state(8) = -9.82;

    //Initialise co-variance matrix.
    this->cur_covariance.setZero();

    //We don't know where the ball is at all, so initialise extremly high)
    for(uint8_t i = 0; i < this->cur_covariance.rows(); i++)
        this->cur_covariance(i, i) = 999999999999999.;

    //Set the measurement matrix. This is easy, it's just identity.
    this->measurement_matrix = Eigen::MatrixXd::Identity(this->measurement_matrix.rows(), this->measurement_matrix.cols());

    //Now the transistion matrix. It's more complex.
    //Here we just set it as identity. The other parameters are dynamic, so we have to change them each time anyway.
    //See update_transistion_matrix for details.
    this->transition_matrix = Eigen::MatrixXd::Identity(this->transition_matrix.rows(), this->transition_matrix.cols());

}
Kalman_Estimator::~Kalman_Estimator() {}


int main(int argc, char **argv){
  ros::init(argc, argv, "kalman_estimator");
  Kalman_Estimator k_e;
  ros::spin();

  return 0;
}
