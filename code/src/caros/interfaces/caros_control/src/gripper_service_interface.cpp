#include <caros/gripper_service_interface.h>
#include <caros/common.h>
#include <caros/common_robwork.h>

#include <caros_control_msgs/GripperState.h>

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <string>

using namespace caros;

GripperServiceInterface::GripperServiceInterface(const ros::NodeHandle& nodehandle)
    : nodehandle_(nodehandle, GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Do nothing */
}

GripperServiceInterface::GripperServiceInterface()
{
  /* Do nothing */
  ROS_FATAL_STREAM(
      "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

GripperServiceInterface::~GripperServiceInterface()
{
  /* Currently no special things to clean up */
}

bool GripperServiceInterface::configureInterface()
{
  return initService();
}

bool GripperServiceInterface::initService()
{
  if (gripper_state_publisher_ || srv_move_q_ || srv_grip_q_ || srv_set_force_q_ || srv_set_velocity_q_ ||
      srv_stop_movement_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more GripperServiceInterface services or publishers. If this is not fully intended then "
        "this should be considered a bug!");
  }

  gripper_state_publisher_ =
      nodehandle_.advertise<caros_control_msgs::GripperState>("gripper_state", GRIPPER_STATE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!gripper_state_publisher_, "The GripperState publisher is empty!");

  srv_move_q_ = nodehandle_.advertiseService("move_q", &GripperServiceInterface::moveQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_q_, "The move_q service is empty!");

  srv_grip_q_ = nodehandle_.advertiseService("grip_q", &GripperServiceInterface::gripQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_grip_q_, "The grip_q service is empty!");

  srv_set_force_q_ = nodehandle_.advertiseService("set_force_q", &GripperServiceInterface::setForceQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_set_force_q_, "The set_force_q service is empty!");

  srv_set_velocity_q_ =
      nodehandle_.advertiseService("set_velocity_q", &GripperServiceInterface::setVelocityQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_set_velocity_q_, "The set_velocity_q service is empty!");

  srv_stop_movement_ =
      nodehandle_.advertiseService("stop_movement", &GripperServiceInterface::stopMovementHandle, this);
  ROS_ERROR_STREAM_COND(!srv_stop_movement_, "The stop_movement service is empty!");

  /* Verify that the various ROS services have actually been created properly */
  if (gripper_state_publisher_ && srv_move_q_ && srv_grip_q_ && srv_set_force_q_ && srv_set_velocity_q_ &&
      srv_stop_movement_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM("All GripperServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The gripper service could not be properly initialised - one or more ros services or publishers failed to be "
        "properly initialised.");
    return false;
  }

  return true;
}

void GripperServiceInterface::publishState(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& jointforce,
                                           bool is_moving, bool is_blocked, bool is_stopped, bool is_e_stopped)
{
  caros_control_msgs::GripperState state;

  state.header.stamp = ros::Time::now();

  state.q = caros::toRos(q);
  state.dq = caros::toRos(dq);
  state.force = caros::toRos(jointforce);
  state.is_moving = is_moving;
  state.is_blocked = is_blocked;
  state.is_stopped = is_stopped;
  state.e_stopped = is_e_stopped;

  if (gripper_state_publisher_)
  {
    gripper_state_publisher_.publish(state);
  }
  else
  {
    ROS_ERROR_STREAM(
        "The gripper_state_publisher_ is empty - trying to publish gripper state with a non-working "
        "GripperSreviceInterface object.");
  }
}

bool GripperServiceInterface::moveQHandle(caros_control_msgs::GripperMoveQ::Request& request,
                                          caros_control_msgs::GripperMoveQ::Response& response)
{
  response.success = moveQ(caros::toRw(request.q));
  return true;
}

bool GripperServiceInterface::gripQHandle(caros_control_msgs::GripperGripQ::Request& request,
                                          caros_control_msgs::GripperGripQ::Response& response)
{
  response.success = gripQ(caros::toRw(request.q));
  return true;
}

bool GripperServiceInterface::setForceQHandle(caros_control_msgs::GripperSetForceQ::Request& request,
                                              caros_control_msgs::GripperSetForceQ::Response& response)
{
  response.success = setForceQ(caros::toRw(request.force));
  return true;
}

bool GripperServiceInterface::setVelocityQHandle(caros_control_msgs::GripperSetVelocityQ::Request& request,
                                                 caros_control_msgs::GripperSetVelocityQ::Response& response)
{
  response.success = setVelocityQ(caros::toRw(request.velocity));
  return true;
}

bool GripperServiceInterface::stopMovementHandle(caros_control_msgs::GripperStopMovement::Request& request,
                                                 caros_control_msgs::GripperStopMovement::Response& response)
{
  response.success = stopMovement();
  return true;
}
