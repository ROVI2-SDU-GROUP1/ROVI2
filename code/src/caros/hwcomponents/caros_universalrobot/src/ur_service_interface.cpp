#include <caros/ur_service_interface.h>

#include <caros/common.h>
#include <caros/common_robwork.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>

#include <algorithm>

URServiceInterface::URServiceInterface(const ros::NodeHandle& nodehandle) : nodehandle_(nodehandle)
{
  /* Do nothing for now */
  /* No way to verify that this object is properly configured or just a zombie object, since RAII is not being used */
}

URServiceInterface::~URServiceInterface()
{
  /* Currently no special things to clean up */
}

bool URServiceInterface::configureInterface()
{
  if (srv_ur_servo_t_ || srv_ur_servo_q_ || srv_ur_force_mode_start_ || srv_ur_force_mode_update_ ||
      srv_ur_force_mode_stop_ || srv_ur_set_payload_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more URServiceInterface services. If this is not fully intended then this should be "
        "considered a bug!");
  }

  srv_ur_servo_t_ = nodehandle_.advertiseService("servo_t", &URServiceInterface::urServoTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_servo_t_, "The servo_t service is empty!");

  srv_ur_servo_q_ = nodehandle_.advertiseService("servo_q", &URServiceInterface::urServoQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_servo_q_, "The servo_q service is empty!");

  srv_ur_force_mode_start_ =
      nodehandle_.advertiseService("force_mode_start", &URServiceInterface::urForceModeStartHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_force_mode_start_, "The force_mode_start service is empty!");

  srv_ur_force_mode_update_ =
      nodehandle_.advertiseService("force_mode_update", &URServiceInterface::urForceModeUpdateHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_force_mode_update_, "The force_mode_update service is empty!");

  srv_ur_force_mode_stop_ =
      nodehandle_.advertiseService("force_mode_stop", &URServiceInterface::urForceModeStopHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_force_mode_stop_, "The force_mode_stop service is empty!");

  srv_ur_set_payload_ = nodehandle_.advertiseService("set_payload", &URServiceInterface::urSetPayloadHandle, this);
  ROS_ERROR_STREAM_COND(!srv_ur_set_payload_, "The set_payload service is empty!");

  if (srv_ur_servo_t_ && srv_ur_servo_q_ && srv_ur_force_mode_start_ && srv_ur_force_mode_update_ &&
      srv_ur_force_mode_stop_ && srv_ur_set_payload_)
  {
    /* Everything seems to have been properly initialised */
  }
  else
  {
    ROS_ERROR_STREAM(
        "The URService could not be properly initialised - one or more ros services may not be up and running or "
        "working as intended!");
    return false;
  }

  return true;
}

bool URServiceInterface::urServoTHandle(caros_universalrobot::UrServiceServoT::Request& request,
                                        caros_universalrobot::UrServiceServoT::Response& response)
{
  rw::math::Transform3D<> target = caros::toRw(request.target);
  response.success = urServoT(target);
  return true;
}

bool URServiceInterface::urServoQHandle(caros_universalrobot::UrServiceServoQ::Request& request,
                                        caros_universalrobot::UrServiceServoQ::Response& response)
{
  rw::math::Q target = caros::toRw(request.target);
  response.success = urServoQ(target);
  return true;
}

bool URServiceInterface::urForceModeStartHandle(caros_universalrobot::UrServiceForceModeStart::Request& request,
                                                caros_universalrobot::UrServiceForceModeStart::Response& response)
{
  rw::math::Transform3D<> ref_t_offset = caros::toRw(request.base2forceFrame);
  rw::math::Wrench6D<> wrench_target;
  wrench_target(0) = request.wrench.force.x;
  wrench_target(1) = request.wrench.force.y;
  wrench_target(2) = request.wrench.force.z;

  wrench_target(3) = request.wrench.torque.x;
  wrench_target(4) = request.wrench.torque.y;
  wrench_target(5) = request.wrench.torque.z;

  std::size_t index;
  rw::math::Q selection(request.selection.size());
  index = 0;
  for (const auto item : request.selection)
  {
    selection(index++) = static_cast<double>(item);
  }

  rw::math::Q limits(request.limits.size());
  index = 0;
  for (const auto item : request.limits)
  {
    limits(index++) = static_cast<double>(item);
  }

  response.success = urForceModeStart(ref_t_offset, selection, wrench_target, limits);
  return true;
}

bool URServiceInterface::urForceModeUpdateHandle(caros_universalrobot::UrServiceForceModeUpdate::Request& request,
                                                 caros_universalrobot::UrServiceForceModeUpdate::Response& response)
{
  rw::math::Wrench6D<> wrench_target;
  wrench_target(0) = request.wrench.force.x;
  wrench_target(1) = request.wrench.force.y;
  wrench_target(2) = request.wrench.force.z;

  wrench_target(3) = request.wrench.torque.x;
  wrench_target(4) = request.wrench.torque.y;
  wrench_target(5) = request.wrench.torque.z;

  response.success = urForceModeUpdate(wrench_target);
  return true;
}

bool URServiceInterface::urForceModeStopHandle(caros_universalrobot::UrServiceForceModeStop::Request& request,
                                               caros_universalrobot::UrServiceForceModeStop::Response& response)
{
  response.success = urForceModeStop();
  return true;
}

bool URServiceInterface::urSetPayloadHandle(caros_universalrobot::UrServicePayload::Request& request,
                                            caros_universalrobot::UrServicePayload::Response& response)
{
  double mass = request.mass;
  rw::math::Vector3D<> center_of_mass;
  center_of_mass[0] = request.com.at(0);
  center_of_mass[1] = request.com.at(1);
  center_of_mass[2] = request.com.at(2);

  response.success = urSetPayload(mass, center_of_mass);
  return true;
}
