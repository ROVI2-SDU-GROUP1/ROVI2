#include <caros/serial_device_service_interface.h>
#include <caros/common.h>
#include <caros/common_robwork.h>

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <vector>
#include <tuple>

namespace
{
template <typename targets_t, typename speeds_t, typename container_t>
bool fillContainerWithTargetsAndSpeeds(const targets_t& targets, const speeds_t& speeds, container_t& res)
{
  if (targets.size() != speeds.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size()
                                 << " speeds, but there should be the same amount of each!");
    return false;
  }

  /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
  ROS_ASSERT(targets.size() == speeds.size());
  res.clear();
  res.reserve(targets.size());

  /* TODO:
   * Perform the for-loop within a try-catch block to catch out-of-range access within the .at() call
   */
  for (typename targets_t::size_type index = 0; index < targets.size(); ++index)
  {
    res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index)));
  }

  return true;
}
}  // end namespace

using namespace caros;

SerialDeviceServiceInterface::SerialDeviceServiceInterface(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle, SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Do nothing */
}

SerialDeviceServiceInterface::SerialDeviceServiceInterface()
{
  /* Do nothing */
  ROS_FATAL_STREAM(
      "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

SerialDeviceServiceInterface::~SerialDeviceServiceInterface()
{
  /* Nothing special needs to be done - relying on ROS's RAII design */
}

bool SerialDeviceServiceInterface::configureInterface()
{
  return initService();
}

bool SerialDeviceServiceInterface::initService()
{
  if (srv_move_lin_ || srv_move_ptp_ || srv_move_ptp_t_ || srv_move_vel_q_ || srv_move_vel_t_ || srv_move_servo_q_ ||
      srv_move_servo_t_ || srv_move_stop_ || device_state_publisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more SerialDeviceServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  device_state_publisher_ =
      nodehandle_.advertise<caros_control_msgs::RobotState>("robot_state", SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!device_state_publisher_, "The RobotState publisher is empty!");

  srv_move_lin_ = nodehandle_.advertiseService("move_lin", &SerialDeviceServiceInterface::moveLinHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_lin_, "The move_lin service is empty!");

  srv_move_ptp_ = nodehandle_.advertiseService("move_ptp", &SerialDeviceServiceInterface::movePtpHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_ptp_, "The move_ptp service is empty!");

  srv_move_ptp_t_ = nodehandle_.advertiseService("move_ptp_t", &SerialDeviceServiceInterface::movePtpTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_ptp_t_, "The move_ptp_t service is empty!");

  srv_move_vel_q_ = nodehandle_.advertiseService("move_vel_q", &SerialDeviceServiceInterface::moveVelQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_vel_q_, "The move_vel_q service is empty!");

  srv_move_vel_t_ = nodehandle_.advertiseService("move_vel_t", &SerialDeviceServiceInterface::moveVelTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_vel_t_, "The move_vel_t service is empty!");

  srv_move_servo_q_ =
      nodehandle_.advertiseService("move_servo_q", &SerialDeviceServiceInterface::moveServoQHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_servo_q_, "The move_servo_q service is empty!");

  srv_move_servo_t_ =
      nodehandle_.advertiseService("move_servo_t", &SerialDeviceServiceInterface::moveServoTHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_servo_t_, "The move_servo_t service is empty!");

  srv_move_stop_ = nodehandle_.advertiseService("move_stop", &SerialDeviceServiceInterface::moveStopHandle, this);
  ROS_ERROR_STREAM_COND(!srv_move_stop_, "The move_stop service is empty!");

  if (srv_move_lin_ && srv_move_ptp_ && srv_move_ptp_t_ && srv_move_vel_q_ && srv_move_vel_t_ && srv_move_servo_q_ &&
      srv_move_servo_t_ && srv_move_stop_ && device_state_publisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM(
        "All SerialDeviceServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The SerialDeviceServiceInterface could not be properly initialised - one or more ROS services or publishers "
        "failed to be properly initialised.");
    return false;
  }

  return true;
}

void SerialDeviceServiceInterface::publishState(const caros_control_msgs::RobotState& state)
{
  device_state_publisher_.publish(state);
}

/************************************************************************
 * ROS service handle functions
 ************************************************************************/
/* TODO:
 * Rewrite the functions to also take in the blends (which should be added to the .srv files). Also remember to update
 * the non-handle versions.
 */
bool SerialDeviceServiceInterface::moveLinHandle(caros_control_msgs::SerialDeviceMoveLin::Request& request,
                                                 caros_control_msgs::SerialDeviceMoveLin::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveLin(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpHandle(caros_control_msgs::SerialDeviceMovePtp::Request& request,
                                                 caros_control_msgs::SerialDeviceMovePtp::Response& response)
{
  QAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = movePtp(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpTHandle(caros_control_msgs::SerialDeviceMovePtpT::Request& request,
                                                  caros_control_msgs::SerialDeviceMovePtpT::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = movePtpT(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveVelQHandle(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelQ::Response& response)
{
  rw::math::Q vel = caros::toRw(request.vel);
  response.success = moveVelQ(vel);

  return true;
}

bool SerialDeviceServiceInterface::moveVelTHandle(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelT::Response& response)
{
  rw::math::VelocityScrew6D<> vel = caros::toRw(request.vel);
  response.success = moveVelT(vel);

  return true;
}

/* TODO:
 * Add speeds implementation
 * This should also be added to the RobWorkHardware URCallBackInterface servo functionality, where the speed should be
 * optional, defaulting to the currently hardcoded value...
 */
bool SerialDeviceServiceInterface::moveServoQHandle(caros_control_msgs::SerialDeviceMoveServoQ::Request& request,
                                                    caros_control_msgs::SerialDeviceMoveServoQ::Response& response)
{
  QAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveServoQ(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveServoTHandle(caros_control_msgs::SerialDeviceMoveServoT::Request& request,
                                                    caros_control_msgs::SerialDeviceMoveServoT::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveServoT(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveStopHandle(caros_common_msgs::EmptySrv::Request& request,
                                                  caros_common_msgs::EmptySrv::Response& response)
{
  response.success = moveStop();

  return true;
}
