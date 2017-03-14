#include <caros/universal_robots.h>

#include <caros/common.h>
#include <caros/common_robwork.h>

#include <rw/math/MetricFactory.hpp>

#include <ros/assert.h>

#include <string>
#include <vector>

#define URRT_PORT 30003
#define UR_PORT 30001
#define URRTDATA_QACTUAL_SIZE 6

namespace caros
{
UniversalRobots::UniversalRobots(const ros::NodeHandle& nodehandle)
    : CarosNodeServiceInterface(nodehandle),
      SerialDeviceServiceInterface(nodehandle),
      URServiceInterface(nodehandle),
      nodehandle_(nodehandle),
      workcell_(NULL),
      device_(NULL)
{
  /* Currently nothing specific should happen */
}

UniversalRobots::~UniversalRobots()
{
  /* Nothing special to destroy */
}

bool UniversalRobots::activateHook()
{
  /* TODO:
   * - Maybe split all this configuration/activation logic up into a configure and connect step, like what has been done
   * for the SDH node.
   * ^- Should the fetching of the parameters be moved into its own function, instead of having this big activateHook()
   * function?
   */
  /************************************************************************
   * Parameters
   ************************************************************************/
  std::string device_name;
  if (!nodehandle_.getParam("device_name", device_name))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace()
                                       << "/device_name' was not present on the parameter "
                                          "server! This parameter has to be specified "
                                          "for this node to work properly.",
                     URNODE_MISSING_PARAMETER);

    return false;
  }

  std::string device_ip;
  if (!nodehandle_.getParam("device_ip", device_ip))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace()
                                       << "/device_ip' was not present on the parameter server! "
                                          "This parameter has to be specified for this "
                                          "node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  std::string callback_ip;
  if (!nodehandle_.getParam("callback_ip", callback_ip))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace()
                                       << "/callback_ip' was not present on the parameter "
                                          "server! This parameter has to be specified "
                                          "for this node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  std::string callback_port;
  if (!nodehandle_.getParam("callback_port", callback_port))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/callback_port' was not present on the "
                                                                        "parameter server! This parameter has to be "
                                                                        "specified for this node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  workcell_ = caros::getWorkCell();
  if (workcell_ == NULL)
  {
    CAROS_FATALERROR("No workcell was provided!", URNODE_MISSING_WORKCELL);
    return false;
  }

  ROS_ASSERT(workcell_ != NULL);
  ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
  device_ = workcell_->findDevice(device_name);
  if (device_ == NULL)
  {
    CAROS_FATALERROR("Unable to find device " << device_name << " in the loaded workcell",
                     URNODE_NO_SUCH_DEVICE_IN_WORKCELL);
    return false;
  }

  state_ = workcell_->getDefaultState();
  ik_solver_ = rw::common::ownedPtr(new rw::invkin::JacobianIKSolver(device_, state_));
  ROS_ASSERT(ik_solver_ != NULL);

  try
  {
    urrt_.connect(device_ip, URRT_PORT);
  }
  catch (rw::common::Exception& exp)
  {
    CAROS_FATALERROR("Could not connect to urrt:" << exp.what(), URNODE_UNSUCCESSFUL_CONNECT_TO_URRT);
    return false;
  }

  try
  {
    ur_.connect(device_ip, UR_PORT);
  }
  catch (rw::common::Exception& exp)
  {
    CAROS_FATALERROR("Could not connect to urrt:" << exp.what(), URNODE_UNSUCCESSFUL_CONNECT_TO_UR);
    return false;
  }

  unsigned int numeric_callback_port = 0;
  try
  {
    numeric_callback_port = std::stoul(callback_port);
  }
  catch (std::exception& e)
  {
    CAROS_FATALERROR("The supplied callback port could not be converted to a numeric value.",
                     URNODE_INVALID_CALLBACKPORT);
    return false;
  }

  /* The order of starting ur_ and urrt_ doesn't seem to matter */
  /* No feedback from startCommunication() ... there are debug log messages on whether it was sort of successful or not
   */
  ur_.startCommunication(callback_ip, numeric_callback_port);
  /* No feedback from start() */
  urrt_.start();

  if (!URServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The URService could not be configured correctly.", URNODE_URSERVICE_CONFIGURE_FAIL);
    return false;
  }

  if (!SerialDeviceServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The SerialDeviceService could not be configured correctly.",
                     URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool UniversalRobots::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* TODO: Missing handling all the different scenarios. */
  /* TODO: Go into fatal error if recover is invoked on the unsupported error scenarios */
  bool resolved = false;

  switch (error_code)
  {
    case URNODE_UNSUPPORTED_Q_LENGTH:
      /* Simply acknowledge that a wrong Q was provided */
      resolved = true;
      break;
    case URNODE_INTERNAL_ERROR:
      CAROS_FATALERROR("Can not resolve an internal error... ending up in this case/situation is a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
    default:
      CAROS_FATALERROR("The provided error code '"
                           << error_code << "' has no recovery functionality! - this should be considered a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
  }

  return resolved;
}

void UniversalRobots::runLoopHook()
{
  bool urrt_has_data = urrt_.hasData();
  bool ur_has_data = ur_.getPrimaryInterface().hasData();

  if (urrt_has_data == false || ur_has_data == false)
  {
    ROS_DEBUG_STREAM("Missing some data from UR: urrt_has_data = '" << urrt_has_data << "' and ur_has_data = '"
                                                                    << ur_has_data << "'");
    return;
  }

  rwhw::URRTData urrt_data = urrt_.getLastData();
  rwhw::UniversalRobotsData pur_data = ur_.getPrimaryInterface().getLastData();

  ROS_DEBUG_STREAM("E-stop: '" << pur_data.emergencyStopped << "' Security-stop: '" << pur_data.securityStopped
                               << "' Running: '" << pur_data.programRunning << "'");

  auto messages = ur_.getPrimaryInterface().getMessages();
  while (!messages.empty())
  {
    ROS_DEBUG_STREAM("UR Message: " << messages.front());
    messages.pop();
  }
  ur_.getPrimaryInterface().clearMessages();

  if (urrt_data.qActual.size() == URRTDATA_QACTUAL_SIZE)
  {
    caros_control_msgs::RobotState robot_state;
    qcurrent_ = urrt_data.qActual;
    robot_state.q = caros::toRos(urrt_data.qActual);
    robot_state.dq = caros::toRos(urrt_data.dqActual);
    robot_state.header.frame_id = nodehandle_.getNamespace();
    robot_state.header.stamp = ros::Time::now();
    robot_state.e_stopped = caros::toRos(pur_data.emergencyStopped);

    /* TODO: Currently there is a delay somewhere, where the data gotten from the robot is really delayed quite a bit -
     * atleast for the emergency stop. */

    /* TODO: This isMoving() function is not working - returns false eventhough the robot is moving...
     * isMoving() is returning a variable that is modified within a thread - Perhaps the compiler has optimised the read
     * to be constant? */
    robot_state.is_moving = caros::toRos(ur_.isMoving());

    /* TODO:
     * isColliding is hardcoded to be false....
     * ^- Look at the old out-commented code in the MARVIN version for hints on how the collision detection was done (if
     * it's desirable to bring back that functionality)
     */
    robot_state.is_colliding = caros::toRos(false);

    SerialDeviceServiceInterface::publishState(robot_state);
  }
  else
  {
    /* TODO:
     * Should this be an error (just ROS error or also a CAROS error - e.g. changing the state of this node to error or
     * fatalerror)?
     */
    ROS_WARN_STREAM("The size of urrt_data.qActual is '" << urrt_data.qActual.size() << "' but should be '"
                                                         << URRTDATA_QACTUAL_SIZE << "'!"
                                                         << " - Not publishing robot state!");
  }
}

void UniversalRobots::errorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

void UniversalRobots::fatalErrorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

/************************************************************************
 * URServiceInterface functions
 ************************************************************************/
bool UniversalRobots::urServoT(const rw::math::Transform3D<>& target)
{
  ROS_DEBUG_STREAM("servoT: " << target);

  if (!isInWorkingCondition())
  {
    return false;
  }

  device_->setQ(qcurrent_, state_);
  std::vector<rw::math::Q> solutions = ik_solver_->solve(target, state_);
  if (solutions.empty())
  {
    ROS_ERROR_STREAM("servoT: Unable to find IK solution for target = '" << target << "' and qcurrent_ = '" << qcurrent_
                                                                         << "'");
    return false;
  }

  rw::math::Q closest = solutions.front();

  if (!supportedQSize(closest))
  {
    return false;
  }

  ROS_DEBUG_STREAM("servoT: Q-configuration being sent to driver: " << closest);
  ur_.servo(closest);
  /* There is no (immediate) feedback from the ur_.servo() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urServoQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM("ServoQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  ur_.servo(q);
  /* There is no (immediate) feedback from the ur_.servo() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urForceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection,
                                       const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& limits)
{
  ROS_DEBUG_STREAM("ForceModeStart arguments begin:");
  ROS_DEBUG_STREAM("refToffset: " << refToffset);
  ROS_DEBUG_STREAM("selection: " << selection);
  ROS_DEBUG_STREAM("wrench_target: " << wrench_target);
  ROS_DEBUG_STREAM("limits: " << limits);
  ROS_DEBUG_STREAM("ForceModeStart arguments end");

  if (!isInWorkingCondition())
  {
    return false;
  }

  if (selection.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in selection is '" << selection.size() << "' but should be '" << 6 << "'");
    return false;
  }
  if (limits.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in limits is '" << limits.size() << "' but should be '" << 6 << "'");
    return false;
  }

  ur_.forceModeStart(refToffset, selection, wrench_target, limits);
  /* There is no (immediate) feedback from the ur_.forceModeStart() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target)
{
  ROS_DEBUG_STREAM("New wrench target for forceModeUpdate: " << wrench_target);

  if (!isInWorkingCondition())
  {
    return false;
  }

  ur_.forceModeUpdate(wrench_target);
  return true;
}

bool UniversalRobots::urForceModeStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  ur_.forceModeEnd();

  return true;
}

bool UniversalRobots::urSetPayload(const double& mass, const rw::math::Vector3D<>& com)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  ur_.setPayload(mass, com);

  return true;
}

/************************************************************************
 * SerialDeviceServiceInterface
 ************************************************************************/
bool UniversalRobots::moveLin(const TransformAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("moveLin with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    /* moveT(...) is void, so no errorcode is returned. Furthermore the implementation (at least in revision 5472)
     * doesn't make use of the speeds. */
    ur_.moveT(std::get<0>(target), std::get<1>(target));
    /* This could be rewritten to be more explicit about the parameters, such that std::get<1> is the speed */
  }

  /* Simply return true, given there is no feedback from the ur_.moveT(...) function */
  return true;
}

bool UniversalRobots::movePtp(const QAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtp with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    const auto& q = std::get<0>(target);
    if (!supportedQSize(q))
    {
      return false;
    }

    ur_.moveQ(q, std::get<1>(target));
  }

  return true;
}

bool UniversalRobots::movePtpT(const TransformAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtpT with " << targets.size() << " target(s).");

  if (!isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    device_->setQ(qcurrent_, state_);
    rw::math::Transform3D<> transform = std::get<0>(target);
    std::vector<rw::math::Q> solutions = ik_solver_->solve(transform, state_);
    if (solutions.empty())
    {
      ROS_WARN_STREAM("movePtpT: Unable to find IK solution for: " << transform << " with qcurrent: " << qcurrent_);
      return false;
    }

    const rw::math::Q& q = solutions.front();
    if (!supportedQSize(q))
    {
      return false;
    }

    ur_.moveQ(q, std::get<1>(target));
  }

  return true;
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelQ(const rw::math::Q& q_vel)
{
  ROS_ERROR_STREAM("Current implementation does not follow the specification!");
  return false;

/******************************
 * Old Implementation
 ******************************/
#if 0
  if (!isInWorkingCondition() || !supportedQSize(q_vel))
  {
    return false;
  }

  /* TODO:
   * Missing documentation on why the factor 0.1 is used and not some other arbitrary value?
   * And 1/10th of the value is added directly to the current joint values/angles, making a q-value of 0-100 (%) up to
   * 10 radians, which is quite a lot - I doubt that this was the intension when it got implemented in MARVIN...
   */
  return urServoQ(qcurrent_ + q_vel * 0.1);
#endif
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  ROS_ERROR_STREAM("Current implementation has not been verified to follow the specification!");
  return false;

/******************************
 * Old Implementation
 ******************************/
#if 0
  if (!isInWorkingCondition())
  {
    return false;
  }

  device_->setQ(qcurrent_, state_);
  rw::math::Jacobian jac = device_->baseJend(state_);
  /* TODO:
   * Find out where the 'Eigen' got introduced from, since it doesn't exist...
   *	rw::math::Jacobian jac_inv( rw::math::LinearAlgebra::pseudoInverseEigen(jac.e()) );
   */
  rw::math::Jacobian jac_inv(rw::math::LinearAlgebra::pseudoInverse(jac.e()));

  /* TODO:
   * Could use some more documentation on why the factor of 0.1 is being used (see todo comment for moveVelQ)
   */
  rw::math::Q qtarget = qcurrent_ + (jac_inv * t_vel) * 0.1;
  return urServoQ(qtarget);
#endif
}

bool UniversalRobots::moveServoQ(const QAndSpeedContainer_t& targets)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  /* Throwing away the speed:
   * RobWorkHardware doesn't support specifying the speed when servoing
   */
  bool res = false;
  for (const auto& target : targets)
  {
    const auto& q = std::get<0>(target);
    if (!supportedQSize(q))
    {
      return false;
    }

    res = urServoQ(q);
    if (!res)
    {
      break;
    }
  }

  return res;
}

bool UniversalRobots::moveServoT(const TransformAndSpeedContainer_t& targets)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  /* Throwing away the speed */
  for (const auto& target : targets)
  {
    res = urServoT(std::get<0>(target));
    if (!res)
    {
      break;
    }
  }

  return res;
}

bool UniversalRobots::moveStop()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  ur_.stopRobot();

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool UniversalRobots::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  return true;
}

bool UniversalRobots::supportedQSize(const rw::math::Q& q)
{
  if (q.size() != SUPPORTED_Q_LENGTH_FOR_UR)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_UR,
                URNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  return true;
}

}  // namespace caros
