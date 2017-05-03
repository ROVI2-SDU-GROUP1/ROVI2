#include <caros/robotiq_node.h>

#include <rw/rw.hpp>
#include <rwhw/robotiq/Robotiq3.hpp>
#include <rwhw/robotiq/Robotiq2.hpp>

#include <utility>
#include <string>

namespace caros
{
RobotiqNode::RobotiqNode(const ros::NodeHandle& node_handle, const HandType hand_type)
    : caros::CarosNodeServiceInterface(node_handle),
      caros::GripperServiceInterface(node_handle),
      robotiq_(NULL),
      node_handle_(node_handle),
      hand_type_(hand_type)

{
  switch (hand_type_)
  {
    case HandType::ROBOTIQ2:
      last_q_ = rw::math::Q(1, 0.0);
      break;
    case HandType::ROBOTIQ3:
      last_q_ = rw::math::Q(4, 0.0, 0.0, 0.0, 0.0);
      break;
    default:
      /* No supported initialisation for the chosen HandType */
      throw std::invalid_argument("The chosen HandType is not supported - this is a bug!");
      break;
  }
}

RobotiqNode::~RobotiqNode()
{
  if (robotiq_ != NULL)
  {
    if (robotiq_->isConnected())
    {
      ROS_DEBUG_STREAM("Still connected to the Robotiq device - going to stop the device and disconnect.");
      robotiq_->disconnect();
    }
    robotiq_ = NULL;
  }
  else
  {
    ROS_DEBUG_STREAM("There was no Robotiq device to destroy before deallocating/destroying the RobotiqNode object.");
  }
}

bool RobotiqNode::activateHook()
{
  if (!configureRobotiqDevice())
  {
    return false;
  }

  if (!connectToRobotiqDevice())
  {
    return false;
  }

  return true;
}

bool RobotiqNode::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* TODO(any): */

  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented yet!");
  ROS_BREAK();

  return false;
}

void RobotiqNode::runLoopHook()
{
  static bool first_invocation = true;

  try
  {
    if (robotiq_ == 0)
    {
      CAROS_FATALERROR("The Robotiq device is not configured", ROBOTIQNODE_INTERNAL_ERROR);
      return;
    }

    if (!robotiq_->isConnected())
    {
      CAROS_ERROR("There is no established connection to the Robotiq device.",
                  ROBOTIQNODE_ROBOTIQ_DEVICE_NO_CONNECTION);
      return;
    }

    /************************************************************************
     * Get the time since last time this function was run.
     ************************************************************************/
    ros::Time now = ros::Time::now();
    ros::Duration diff = now - last_loop_time_;

    /************************************************************************
     * Get current gripper state and split values
     ************************************************************************/
    robotiq_->getAllStatusCMD();
    rw::math::Q q = robotiq_->getQ();
    rw::math::Q dq_calc = (q - last_q_) / diff.toSec();
    rw::math::Q force = robotiq_->getQCurrent();
    bool is_moving = robotiq_->isGripperMoving();
    bool is_blocked = robotiq_->isGripperBlocked();
    bool is_stopped = !robotiq_->isGripperMoving() && !robotiq_->isGripperBlocked();
    /* FIXME: hardcoded isEstop value */
    bool is_emergency_stopped = false;
    /* Don't publish wrong information when first invoked, because the last_q_ and last_loop_time_ are initialised to 0
     */
    if (!first_invocation)
    {
      publishState(q, dq_calc, force, is_moving, is_blocked, is_stopped, is_emergency_stopped);
    }
    else
    {
      first_invocation = false;
    }

    last_q_ = q;
    last_loop_time_ = now;
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQNODE_INTERNAL_ERROR);
    return;
  }
}

void RobotiqNode::errorLoopHook()
{
  /* Stop the Robotiq's current action(s) */
  if (robotiq_ == 0)
  {
    ROS_DEBUG_STREAM("The Robotiq device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    robotiq_->stopCmd();
    robotiq_->disconnect();
  }
}

void RobotiqNode::fatalErrorLoopHook()
{
  /* Stop the Robotiq's current action(s) */
  if (robotiq_ == 0)
  {
    ROS_DEBUG_STREAM("The Robotiq device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    robotiq_->stopCmd();
    robotiq_->disconnect();
  }
}

bool RobotiqNode::configureRobotiqDevice()
{
  if (robotiq_ != 0)
  {
    /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
    CAROS_FATALERROR(
        "The Robotiq device is already active - trying to configure an already configured Robotiq node is a bug!",
        ROBOTIQNODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE);
    return false;
  }

  /* Fetch parameters (if any) or use the defaults */
  std::string port_param_name = "device_port";
  ROS_DEBUG_STREAM_COND(!node_handle_.hasParam(port_param_name),
                        "Parameter " << port_param_name << " not found from param server. Using default.");
  node_handle_.param(port_param_name, port_, 502);

  std::string ip_param_name = "device_ip";
  ROS_DEBUG_STREAM_COND(!node_handle_.hasParam(ip_param_name),
                        "Parameter " << ip_param_name << " not found from param server. Using default.");

  switch (hand_type_)
  {
    case HandType::ROBOTIQ2:
      node_handle_.param(ip_param_name, ip_, std::string("192.168.100.22"));
      robotiq_ = rw::common::ownedPtr(new rwhw::Robotiq2());
      break;
    case HandType::ROBOTIQ3:
      node_handle_.param(ip_param_name, ip_, std::string("192.168.100.21"));
      robotiq_ = rw::common::ownedPtr(new rwhw::Robotiq3());
      break;
  }
  // TODO(any): Verify that the chosen parameters are valid?

  if (!GripperServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS GripperServiceInterface could not be configured correctly.",
                     ROBOTIQNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  /* Outputting information on supported value ranges */
  typedef std::pair<rw::math::Q, rw::math::Q> PairQ;
  PairQ position_limits = robotiq_->getLimitPos();
  PairQ velocity_limits = robotiq_->getLimitVel();
  PairQ force_limits = robotiq_->getLimitForce();

  ROS_ERROR_STREAM_COND(position_limits.first.size() != position_limits.second.size(),
                        "The sizes of the Q's in the position limit pair are not equal. first contains "
                            << position_limits.first.size() << " and second contains " << position_limits.second.size()
                            << " elements.");
  ROS_ERROR_STREAM_COND(velocity_limits.first.size() != velocity_limits.second.size(),
                        "The sizes of the Q's in the velocity limit pair are not equal. first contains "
                            << velocity_limits.first.size() << " and second contains " << velocity_limits.second.size()
                            << " elements.");
  ROS_ERROR_STREAM_COND(force_limits.first.size() != force_limits.second.size(),
                        "The sizes of the Q's in the force limit pair are not equal. first contains "
                            << force_limits.first.size() << " and second contains " << force_limits.second.size()
                            << " elements.");
  ROS_DEBUG_STREAM("Lower position limits: " << position_limits.first);
  ROS_DEBUG_STREAM("Upper position limits: " << position_limits.second);
  ROS_DEBUG_STREAM("Lower velocity limits: " << velocity_limits.first);
  ROS_DEBUG_STREAM("Upper velocity limits: " << velocity_limits.second);
  ROS_DEBUG_STREAM("Lower force limits: " << force_limits.first);
  ROS_DEBUG_STREAM("Upper force limits: " << force_limits.second);

  /* TODO: Debug information on what was configured accordingly to the parameter server? */
  return true;
}

bool RobotiqNode::connectToRobotiqDevice()
{
  if (robotiq_ == 0)
  {
    CAROS_FATALERROR("The Robotiq device is not configured", ROBOTIQNODE_INTERNAL_ERROR);
    return false;
  }

  if (robotiq_->isConnected())
  {
    ROS_ERROR_STREAM(
        "'"
        << __PRETTY_FUNCTION__
        << "' invoked even though a connection to the Robotiq device has already been established - this is a bug!");
    return false;
  }

  /* Connect according to configured parameters */
  if (!robotiq_->connect(ip_, port_))
  {
    CAROS_FATALERROR("The Robotiq hand was not able to connect to " << ip_ << ":" << port_
                     << " or the hand could not be activated (is it connected to the controller box?)",
                     ROBOTIQNODE_ROBOTIQ_DEVICE_CONNECT_FAILED);
    return false;
  }

  /* Only very rare and obscure situations should cause this to fail, since the above connect was successful */
  if (!robotiq_->isConnected())
  {
    /* Something went wrong right after connecting */
    CAROS_FATALERROR("Failed to properly connect to the Robotiq device.", ROBOTIQNODE_ROBOTIQ_DEVICE_CONNECT_FAILED);
    return false;
  }

  return true;
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
bool RobotiqNode::moveQ(const rw::math::Q& q)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (robotiq_->getNumberOfJoints() != q.size())
  {
    CAROS_ERROR("MoveQ called with a wrong number of joints.", ROBOTIQNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  ROS_DEBUG_STREAM("moveQ with " << q.size() << " joint(s).");

  try
  {
    robotiq_->moveCmd(q);
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQNODE_INTERNAL_ERROR);
    return false;
  }
  return true;
}

bool RobotiqNode::gripQ(const rw::math::Q& q)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (robotiq_->getNumberOfJoints() != q.size())
  {
    CAROS_ERROR("GripQ called with a wrong number of joints.", ROBOTIQNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  try
  {
    robotiq_->moveCmd(q);
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQNODE_INTERNAL_ERROR);
    return false;
  }
  return true;
}

bool RobotiqNode::setForceQ(const rw::math::Q& q)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (robotiq_->getNumberOfJoints() != q.size())
  {
    CAROS_ERROR("SetForceQ called with a wrong number of joints.", ROBOTIQNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  robotiq_->setTargetQForce(q);

  return true;
}

bool RobotiqNode::setVelocityQ(const rw::math::Q& q)
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  if (robotiq_->getNumberOfJoints() != q.size())
  {
    CAROS_ERROR("SetVelocityQ called with a wrong number of joints.", ROBOTIQNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  robotiq_->setTargetQVel(q);

  return true;
}

bool RobotiqNode::stopMovement()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  try
  {
    robotiq_->stopCmd();
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool RobotiqNode::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (robotiq_ == 0)
  {
    CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQNODE_NO_ROBOTIQ_DEVICE);
    return false;
  }

  if (!robotiq_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQNODE_ROBOTIQ_DEVICE_NO_CONNECTION);
    return false;
  }

  return true;
}
}  // namespace caros
