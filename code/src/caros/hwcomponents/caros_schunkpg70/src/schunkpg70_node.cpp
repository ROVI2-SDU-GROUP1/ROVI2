#include <caros/schunkpg70_node.h>

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Exception.hpp>
#include <rw/math/MetricUtil.hpp>

#include <rwhw/schunkpg70/SchunkPG70.hpp>
#include <rwhw/PowerCube/Cube.hpp>

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <utility>
#include <cstddef>  // Provides NULL

/* Notes:
 * This node is designed to be run in a single thread that doesn't allow concurrently processing of the set-commands
 * and/or the runLoopHook - eliminating the possibility of race conditions.
 */

namespace caros
{
SchunkPG70Node::SchunkPG70Node(const ros::NodeHandle& nodehandle)
    : caros::CarosNodeServiceInterface(nodehandle),
      caros::GripperServiceInterface(nodehandle),
      node_handle_(nodehandle),
      schunkpg70_(0)
{
  /* Currently nothing specific should happen */
  last_q_ = rw::math::Q(1, 0.0);
}

SchunkPG70Node::~SchunkPG70Node()
{
  if (schunkpg70_ != 0)
  {
    if (schunkpg70_->isConnected())
    {
      ROS_DEBUG_STREAM(
          "Still connected to the SchunkPG70 device when destroying the SchunkPG70Node object - going to stop the "
          "device and "
          "disconnect.");
      schunkpg70_->stop();
      schunkpg70_->disconnect();
    }
    delete schunkpg70_;
    schunkpg70_ = 0;
  }
  else
  {
    ROS_DEBUG_STREAM(
        "There was no SchunkPG70 device to destroy before deallocating/destroying the SchunkPG70Node object.");
  }
}

bool SchunkPG70Node::activateHook()
{
  if (!configureSchunkPG70Device())
  {
    return false;
  }

  if (!connectToSchunkPG70Device())
  {
    return false;
  }

  if (!GripperServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS GripperServiceInterface could not be configured correctly.",
                     SCHUNKPG70NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool SchunkPG70Node::configureSchunkPG70Device()
{
  if (schunkpg70_ != 0)
  {
    /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
    CAROS_FATALERROR(
        "The SchunkPG70 device is already active - trying to configure an already configured SchunkPG70 node is a bug!",
        SCHUNKPG70NODE_SCHUNKPG70_DEVICE_ALREADY_ACTIVE);
    return false;
  }

  schunkpg70_ = new rwhw::SchunkPG70;

  /* Fetch parameters (if any) or use the defaults */
  node_handle_.param("serial_device", serial_device_, std::string("/dev/ttyUSB0"));

  return true;
}

bool SchunkPG70Node::connectToSchunkPG70Device()
{
  if (schunkpg70_ == 0)
  {
    CAROS_FATALERROR("Can not connect to the SchunkPG70 device when it has not yet been configured.",
                     SCHUNKPG70NODE_INTERNAL_ERROR);
    return false;
  }

  if (schunkpg70_->isConnected())
  {
    CAROS_FATALERROR(
        "A connection to the SchunkPG70 device has already been established, so trying to connect to it again is a "
        "bug!",
        SCHUNKPG70NODE_INTERNAL_ERROR);
    return false;
  }

  /* Connect according to interface type and configured parameters */
  ROS_DEBUG_STREAM("Connecting using serial device: " << serial_device_);
  if (schunkpg70_->connectSerial(serial_device_))
  {
    ROS_ASSERT(schunkpg70_->isConnected());
    ROS_DEBUG_STREAM("Connection successful.");
  }
  else
  {
    /* Something went wrong when connecting */
    ROS_ASSERT(!schunkpg70_->isConnected());
    CAROS_FATALERROR("Failed to properly connect to the SchunkPG70 device.",
                     SCHUNKPG70NODE_SCHUNKPG70_DEVICE_CONNECT_FAILED);
    return false;
  }

  return true;
}

bool SchunkPG70Node::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  ROS_ERROR_STREAM("The recoverHook() has not been implemented yet!");

  return false;
}

void SchunkPG70Node::runLoopHook()
{
  static bool first_invocation = true;

  if (schunkpg70_ == 0)
  {
    CAROS_FATALERROR("The SchunkPG70 device is not configured", SCHUNKPG70NODE_INTERNAL_ERROR);
    return;
  }

  if (!schunkpg70_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SchunkPG70 device.",
                SCHUNKPG70NODE_SCHUNKPG70_DEVICE_NO_CONNECTION);
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
  rw::math::Q q(1, 0.0);
  schunkpg70_->getQ(q);
  rw::math::Q dq_calc = (q - last_q_) / diff.toSec();
  // There is no feedback on the force/current, defaulting to an empty Q
  rw::math::Q force(0);

  // Manually looking at q values and speed to diagnose whether the gripper is moving, blocked or stopped - there is no
  // functionality for this built into the RobWorkHardware schunkpg70 driver.
  bool is_moving = (dq_calc[0] != 0.0) ? true : false;
  bool is_blocked = (dq_calc[0] == 0 && q != target_q_) ? true : false;
  bool is_stopped = !is_moving && !is_blocked;
  /* FIXME: hardcoded isEstop value */
  bool is_emergency_stopped = false;
  /* Don't publish wrong information when first invoked, because the last_q_ and last_loop_time_ are initialised to 0 */
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

void SchunkPG70Node::errorLoopHook()
{
  /* Stop the SchunkPG70's current action(s) */
  if (schunkpg70_ == 0)
  {
    ROS_DEBUG_STREAM("The SchunkPG70 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (schunkpg70_->isConnected())
    {
      schunkpg70_->stop();
    }
  }
}

void SchunkPG70Node::fatalErrorLoopHook()
{
  /* Stop the SchunkPG70's current action(s) */
  if (schunkpg70_ == 0)
  {
    ROS_DEBUG_STREAM("The SchunkPG70 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (schunkpg70_->isConnected())
    {
      schunkpg70_->stop();
      /* A fatal error should disconnect the SchunkPG70 device */
      schunkpg70_->disconnect();
    }
  }
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
bool SchunkPG70Node::moveQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "moveQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  target_q_ = q;

  bool ret = false;
  /* TODO:
   * This is a blocking call, so this will make this node become unresponsive until the movement has finished.
   * This is the chosen design choice for now, since introducing concurrency could result in weird errors within the
   * rwhw schunkpg70 driver.
   */
  ret = schunkpg70_->setQ(q);

  return ret;
}

bool SchunkPG70Node::gripQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "gripQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  // Throw away the supplied q, as the rwhw driver basically only support closing the parallel gripper and not grasping
  // at a specific Q-configuration.
  schunkpg70_->close();

  return true;
}

bool SchunkPG70Node::setForceQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setForceQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  // datasheet says that the maximum force is 200N, this is assumed linear for 0 to 100, where 100 == 200N
  double power = q[0] * 100 / 200;
  if (power > 100)
  {
    power = 100;
  }
  else if (power < 0)
  {
    power = 0;
  }
  // set the power accordingly
  try
  {
    schunkpg70_->setGraspPowerPct(power);
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), SCHUNKPG70NODE_INTERNAL_ERROR);
  }

  ROS_DEBUG_STREAM_NAMED("received_q", "setForceQ: " << q << " was translated into " << power << "%");

  return true;
}

bool SchunkPG70Node::setVelocityQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setVelocityQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  ROS_ERROR_STREAM("Specifying the velocity is not supported!");
  return false;
}

bool SchunkPG70Node::stopMovement()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  schunkpg70_->stop();

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool SchunkPG70Node::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (schunkpg70_ == 0)
  {
    CAROS_FATALERROR("The SchunkPG70 device is not configured.", SCHUNKPG70NODE_NO_SCHUNKPG70_DEVICE);
    return false;
  }

  if (!schunkpg70_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SchunkPG70 device.",
                SCHUNKPG70NODE_SCHUNKPG70_DEVICE_NO_CONNECTION);
    return false;
  }

  return true;
}

bool SchunkPG70Node::supportedQSize(const rw::math::Q& q)
{
  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SCHUNKPG70NODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SCHUNKPG70NODE,
                SCHUNKPG70NODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  return true;
}
}  // namespace caros
