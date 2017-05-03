#include <caros/sdh_node.h>

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Exception.hpp>
#include <rw/math/MetricUtil.hpp>

#include <rwhw/sdh/SDHDriver.hpp>

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <utility>
#include <cstddef>  // Provides NULL
#include <cstdint>

/* Notes:
 * This node is designed to be run in a single thread that doesn't allow concurrently processing of the set-commands
 * and/or the runLoopHook - eliminating the possibility of race conditions.
 */

/* TODO:
 * The node implementation can be greatly simplified if the workaround for the movement of the SDH fingers can be placed
 *in RobWorkHardware or simply removed (requires collecting the debug data and comparing with the SDH temperature).
 * Add support for emergency stop. This can be triggered either through CarosNodeServiceInterface or directly to this
 *node. The SDH library contains a function for emergency stop, which should be used and called immediately.
 *   - What is required to actually release the emergencystop (within the rwhw::SDHDriver / SDHLibrary)?
 *
 * Could keep track of whether the services/commands (moveQ, gripQ, etc.) finish executing before new commands are
 *received (it's not an error, but maybe it would be nice to get some debug/info information regarding this. Hopefully
 *it would make it easier to reason/debug "what just happened").
 */

namespace caros
{
SDHNode::SDHNode(const ros::NodeHandle& nodehandle)
    : caros::CarosNodeServiceInterface(nodehandle),
      caros::GripperServiceInterface(nodehandle),
      node_handle_(nodehandle),
      current_state_(SDHNode::WAIT),
      next_state_(current_state_),
      sdh_(0)
{
  /* Currently nothing specific should happen */
}

SDHNode::~SDHNode()
{
  if (sdh_ != 0)
  {
    if (sdh_->isConnected())
    {
      ROS_DEBUG_STREAM(
          "Still connected to the SDH device when destroying the SDHNode object - going to stop the device and "
          "disconnect.");
      sdh_->stop();
      sdh_->disconnect();
    }
    delete sdh_;
    sdh_ = 0;
  }
  else
  {
    ROS_DEBUG_STREAM("There was no SDH device to destroy before deallocating/destroying the SDHNode object.");
  }
}

bool SDHNode::activateHook()
{
  if (!configureSDHDevice())
  {
    return false;
  }

  if (!connectToSDHDevice())
  {
    return false;
  }

  if (!GripperServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS GripperServiceInterface could not be configured correctly.",
                     SDHNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  /* Outputting information on supported value ranges */
  /* TODO: This could be made part of the GripperServiceInterface - possibly as a message that is returned (or
   * published) when a client asks for it.
   * If the hardware is intelligent enough to provide new values/boundaries according to position or grasping mode, then
   * it could make sense to publish that information when it changes
   */
  std::pair<rw::math::Q, rw::math::Q> position_limits = sdh_->getPosLimits();
  rw::math::Q velocity_limits = sdh_->getVelLimits();
  rw::math::Q acceleration_limits = sdh_->getAccLimits();
  rw::math::Q current_limits = sdh_->getCurrentLimits();

  ROS_ERROR_STREAM_COND(position_limits.first.size() != position_limits.second.size(),
                        "The sizes of the Q's in the position limit pair are not equal; first contains "
                            << position_limits.first.size() << " and second contains " << position_limits.second.size()
                            << " elements.");

  ROS_DEBUG_STREAM("Lower position limits: " << position_limits.first);
  ROS_DEBUG_STREAM("Upper position limits: " << position_limits.second);
  ROS_DEBUG_STREAM("Velocity limits: " << velocity_limits);
  ROS_DEBUG_STREAM("Acceleration limits: " << acceleration_limits);
  ROS_DEBUG_STREAM("Current limits: " << current_limits);

  return true;
}

bool SDHNode::configureSDHDevice()
{
  if (sdh_ != 0)
  {
    /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
    CAROS_FATALERROR("The SDH device is already active - trying to configure an already configured SDH node is a bug!",
                     SDHNODE_SDH_DEVICE_ALREADY_ACTIVE);
    return false;
  }
  sdh_ = new rwhw::SDHDriver;

  /* Fetch parameters (if any) or use the defaults */
  node_handle_.param("interface_type", interface_type_, std::string("CAN"));

  node_handle_.param("rs232_device", rs232_device_, std::string(""));
  node_handle_.param("rs232_port", rs232_port_, 0);
  node_handle_.param("rs232_baudrate", rs232_baudrate_, 115200);
  node_handle_.param("rs232_timeout", rs232_timeout_, 0.5);

  node_handle_.param("can_device", can_device_, std::string("/dev/pcan0"));
  node_handle_.param("can_baudrate", can_baudrate_, 1000000);
  node_handle_.param("can_timeout", can_timeout_, 0.5);

  /* TODO: Debug information on what was configured accoringly to the parameter server? */
  return true;
}

bool SDHNode::connectToSDHDevice()
{
  if (sdh_ == 0)
  {
    CAROS_FATALERROR("Can not connect to the SDH device when it has not yet been configured.", SDHNODE_INTERNAL_ERROR);
    return false;
  }

  if (sdh_->isConnected())
  {
    CAROS_FATALERROR(
        "A connection to the SDH device has already been established, so trying to connect to it again is a bug!",
        SDHNODE_INTERNAL_ERROR);
    return false;
  }

  /* Connect according to interface type and configured parameters */
  if (interface_type_ == "RS232")
  {
    if (rs232_device_.empty())
    {
      sdh_->connect(rs232_port_, static_cast<std::uint64_t>(rs232_baudrate_), rs232_timeout_, NULL);
    }
    else
    {
      sdh_->connect(rs232_port_, static_cast<std::uint64_t>(rs232_baudrate_), rs232_timeout_, rs232_device_.c_str());
    }
  }
  else if (interface_type_ == "CAN")
  {
    sdh_->connect(can_device_, can_baudrate_, can_timeout_);
  }
  else
  {
    CAROS_FATALERROR("The specified interface '" << interface_type_ << "' is not supported.",
                     SDHNODE_UNSUPPORTED_INTERFACE_TYPE);
    return false;
  }

  /* Verify that the connection to the SDH device has been established - this eliminates the need for verifying that the
   * sdh_->connect() function calls actually succeed */
  if (!sdh_->isConnected())
  {
    /* Something went wrong when connecting */
    CAROS_FATALERROR("Failed to properly connect to the SDH device.", SDHNODE_SDH_DEVICE_CONNECT_FAILED);
    return false;
  }

  return true;
}

bool SDHNode::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* Remember to place the state machine in a proper state according to the recovery (e.g. WAIT) */

  ROS_ERROR_STREAM("The recoverHook() has not been implemented yet!");

  return false;
}

void SDHNode::runLoopHook()
{
  try
  {
    if (sdh_ == 0)
    {
      CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
      return;
    }

    if (!sdh_->isConnected())
    {
      CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
      return;
    }

    /************************************************************************
     * Get Current Joint Positions
     * Currently used as part of the workaround for the movement of the SDH fingers.
    ************************************************************************/
    current_q_ = sdh_->getQ();

    /************************************************************************
     * Velocity Calculation
     * Used as part of the workaround for the movement of the SDH fingers.
    ************************************************************************/
    double time = vel_update_timer_.getTime();

    if (last_q_.size() != current_q_.size())
    {
      /* lastq_ has not been set before (first time the runloopHook is being called), so set it to current_q_ */
      last_q_ = current_q_;
    }
    else if (time > MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY)
    {
      /* calculate velocity */
      vel_q_ = (last_q_ - current_q_) / time;
      last_q_ = current_q_;
      vel_update_timer_.resetAndResume();
      ROS_DEBUG_STREAM_NAMED("velocity", "New calculated velocity: " << vel_q_);
    }
    ROS_DEBUG_STREAM_NAMED("velocity", "Calculated velocity: " << vel_q_);
    ROS_DEBUG_STREAM_NAMED("velocity", "SDH reported velocity: " << sdh_->getdQ());

    /************************************************************************
     * Publish SDH state
     ************************************************************************/
    /* Publishing the SDH state before the state machine because then the measured/sampled values will (probably) have
     *the best match with the current SDH action(s).
     *
     * The units of the reported values should match what is specified in the GripperState.msg specification.
     * The rwhw::SDHDriver constructor specifies the use of radians.
     */
    rw::math::Q q = current_q_;
    /* Using the calculated velocity (FIXME: either continue to use vel_q_ or use sdh_->getdQ() if they report similar
     * values - see the velocity debug messages) */
    rw::math::Q dq = vel_q_;
    /* FIXME: the current could be converted to a force, given it would make sense to do so - but it requires knowledge
     * of the kinematics to calculate the force that is being applied e.g. at a particular fingertip or where the
     * contact surface is. */
    rw::math::Q force = sdh_->getQCurrent();

    rw::math::Q compare = rw::math::Q::zero(dq.size());
    bool is_moving = (compare != dq) ? true : false;
    bool is_blocked = false;
    /* TODO: This can possibly give a wrong report in the situation where a new move_q has just been initiated - so the
     * calculated distance between current position and target is greater than the threshold, but the
     * measured/calculated velocity is still zero (and is_moving is "falsely" set to false) */
    /* Only calculate the distance when move_q_ has the same length/size as current_q_. */
    if (move_q_.size() == current_q_.size())
    {
      if (!is_moving && (rw::math::MetricUtil::dist2(current_q_, move_q_) >= MOVE_DISTANCE_STOPPED_THRESHOLD))
      {
        is_blocked = true;
      }
    }
    bool is_stopped = true;
    /* If not moving nor blocked, then it must be stopped (i.e. reached the target (see GripperState.msg specification))
     */
    if (is_moving || is_blocked)
    {
      is_stopped = false;
    }
    /* TODO: properly handle is_emergency_stopped - the logic to register whether an emergency stop is activated or
     * deactivated is missing. */
    bool is_emergency_stopped = false;
    publishState(q, dq, force, is_moving, is_blocked, is_stopped, is_emergency_stopped);

    /************************************************************************
     * State Machine
     * Used to apply workaround for the movement of the SDH fingers.
    ************************************************************************/
    current_state_ = next_state_;
    switch (current_state_)
    {
      case WAIT:
        /* Do nothing */
        break;
      case MOVE_WAIT:
        /* Workaround to avoid having the SDH try to move the fingers until
         * MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING has passed, when the fingers are almost at their
         * target - (there should be a "bug" causing the SDH to dissipate power when the target can't be reached
         * according to the firmware) */
        if (rw::math::MetricUtil::dist2(current_q_, move_q_) < MOVE_DISTANCE_STOPPED_THRESHOLD)
        {
          /* Debug functionality to test the usage of sdh_->waitCmd(0) instead of looking at the remaining distance to
           * the target */
          ROS_DEBUG_STREAM_NAMED("move_wait", "sdh_->waitCmd(0) returned: " << sdh_->waitCmd(0));

          sdh_->stop();
          next_state_ = WAIT;
        }
        else if (move_start_timer_.getTime() > MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING)
        {
          ROS_DEBUG_STREAM_NAMED("move_wait", "Waited long enough to possible intervene.");
          if (rw::math::MetricUtil::normInf(vel_q_) < MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING)
          {
            ROS_DEBUG_STREAM_NAMED("move_wait", "Intervening due to the fingers not being considered moving.");
            sdh_->stop();
            next_state_ = WAIT;
          }
        }
        break;
      default:
        ROS_FATAL_STREAM("Unknown state in the SDH state machine '"
                         << current_state_ << "' (a value is expected due to enum implementation) - This is a bug!");
        /* This is considered a fatal error, but should never happen. */
        CAROS_FATALERROR("Unknown state in the SDH state machine", SDHNODE_INTERNAL_ERROR);
        break;
    }
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return;
  }
}

void SDHNode::errorLoopHook()
{
  /* Stop the SDH's current action(s) */
  if (sdh_ == 0)
  {
    ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (sdh_->isConnected())
    {
      sdh_->stop();
    }
  }
}

void SDHNode::fatalErrorLoopHook()
{
  /* Stop the SDH's current action(s) */
  if (sdh_ == 0)
  {
    ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (sdh_->isConnected())
    {
      sdh_->stop();
      /* A fatal error should disconnect the SDH device */
      sdh_->disconnect();
    }
  }
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
/* Note:
 * The checks isInRunning(), (sdh_ == 0) and (! sdh_->isConnected()) are not placed in one common function, because the
 * CAROS_ERROR and CAROS_FATALERROR macros are using source code lines to sort of pinpoint the "faulty" function.
 * When a more appropriate method that can reduce this code duplication is found, then it should be implemented! (A
 * preprocessor code generating macro is not exactly a nice and easily maintainable solution)
 */
bool SDHNode::moveQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "moveQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  try
  {
    move_q_ = q;
    move_start_timer_.resetAndResume();

    sdh_->moveCmd(move_q_);
    next_state_ = MOVE_WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::gripQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "gripQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  try
  {
    sdh_->moveCmd(q);
    /* Do nothing; letting the SDH continue to apply force as part of its grasp */
    next_state_ = WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::setForceQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setForceQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  try
  {
    sdh_->setTargetQCurrent(q);
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::setVelocityQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setVelocityQ: " << q);

  if (!isInWorkingCondition() || !supportedQSize(q))
  {
    return false;
  }

  try
  {
    sdh_->setTargetQVel(q);
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::stopMovement()
{
  if (!isInWorkingCondition())
  {
    return false;
  }

  try
  {
    sdh_->stop();
    next_state_ = WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool SDHNode::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  return true;
}

bool SDHNode::supportedQSize(const rw::math::Q& q)
{
  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE,
                SDHNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  return true;
}
}  // namespace caros
