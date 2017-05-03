#include <caros/caros_node_service_interface.h>
#include <caros_common_msgs/CarosNodeState.h>

#include <ros/ros.h>

#include <string>

namespace caros
{
namespace
{
/* The order of the states _has_ to be the same as the order defined in CarosNodeServiceInterface.hpp for NodeState */
static std::string g_caros_state_string[] = {"PREINIT", "RUNNING", "ERROR", "FATALERROR"};
}

CarosNodeServiceInterface::CarosNodeServiceInterface(const ros::NodeHandle& nodehandle,
                                                     const double loop_rate_frequency)
    : nodehandle_(nodehandle, CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE),
      node_state_(PREINIT),
      loop_rate_frequency_(loop_rate_frequency),
      loop_rate_(loop_rate_frequency_)
{
  if (!initCarosNode())
  {
    throw std::runtime_error("Could not properly initialise the required basic services and/or publishers.");
  }
}

CarosNodeServiceInterface::~CarosNodeServiceInterface()
{
  /* Nothing specific to destroy */
}

void CarosNodeServiceInterface::start()
{
  if (!activateNode())
  {
    ROS_DEBUG_STREAM(
        "activateNode() was unsuccessful - the node should now be in an error state according to best practices.");
  }

  caros_common_msgs::CarosNodeState state;
  while (ros::ok())
  {
    ros::spinOnce();

    ROS_FATAL_STREAM_COND(node_state_ == PREINIT, "The CAROS node is not supposed to be in the state '"
                                                      << g_caros_state_string[PREINIT]
                                                      << "' at this point - this is a bug!");
    ROS_ASSERT(node_state_ != PREINIT);

    /* Transitions to the error states will be handled right away in the same ROS cycle */

    if (node_state_ == RUNNING)
    {
      runLoopHook();
    }
    /* Process errors if any occurred */
    if (node_state_ == ERROR)
    {
      errorLoopHook();
    }
    /* Also process fatal error state (if an error becomes a fatal error, then it will also be processed in the same ROS
     * cycle) */
    if (node_state_ == FATALERROR)
    {
      fatalErrorLoopHook();
    }

    publishNodeState();

    /* Sleep in order to run approximately at the specified frequency */
    loop_rate_.sleep();
    /* TODO:
     * Replace this sleeping using a ros::Rate with a ros::Timer using a callback function - that is the recommended way
     * to do it according to http://wiki.ros.org/roscpp/Overview/Time#Sleeping_and_Rates
     * Also output some debug information related to how well the scheduling is going, so delays can easily be seen in
     * the logs.
     */
  }
}

bool CarosNodeServiceInterface::activateNode()
{
  // can only be called when in PREINIT state
  if (node_state_ != PREINIT)
  {
    ROS_ERROR_STREAM("Activate can only be called when in "
                     << g_caros_state_string[PREINIT] << " state. The node was in " << g_caros_state_string[node_state_]
                     << ".");
    return false;
  }

  if (activateHook())
  {
    changeState(RUNNING);
  }
  else
  {
    /* It is considered a fatal error if the activateHook() failed and this object was not placed in an error state! */
    NodeState current_state = getState();
    if (current_state != ERROR && current_state != FATALERROR)
    {
      ROS_FATAL_STREAM("activateHook() failed and the CAROS node was never put into an error state - this is a bug!");
    }
    else
    {
      ROS_DEBUG_STREAM("activateHook() failed.");
    }

    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::recoverNode()
{
  // can only be called when in ERROR state
  if (node_state_ != ERROR)
  {
    ROS_WARN_STREAM("Recover can only be called from " << g_caros_state_string[ERROR] << " state. The node was in "
                                                       << g_caros_state_string[node_state_] << ".");
    return false;
  }

  if (recoverHook(error_msg_, error_code_))
  {
    ROS_ERROR_STREAM_COND(previous_state_ == FATALERROR, "A successful recovery brings the node back into the "
                                                             << g_caros_state_string[FATALERROR]
                                                             << " state - This is a bug!");
    changeState(previous_state_);
  }
  else
  {
    ROS_DEBUG_STREAM("recoverHook() failed.");

    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::terminateNode()
{
  /* Can be called at all times */
  /* Simply request shutdown - a non blocking call and shutdown will occur at next "ROS cycle". */
  ros::requestShutdown();

  return true;
}

void CarosNodeServiceInterface::error(const std::string& msg, const int64_t error_code)
{
  ROS_DEBUG_STREAM("CarosNodeError: " << msg << "; error code: " << error_code);
  /* keep a copy of the error message so it can be published */
  error_msg_ = msg;
  error_code_ = error_code;
  changeState(ERROR);
}

void CarosNodeServiceInterface::fatalError(const std::string& msg, const int64_t error_code)
{
  ROS_DEBUG_STREAM("CarosNodeFatalError: " << msg << "; error code: " << error_code);
  /* keep a copy of the (fatal) error message so it can be published */
  error_msg_ = msg;
  error_code_ = error_code;
  changeState(FATALERROR);
}

void CarosNodeServiceInterface::setLoopRateFrequency(const double frequency)
{
  ROS_DEBUG_STREAM("Changing the loop rate frequency from " << loop_rate_frequency_ << " to " << frequency);
  loop_rate_frequency_ = frequency;
  loop_rate_ = ros::Rate(loop_rate_frequency_);
}

void CarosNodeServiceInterface::changeState(const NodeState new_state)
{
  ROS_DEBUG_STREAM("Changing state from " << g_caros_state_string[node_state_] << " to "
                                          << g_caros_state_string[new_state]);
  if (new_state != node_state_)
  {
    previous_state_ = node_state_;
    node_state_ = new_state;
    publishNodeState(true);
  }
  else
  {
    ROS_DEBUG_STREAM("Not changing state as the new state is the same as the current state!");
  }
}

bool CarosNodeServiceInterface::initCarosNode()
{
  if (node_state_publisher_ || srv_recover_ || srv_terminate_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more CarosNodeServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  node_state_publisher_ = nodehandle_.advertise<caros_common_msgs::CarosNodeState>("caros_node_state", 1);
  ROS_ERROR_STREAM_COND(!node_state_publisher_, "The caros_node_state publisher is empty!");

  srv_recover_ = nodehandle_.advertiseService("recover", &CarosNodeServiceInterface::recoverHandle, this);
  ROS_ERROR_STREAM_COND(!srv_recover_, "The recover service is empty!");

  srv_terminate_ = nodehandle_.advertiseService("terminate", &CarosNodeServiceInterface::terminateHandle, this);
  ROS_ERROR_STREAM_COND(!srv_terminate_, "The terminate service is empty!");

  if (node_state_publisher_ && srv_recover_ && srv_terminate_)
  {
    /* Everything seems to be properly initialised */
  }
  else
  {
    ROS_FATAL_STREAM("One or more of the ROS publishers or services could not be properly initialised.");
    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return recoverNode();
}

bool CarosNodeServiceInterface::terminateHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return terminateNode();
}

void CarosNodeServiceInterface::publishNodeState(const bool state_changed)
{
  caros_common_msgs::CarosNodeState state;
  state.state = g_caros_state_string[node_state_];
  state.inError = node_state_ == ERROR || node_state_ == FATALERROR;
  if (state.inError)
  {
    state.error_msg = error_msg_;
    state.error_code = error_code_;
  }
  /* The error_msg is not being cleared when the state no longer is in error, this is intended to provide a minor error
   * log / history of errors.
   * TODO: Provide a history/log of the last N errors together with some info such as a timestamp (maybe how long the
   * node was in the error state) and similar.
   */

  state.changed_event = state_changed;

  node_state_publisher_.publish(state);
}
}  // namespace caros
