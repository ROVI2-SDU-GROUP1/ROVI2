#ifndef CAROS_CAROS_NODE_SERVICE_INTERFACE_H
#define CAROS_CAROS_NODE_SERVICE_INTERFACE_H

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>
#include <stdexcept>

#define CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_node"

/**
 * \addtogroup CAROS_NODE_SI CAROS Node Service Interface
 *
 * The CAROS Node Service Interface consists of a statemachine that eases the process of creating a ROS node, also
 *called a CAROS node because it follows the CAROS conventions.
 * It mainly consists of the following state machine:
 * ![statemachine](../../../assets/caros_node_statemachine.png "CAROS Node statemachine")
 *
 * And is supposed to streamline the error functionality and reporting of CAROS nodes.
 *
 * The \link caros::CarosNodeServiceInterface::start() start()\endlink function is used for handing over the control to
 *the state machine, which will try to transition the node into the running state. In doing so, the \link
 *caros::CarosNodeServiceInterface::activateHook() activateHook()\endlink function is being invoked and allows for the
 *specific node implementation to e.g. establish connection to hardware device(s) and configure the supported service
 *interfaces, and otherwise do whatever is necessary to get into a running / live state (for more information see \link
 *caros::CarosNodeServiceInterface::activateHook() activateHook()\endlink)
 *
 * Error recovery is done through the ROS service *recover*, which invokes the \link
 *caros::CarosNodeServiceInterface::recoverHook() recoverHook()\endlink, allowing the node to react according to the
 *error codes. This has not been fully implemented yet nor has the supporting facilities for better error reporting been
 *properly designed yet.
 *
 * Loop hooks (e.g. \link caros::CarosNodeServiceInterface::runLoopHook() runLoopHook()\endlink) are invoked at the
 *frequency specified through the \link
 *caros::CarosNodeServiceInterface::CarosNodeServiceInterface(const ros::NodeHandle&, const double) constructor\endlink
 *or \link caros::CarosNodeServiceInterface::setLoopRateFrequency() setLoopRateFrequency()\endlink. They allow for
 *having functionality be executed periodically, such as sampling from
 *the connected device/sensor and/or publish information regarding the current status of the node (see the actual node
 *implementations for more concrete examples).
 *
 * \link CAROS_ERROR CAROS_ERROR(...)\endlink, are errors meant to be somewhat autonomously corrected (e.g. another node
 *has to be notified of the error and possibly recover depending on the situation).
 *
 * \link CAROS_FATALERROR CAROS_FATALERROR(...)\endlink, are errors that require human intervention and can't be
 *autonomously corrected, such as a broken connection to the device. Solving the error should also result in the node
 *being restarted/respawned.
 *
 * @{
 */

/**
 * @brief Emit an CAROS node error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_ERROR("The value of x is " << x << ". x should be less than zero.", 2);
\endcode
 *
 */
#define CAROS_ERROR(ostream_expression, error_code)                                                \
  do                                                                                               \
  {                                                                                                \
    std::stringstream ERROR__stream;                                                               \
    ERROR__stream << ostream_expression;                                                           \
    ROS_ERROR_STREAM("CarosNodeError: " << ERROR__stream.str() << "; error code: " << error_code); \
    error(ERROR__stream.str(), error_code);                                                        \
  } while (0)

/**
 * @brief Emit an CAROS node fatal error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_FATALERROR("The value of x is " << x << ". x must not be less than zero.", 5);
\endcode
 *
 */
#define CAROS_FATALERROR(ostream_expression, error_code)                                                     \
  do                                                                                                         \
  {                                                                                                          \
    std::stringstream FATALERROR__stream;                                                                    \
    FATALERROR__stream << ostream_expression;                                                                \
    ROS_ERROR_STREAM("CarosNodeFatalError: " << FATALERROR__stream.str() << "; error code: " << error_code); \
    fatalError(FATALERROR__stream.str(), error_code);                                                        \
  } while (0)

/**
 * @}
 */
namespace caros
{
namespace CAROS_NODE_ERRORCODES
{
//! \ingroup CAROS_NODE_SI
enum CAROS_NODE_ERRORCODE
{
  CAROS_NODE_NO_ERROR_CODE_SUPPLIED = 0
};
}
typedef CAROS_NODE_ERRORCODES::CAROS_NODE_ERRORCODE CAROS_NODE_ERRORCODE;

/**
 * \ingroup CAROS_NODE_SI
 * @brief A node service interface that defines a simple statemachine from which
 * the node can be controlled.
 */
class CarosNodeServiceInterface
{
 public:
  /**
   * @brief constructor.
   * @param[in] nodehandle The nodehandle to use for ROS services and publishers.
   * @param[in] loop_rate_frequency Optional parameter that specifies the frequency [Hz] of this ROS node - see
   * setLoopRateFrequency() for more information.
   *
   * @throws std::runtime_error if the basic functionality can't be properly initialised.
   * @note Marked explicit to enforce that an actual nodehandle is being passed in.
   */
  explicit CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loop_rate_frequency = 30);

  /**
   * @brief virtual destructor
   */
  virtual ~CarosNodeServiceInterface();

  /**
   * @brief Start the CAROS node
   *
   * Invoke this function to hand over the control to the CAROS node statemachine.
   * This is a blocking function that will return when ROS is being told to shutdown.
   */
  void start();

  /**
   * @brief The states that the CAROS node can be in
   */
  enum NodeState
  {
    PREINIT = 0,
    RUNNING,
    ERROR,
    FATALERROR
  };

 protected:
  /**
   * @name ROS Service Hooks
   * @brief These hooks needs to be implemented in the deriving node, allowing for a common interface for controlling
   * CAROS nodes.
   */
  /** @{ */
  /**
   * @brief This is called when the node is transitioning to the RUNNING state, which will happen automatically when
   *invoking start()
   *
   * This hook should be used to establish connections to the hardware and activate the hardware.
   * It is also here that other interfaces should be initialised (e.g. the
   *caros::GripperServiceInterface::configureInterface()), together with
   *advertising ROS services and publishers that are specific to the node.
   * If an error occurs, then either @ref caros::CAROS_ERROR or @ref caros::CAROS_FATALERROR should be called depending
   *on the severity of the error,
   *and false returned.
   */
  virtual bool activateHook() = 0;

  /**
   * @brief This is called as part of a recovery process that is invoked through the ROS service "recover".
   * @param[in] error_msg The corresponding error message.
   * @param[in] error_code The error code that should be recovered from.
   *
   * This hook should be used to perform any necessary steps to recover from the error.
   * @note The design of the recovery process is to be considered incomplete. Some things are missing such as the
   *ability to
   *properly see what error the node has been told to recover from (it's available in the CarosNodeService, but how to
   *properly use it when recovering is undecided).
   */
  virtual bool recoverHook(const std::string& error_msg, const int64_t error_code) = 0;
  /** @} */

  /**
   * @name Loop Hooks
   *
   * The loop hook corresponding to the current state will be invoked at the frequency specified when calling the
   *constructor CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loop_rate_frequency) or set
   *through setLoopRateFrequency().
   */
  /** @{ */
  /**
   * @brief This is invoked in the RUNNING state.
   */
  virtual void runLoopHook() = 0;

  /**
   * @brief This is invoked in the ERROR state.
   */
  virtual void errorLoopHook()
  {
    /* Empty */
  }

  /**
   * @brief This is invoked in the FATALERROR state.
   */
  virtual void fatalErrorLoopHook()
  {
    /* Empty */
  }
  /** @} */

  /**
   * @brief Get current state.
   * @returns the current state.
   */
  NodeState getState()
  {
    return node_state_;
  }

  /**
   * @brief Get the previous state.
   * @returns the previous state.
   */
  NodeState getPreviousState()
  {
    return previous_state_;
  }

  /** @{ */
  bool isInRunning()
  {
    return node_state_ == RUNNING;
  }
  bool isInError()
  {
    return node_state_ == ERROR;
  }
  bool isInFatalError()
  {
    return node_state_ == FATALERROR;
  }
  /** @} */

  /**
   * @brief Change the frequency of this ROS node.
   *
   * @param[in] frequency The new frequency [Hz].
   *
   * Change how often this node is supposed to execute ROS service callbacks and publish CAROS node messages.
   * A very small value or a negative value will (according to the current roscpp implementation) cause this ROS node to
   *process the service callbacks and publishers as fast as possible.
   */
  void setLoopRateFrequency(const double frequency);

  /**
   * @brief Get the frequency of this ROS node.
   *
   * @returns The frequency [Hz].
   */
  double getLoopRateFrequency()
  {
    return loop_rate_frequency_;
  }

  /**
   * @brief error function, should not be invoked directly but through the @ref caros::CAROS_ERROR macro.
   */
  void error(const std::string& msg,
             const int64_t error_code = CAROS_NODE_ERRORCODES::CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

  /**
   * @brief fatal error function, should not be invoked directly but through the @ref caros::CAROS_FATALERROR macro.
   */
  void fatalError(const std::string& msg,
                  const int64_t error_code = CAROS_NODE_ERRORCODES::CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  CarosNodeServiceInterface();

  /** @{ */
  /**
   * @brief Activate the CAROS node.
   *
   * This transitions the node into the RUNNING state. This should have the node establish connections to the hardware
   *and activate it, together with advertising the ROS services and publishers for the node.
   *
   * @pre In PREINIT state
   * @post Success: in RUNNING state
   * @post Failure: in one of the error states, depending on the severity of the failure
   */
  bool activateNode();

  /**
   * @brief Recover from an error.
   *
   * If it's possible to recover then the node will transition back into the state that the node was in before entering
   *the ERROR state.
   *
   * @pre In ERROR state
   * @post Success: in the previous state
   * @post Failure: In ERROR state [ TODO: Settle on a strategy possibly involving FATALERROR ]
   */
  bool recoverNode();

  /**
   * @brief Request the node to shutdown.
   *
   * This is a non-blocking call, so the shutdown will happen at the next "ROS cycle" where it checks whether a shutdown
   *has been requested (e.g. ros::ok())
   *
   * @pre In any state
   * @post In same state as before
   */
  bool terminateNode();
  /** @} */

  /**
   * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the
   *callbacks.
   *
   * @note This should not be called directly.
   */
  bool initCarosNode();

  /** @{ */
  /**
   * @brief ROS service wrapper for recoverNode().
   */
  bool recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
   * @brief ROS service wrapper for terminateNode().
   */
  bool terminateHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  /** @} */

  /**
   * @brief Change the state.
   *
   * @note Supposed to be called internally.
   */
  void changeState(const NodeState new_state);

  /**
   * @brief Publishes the state of this CAROS node.
   *
   * @note Supposed to be called internally.
   */
  void publishNodeState(const bool state_changed = false);

 private:
  ros::NodeHandle nodehandle_;

  ros::Publisher node_state_publisher_;

  ros::ServiceServer srv_recover_;
  ros::ServiceServer srv_terminate_;

  NodeState node_state_;
  NodeState previous_state_;

  double loop_rate_frequency_;
  ros::Rate loop_rate_;

  std::string error_msg_;
  /* Using int64_t because it's highly related with the type specified in the caros_common_msgs::caros_node_state
   * message
   */
  int64_t error_code_;
};
}  // namespace caros

#endif  // CAROS_CAROS_NODE_SERVICE_INTERFACE_H
