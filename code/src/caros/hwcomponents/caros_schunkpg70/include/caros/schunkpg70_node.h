#ifndef CAROS_SCHUNKPG70_NODE_H
#define CAROS_SCHUNKPG70_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Constants.hpp>

#include <rwhw/schunkpg70/SchunkPG70.hpp>

#include <ros/ros.h>

#include <string>

#define MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY 0.1
#define MOVE_DISTANCE_STOPPED_THRESHOLD 0.01
#define MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING 2.0
#define MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING (5.0 * rw::math::Deg2Rad)

#define SUPPORTED_Q_LENGTH_FOR_SCHUNKPG70NODE 1

namespace caros
{
/**
 * @brief ROS node for controlling schunkpg70.
 */
class SchunkPG70Node : public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface
{
 public:
  //! constructor
  explicit SchunkPG70Node(const ros::NodeHandle& nodehandle);

  //! destructor
  virtual ~SchunkPG70Node();

  //! @copydoc caros::GripperServiceInterface::moveQ
  bool moveQ(const rw::math::Q& q);
  //! @copydoc caros::GripperServiceInterface::gripQ
  bool gripQ(const rw::math::Q& q);
  //! @copydoc caros::GripperServiceInterface::setForceQ
  bool setForceQ(const rw::math::Q& q);
  //! @copydoc caros::GripperServiceInterface::setVelocityQ
  bool setVelocityQ(const rw::math::Q& q);
  //! @copydoc caros::GripperServiceInterface::stopMovement
  bool stopMovement();

  /* TODO: Properly document the error codes */
  /* TODO: Consider better error codes for schunkpg70NODE_INTERNAL_ERROR */
  /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
  enum SCHUNKPG70NODE_ERRORCODE
  {
    SCHUNKPG70NODE_SCHUNKPG70_DEVICE_ALREADY_ACTIVE = 1,
    SCHUNKPG70NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL,
    SCHUNKPG70NODE_UNSUPPORTED_INTERFACE_TYPE,
    SCHUNKPG70NODE_SCHUNKPG70_DEVICE_CONNECT_FAILED,
    SCHUNKPG70NODE_INTERNAL_ERROR,
    SCHUNKPG70NODE_SCHUNKPG70_DEVICE_NO_CONNECTION,
    SCHUNKPG70NODE_NO_SCHUNKPG70_DEVICE,
    SCHUNKPG70NODE_UNSUPPORTED_Q_LENGTH
  };

 protected:
  // hooks implemented from CarosNodeServiceInterface base class
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  // Utility functions to configure and connect to the schunkpg70 device
  bool configureSchunkPG70Device();
  bool connectToSchunkPG70Device();

 private:
  bool isInWorkingCondition();
  bool supportedQSize(const rw::math::Q& q);

 private:
  ros::NodeHandle node_handle_;
  ros::Time last_loop_time_;
  rw::math::Q last_q_, target_q_;
  rwhw::SchunkPG70* schunkpg70_;
  /* Variables that are to be fetched from a ROS parameter server */
  std::string serial_device_;
};

}  // namespace caros

#endif  // CAROS_SCHUNKPG70_NODE_H
