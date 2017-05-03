#ifndef CAROS_SDH_NODE_H
#define CAROS_SDH_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Constants.hpp>

#include <rwhw/sdh/SDHDriver.hpp>

#include <ros/ros.h>

#include <string>

#define MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY 0.1
#define MOVE_DISTANCE_STOPPED_THRESHOLD 0.01
#define MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING 2.0
#define MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING (5.0 * rw::math::Deg2Rad)

#define SUPPORTED_Q_LENGTH_FOR_SDHNODE 7

namespace caros
{
/**
 * @brief ROS node for controlling SDH.
 *
 * @note This node does not fully comply with the units specified in caros::GripperServiceInterface. Please see the
 * individual service callbacks for more information.
 */
class SDHNode : public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface
{
 public:
  //! constructor
  explicit SDHNode(const ros::NodeHandle& nodehandle);

  //! destructor
  virtual ~SDHNode();

  /**
   * @copydoc caros::GripperServiceInterface::moveQ
   */
  // Units: Radians
  bool moveQ(const rw::math::Q& q);
  /**
   * @copydoc caros::GripperServiceInterface::moveQ
   */
  // Units: Radians
  bool gripQ(const rw::math::Q& q);
  /**
   * @copydoc caros::GripperServiceInterface::setForceQ
   *
   * @note In contrast to the method inherited from (caros::GripperServiceInterface::setForceQ) the force is understood
   * as the electrical current, and (as of this writing) defaults to the unit Ampere.
   *
   * @note Units: Ampere
   */
  bool setForceQ(const rw::math::Q& q);
  /**
   * @copydoc caros::GripperServiceInterface::setVelocityQ
   */
  // Units: Radians/sec
  bool setVelocityQ(const rw::math::Q& q);
  //! @copydoc caros::GripperServiceInterface::stopMovement
  bool stopMovement();

  /* TODO: Properly document the error codes */
  /* TODO: Consider better error codes for SDHNODE_INTERNAL_ERROR */
  /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
  enum SDHNODE_ERRORCODE
  {
    SDHNODE_SDH_DEVICE_ALREADY_ACTIVE = 1,
    SDHNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL,
    SDHNODE_UNSUPPORTED_INTERFACE_TYPE,
    SDHNODE_SDH_DEVICE_CONNECT_FAILED,
    SDHNODE_INTERNAL_ERROR,
    SDHNODE_SDH_DEVICE_NO_CONNECTION,
    SDHNODE_NO_SDH_DEVICE,
    SDHNODE_UNSUPPORTED_Q_LENGTH
  };

 protected:
  // hooks implemented from CarosNodeServiceInterface base class
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  // Utility functions to configure and connect to the SDH device
  bool configureSDHDevice();
  bool connectToSDHDevice();

 private:
  bool isInWorkingCondition();
  bool supportedQSize(const rw::math::Q& q);

 private:
  ros::NodeHandle node_handle_;

  enum SDH_STATE
  {
    WAIT,
    MOVE_WAIT
  };
  SDH_STATE current_state_, next_state_;

  rw::common::Timer move_start_timer_, vel_update_timer_;
  rw::math::Q move_q_, vel_q_, current_q_, last_q_;

  rwhw::SDHDriver* sdh_;

  /* Variables that are to be fetched from a ROS parameter server */
  std::string interface_type_;
  std::string rs232_device_;
  int rs232_port_;
  int rs232_baudrate_;
  double rs232_timeout_;
  std::string can_device_;
  int can_baudrate_;
  double can_timeout_;
};

}  // namespace caros

#endif  // CAROS_SDH_NODE_H
