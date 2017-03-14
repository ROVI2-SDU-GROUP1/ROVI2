#ifndef CAROS_ROBOTIQ_NODE_H
#define CAROS_ROBOTIQ_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rwhw/robotiq/Robotiq.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <stdexcept>
#include <string>

namespace caros
{
/**
 * @brief Ros node for controlling Robotiq-3 and Robotiq-2 grippers.
 * The node inherits from caros::GripperServiceInterface and therefore provides the corresponding
 * topic and services. THIS NODE DOES NOT COMPLY TO THE UNITS SPECIFIED IN caros::GripperServiceInterface.
 * Please see the service callbacks to get more information.
 */
class RobotiqNode : public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface
{
 public:
  enum class HandType
  {
    ROBOTIQ3,
    ROBOTIQ2
  };

 public:
  /**
   * \brief Constructor
   * \param [in] hand_type Specify what type of Robotiq hand to use
   * \throws std::invalid_argument if the chosen HandType is not supported
   */
  RobotiqNode(const ros::NodeHandle& node_handle, const HandType hand_type);

  //! destructor
  virtual ~RobotiqNode();

  /**
   * @copydoc caros::GripperServiceInterface::moveQ
   * In contrast to the method inherited from (caros::GripperServiceInterface::moveQ) the robotiq2 and robotiq3 device actually do not take
   * joint angles in radians but rather use an integer between 0 and 255 for joint angle values.
   * See the hand manuals for the meaning of the values. Otherwise see caros::GripperServiceInterface::moveQ.
  **/
  bool moveQ(const rw::math::Q& q);

  /**
   * @copydoc caros::GripperServiceInterface::gripQ
   * In contrast to the method inherited from (caros::GripperServiceInterface::gripQ) the robotiq2 and robotiq3 device actually do not take
   * joint angles in radians but rather use an integer between 0 and 255 for joint angle values.
   * See the hand manuals for the meaning of the values. Otherwise see caros::GripperServiceInterface::gripQ.
  **/
  bool gripQ(const rw::math::Q& q);

  /**
     * @copydoc caros::GripperServiceInterface::setForceQ
     * In contrast to the method inherited from (caros::GripperServiceInterface::setForceQ) the robotiq2 and robotiq3 device actually do not take
     * forces in Newton but rather use an integer between 0 and 255 for joint force/torque values.
     * See the hand manuals for the meaning of the values. Otherwise see caros::GripperServiceInterface::setForceQ.
    **/
  bool setForceQ(const rw::math::Q& q);

  /**
      * @copydoc caros::GripperServiceInterface::setVelocityQ
      * In contrast to the method inherited from (caros::GripperServiceInterface::setVelocityQ) the robotiq2 and robotiq3 device actually do not take
      * velocities in radians / sec but rather use an integer between 0 and 255 for joint angle speeds.
      * See the hand manuals for the meaning of the values. Otherwise see caros::GripperServiceInterface::setVelocityQ.
     **/
  bool setVelocityQ(const rw::math::Q& q);

  //! @copydoc caros::GripperServiceInterface::stopMovement
  bool stopMovement(void);

  /* TODO: Properly document the error codes */
  /* TODO: Consider better error codes for ROBOTIQNODE_INTERNAL_ERROR */
  /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
  enum ROBOTIQNODE_ERRORCODE
  {
    ROBOTIQNODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE = 1,
    ROBOTIQNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL,
    ROBOTIQNODE_ROBOTIQ_DEVICE_CONNECT_FAILED,
    ROBOTIQNODE_INTERNAL_ERROR,
    ROBOTIQNODE_ROBOTIQ_DEVICE_NO_CONNECTION,
    ROBOTIQNODE_NO_ROBOTIQ_DEVICE,
    ROBOTIQNODE_UNSUPPORTED_Q_LENGTH
  };

 protected:
  // hooks implemented from CarosNodeServiceInterface base class
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  // Utility functions to configure and connect to the Robotiq device
  bool configureRobotiqDevice();
  bool connectToRobotiqDevice();

  // convenience functions
  bool isInWorkingCondition();

 protected:
  ros::Time last_loop_time_;
  rw::math::Q last_q_;
  std::string ip_;
  int port_;

 private:
  rw::common::Ptr<rwhw::Robotiq> robotiq_;
  ros::NodeHandle node_handle_;
  HandType hand_type_;
};
}  // namespace caros

#endif  // CAROS_ROBOTIQ_NODE_H
