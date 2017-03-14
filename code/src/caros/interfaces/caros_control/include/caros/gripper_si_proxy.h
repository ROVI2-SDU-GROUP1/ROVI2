#ifndef CAROS_GRIPPER_SI_PROXY_H
#define CAROS_GRIPPER_SI_PROXY_H

#include <caros/caros_service_client.h>
#include <caros_control_msgs/GripperState.h>

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <string>

namespace caros
{
/**
 * @brief This class implements a C++ proxy to control and read data from a GripperServiceInterface.
 */
class GripperSIProxy
{
 public:
  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the CAROS gripper node
   * @param[in] usePersistentConnections Define usage of persistent connections
   */
  GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname, const bool use_persistent_connections = true);

  //! Destructor
  virtual ~GripperSIProxy();

  /**
   * @brief Move gripper to configuration
   * @param[in] q The target configuration
   * @returns a boolean indicating if the gripper accepted the command.
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   * gripper is not fully working, or the gripper has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the gripper.
   */
  bool moveQ(const rw::math::Q& q);

  /**
   * @brief Grasp with the given configuration
   * @param[in] q The target configuration to use for the grasp
   * @returns a boolean indicating if the gripper accepted the command.
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   *gripper is not fully working, or the gripper has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the gripper.
   */
  bool gripQ(const rw::math::Q& q);

  /**
   * @brief Set the force configuration for the gripper
   * @param[in] q The force configuration
   * @returns a boolean indicating if the gripper successfully completed the command.
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   * gripper is not fully working, or the gripper has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the gripper.
   */
  bool setForceQ(const rw::math::Q& q);

  /**
   * @brief Set the velocity configuration for the gripper
   * @param[in] q The velocity configuration
   * @returns a boolean indicating if the gripper successfully completed the command.
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   * gripper is not fully working, or the gripper has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the gripper.
   */
  bool setVelocityQ(const rw::math::Q& q);

  /**
   * @brief Stop the gripper's movement(s)
   * @returns a boolean indicating if the gripper accepted the command.
   * @throws UnavailableService when the command is currently unavailable. This indicates that the connection to the
   * gripper is not fully working, or the gripper has not announced this service yet.
   * @throws BadServiceCall when an error happened while communicating with the gripper.
   */
  bool stopMovement();

  /**
   * @brief Close established (persistent) connections.
   *
   * @note Is mainly intended for debug purposes, to verify that the reconnect functionality is working as intended.
   */
  void closePersistentConnections();

  /**
   * @brief Get the last reported joint configuration \b Q from the gripper
   * @returns The reported joint configuration of the gripper
   */
  rw::math::Q getQ();

  /**
   * @brief Get the last reported velocities from the gripper
   * @returns The reported velocities of the gripper
   */
  rw::math::Q getQd();

  /**
   * @brief Get the last reported forces from the gripper
   * @returns The reported forces of the gripper
   *
   * @note Currently it is most likely just the (electrical) currents that will be returned, as there is no implemented
   *conversion to get the actual forces instead
   */
  rw::math::Q getForce();

  /**
   * @brief Get the timestamp that is associated with the last reported gripper state
   * @returns The timestamp of the last reported state of the gripper
   */
  ros::Time getTimeStamp();

 protected:
  ros::NodeHandle nodehandle_;
  bool use_persistent_connections_;
  std::string ros_namespace_;

  // services
  caros::CarosServiceClient srv_move_q_;
  caros::CarosServiceClient srv_grip_q_;
  caros::CarosServiceClient srv_set_force_q_;
  caros::CarosServiceClient srv_set_velocity_q_;
  caros::CarosServiceClient srv_stop_movement_;

  // states
  void handleGripperState(const caros_control_msgs::GripperState& state);
  ros::Subscriber sub_gripper_state_;
  // pSV_ is "protectStateVariable", but removed synchronisation until a proper threading mechanism is being
  // implemented within the SIProxy.
  caros_control_msgs::GripperState psv_gripper_state_;
};
}  // namespace caros

#endif  // CAROS_GRIPPER_SI_PROXY_H
