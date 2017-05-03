#ifndef CAROS_GRIPPER_SERVICE_INTERFACE_H
#define CAROS_GRIPPER_SERVICE_INTERFACE_H

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <caros_control_msgs/GripperMoveQ.h>
#include <caros_control_msgs/GripperGripQ.h>
#include <caros_control_msgs/GripperSetForceQ.h>
#include <caros_control_msgs/GripperSetVelocityQ.h>
#include <caros_control_msgs/GripperStopMovement.h>

#include <string>

/* Always publish the latest gripper state */
#define GRIPPER_STATE_PUBLISHER_QUEUE_SIZE 1
#define GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE "caros_gripper_service_interface"

namespace caros
{
/**
 * @brief This is the gripper interface. It defines the
 * minimum interface that a configuration based robotic grasping device needs
 * to implement.
 *
 * All interfaces use meters, radians, Newton and Newtonmeter as base units.
 * (linear joints in meters, rotary joints in radians.)
 * If a device does not support this please be very explicit in the documentation
 * of your node about this.
 *
 * In ROS the namespace of the node is used and it is important that
 * not two GripperServiceInterfaces are running in the same namespace.
 *
 */
class GripperServiceInterface
{
 public:
  /**
   * @brief constructor.
   * @param[in] nodehandle the nodehandle to use for services.
   */
  explicit GripperServiceInterface(const ros::NodeHandle& nodehandle);

  /**
   * @brief virtual destructor
   */
  virtual ~GripperServiceInterface();

  /**
   * @brief signal the gripper to move into a specific configuration Q.
   * After moving to Q the gripper will potentially switch off the motor current.
   * @param q
   * @return
   */
  virtual bool moveQ(const rw::math::Q& q) = 0;

  /**
   * @brief signal the gripper to move into a specific configuration Q.
   * After moving to Q the gripper will not switch off the motor current.
   * The gripper will not show an error in its state if the configuration Q can not be reached.
   * @param q
   * @return
   */
  virtual bool gripQ(const rw::math::Q& q) = 0;

  /**
   * @brief set the desired force configuration that the gripper should use.
   * @param q
   * @return
   */
  virtual bool setForceQ(const rw::math::Q& q) = 0;

  /**
   * @brief set the desired velocity configuration that the gripper should use.
   * @param q
   * @return
   */
  virtual bool setVelocityQ(const rw::math::Q& q) = 0;

  /**
   * @brief signal the gripper to stop all its movements.
   * It should not power down the gripper and/or disconnect from the gripper.
   * @return
   */
  virtual bool stopMovement(void) = 0;

 protected:
  /**
   * @brief Initialise this interface, such that the ROS services and publishers become available.
   * @returns a boolean indicating whether all the ROS services and publishers were successfully made available.
   */
  bool configureInterface();

  /**
   * @brief publish the state of the gripper. Uses GripperState messages
   * @param[in] q joint configuration
   * @param[in] dq joint velocity
   * @param[in] joint_force joint force
   * @param[in] is_moving is moving
   * @param[in] is_blocked is blocked
   * @param[in] is_stopped is stopped
   * @param[in] is_e_stopped is emergency stopped
   */
  void publishState(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& joint_force, bool is_moving,
                    bool is_blocked, bool is_stopped, bool is_e_stopped);

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  GripperServiceInterface();

  /**
   * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the
   * callbacks.
   * This should not be called directly.
   */
  bool initService();

  /* - these functions should be grouped together in the documentation t shortly describe that these are converting from
   * ROS types to e.g. RobWork types according to what the interface expects - */
  bool moveQHandle(caros_control_msgs::GripperMoveQ::Request& request,
                   caros_control_msgs::GripperMoveQ::Response& response);

  bool gripQHandle(caros_control_msgs::GripperGripQ::Request& request,
                   caros_control_msgs::GripperGripQ::Response& response);

  bool setForceQHandle(caros_control_msgs::GripperSetForceQ::Request& request,
                       caros_control_msgs::GripperSetForceQ::Response& response);

  bool setVelocityQHandle(caros_control_msgs::GripperSetVelocityQ::Request& request,
                          caros_control_msgs::GripperSetVelocityQ::Response& response);

  bool stopMovementHandle(caros_control_msgs::GripperStopMovement::Request& request,
                          caros_control_msgs::GripperStopMovement::Response& response);

 private:
  ros::NodeHandle nodehandle_;

  ros::Publisher gripper_state_publisher_;

  ros::ServiceServer srv_move_q_;
  ros::ServiceServer srv_grip_q_;
  ros::ServiceServer srv_set_force_q_;
  ros::ServiceServer srv_set_velocity_q_;
  ros::ServiceServer srv_stop_movement_;

  /************************************************************************
   * Notes:
   * If it is required to know whether this object is configured/initialised then a variable/state needs to be
   *implemented. Furthermore extra care should be taken to make sure that this object is not left in an "undefined"
   *state. This would happen when some of the ros services or publishers fails to be initialised or shutdown properly.
   ************************************************************************/
};
}  // namespace caros

#endif  // CAROS_GRIPPER_SERVICE_INTERFACE_H
