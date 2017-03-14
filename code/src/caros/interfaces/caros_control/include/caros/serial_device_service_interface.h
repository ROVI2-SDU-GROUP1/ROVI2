#ifndef CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H
#define CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H

#include <caros_control_msgs/RobotState.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePtp.h>
#include <caros_control_msgs/SerialDeviceMovePtpT.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveServoQ.h>
#include <caros_control_msgs/SerialDeviceMoveServoT.h>
#include <caros_common_msgs/EmptySrv.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#include <string>
#include <tuple>

/* Always publish the latest serial device state */
#define SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE 1
#define SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_serial_device_service_interface"

namespace caros
{
/**
 * @brief This is the serial device interface. It defines the (minimum) interface that a joint based robotic device
 *needs to implement.
 *
 * All interfaces use meters, radians, Newton and Newtonmeter as base units.
 * (linear joints in meters, rotary joints in radians.)
 * If a device does not support this please be very explicit in the documentation of your node about this.
 *
 * In ROS the namespace of the node is used and it is important that not two SerialDeviceServiceInterface are running in
 *the
 *same namespace.
 */
class SerialDeviceServiceInterface
{
 public:
  explicit SerialDeviceServiceInterface(ros::NodeHandle nodehandle);

  virtual ~SerialDeviceServiceInterface();

  typedef std::vector<std::tuple<const rw::math::Transform3D<>, const float>> TransformAndSpeedContainer_t;
  typedef std::vector<std::tuple<const rw::math::Q, const float>> QAndSpeedContainer_t;

  /* TODO: Not supporting blends at the moment! */
  //! @brief move robot on a linear Cartesian path (in meters)
  virtual bool moveLin(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from c-space point to c-space point (in radians for revolute joints / meters for linear)
  virtual bool movePtp(const QAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from Cartesian point to Cartesian point (in meters) using a pose as target (requires invkin)
  virtual bool movePtpT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot in a servoing fashion specifying joint velocity targets (in radians/sec for revolute joints /
  //! meters/sec for linear)
  virtual bool moveVelQ(const rw::math::Q& q_vel) = 0;
  //! @brief move robot in a servoing fashion specifying a velocity screw in tool coordinates (in meters/sec and
  //! radians/sec)
  virtual bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel) = 0;
  /**
   * @brief move robot in a servoing fashion specifying a joint configuration (in radians for revolute joints and meters
   * for linear joints)
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified
   * target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoQ(const QAndSpeedContainer_t& targets) = 0;
  /**
   * @brief move robot in a servoing fashion specifying a pose (in meters)
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified
   * target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief hard stop the robot
  virtual bool moveStop() = 0;

 protected:
  /**
   * @brief Initialise this interface, such that the ROS services and publishers become available.
   * @returns a boolean indicating whether all the ROS services and publishers were successfully made available.
   */
  bool configureInterface();

  //! publish robot state
  void publishState(const caros_control_msgs::RobotState& state);

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  SerialDeviceServiceInterface();

  bool initService();

  bool moveLinHandle(caros_control_msgs::SerialDeviceMoveLin::Request& request,
                     caros_control_msgs::SerialDeviceMoveLin::Response& response);

  bool movePtpHandle(caros_control_msgs::SerialDeviceMovePtp::Request& request,
                     caros_control_msgs::SerialDeviceMovePtp::Response& response);

  bool movePtpTHandle(caros_control_msgs::SerialDeviceMovePtpT::Request& request,
                      caros_control_msgs::SerialDeviceMovePtpT::Response& response);

  bool moveVelQHandle(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
                      caros_control_msgs::SerialDeviceMoveVelQ::Response& response);

  bool moveVelTHandle(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
                      caros_control_msgs::SerialDeviceMoveVelT::Response& response);

  bool moveServoQHandle(caros_control_msgs::SerialDeviceMoveServoQ::Request& request,
                        caros_control_msgs::SerialDeviceMoveServoQ::Response& response);

  bool moveServoTHandle(caros_control_msgs::SerialDeviceMoveServoT::Request& request,
                        caros_control_msgs::SerialDeviceMoveServoT::Response& response);

  bool moveStopHandle(caros_common_msgs::EmptySrv::Request& request, caros_common_msgs::EmptySrv::Response& response);

 protected:
  std::string service_name_;
  ros::NodeHandle nodehandle_;

  ros::Publisher device_state_publisher_;

  ros::ServiceServer srv_move_lin_;
  ros::ServiceServer srv_move_ptp_;
  ros::ServiceServer srv_move_ptp_t_;
  ros::ServiceServer srv_move_vel_q_;
  ros::ServiceServer srv_move_vel_t_;
  ros::ServiceServer srv_move_servo_q_;
  ros::ServiceServer srv_move_servo_t_;

  ros::ServiceServer srv_move_stop_;
};
}  // namespace caros

#endif  // CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H
