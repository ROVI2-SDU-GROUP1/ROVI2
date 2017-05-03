\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-generate-toc again -->
**Table of Contents**

- [caros_control](#caroscontrol)
- [Interfaces](#interfaces)
    - [Gripper Service Interface](#gripper-service-interface)
    - [Serial Device Service Interface](#serial-device-service-interface)

<!-- markdown-toc end -->

# caros_control #
This component contains interfaces for controlling various types of devices. Examples hereof are gripper or a robot arm. See [classes](annotated.html) for a list of the available control service interfaces and corresponding proxies.

# Interfaces #
All interfaces in here are abstract and parts need to be added by the user for the specific device. All proxies are useable directly

## Gripper Service Interface ##
The @ref caros::GripperServiceInterface represents a simple gripper interface.

All topics and services of @ref caros::GripperServiceInterface are using the basic units meters, radian, Newton and Newtonmeter. If you implement a node using this interface please stick to this (if absolutely not possible please document extensively, e.g., do not copy apidoc but replace). When using the corresponding proxy to talk to a device please look at the device node's documentation first to make sure this is true.

The @ref caros::GripperServiceInterface has one output topic:
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| gripper_state  | @ref caros_control_msgs::GripperState | State of the gripper (joint pos, joint vel, ...)|

In addition the following services are provided.

| Service | Type | Description |
| ------- | :------: | :-------------: |
| move_q  | @ref caros_control_msgs::GripperMoveQ | See @ref caros::GripperServiceInterface::moveQ. |
| grip_q  | @ref caros_control_msgs::GripperGripQ | See @ref caros::GripperServiceInterface::gripQ. |
| set_force_q  | @ref caros_control_msgs::GripperSetForceQ | See @ref caros::GripperServiceInterface::setForceQ. |
| set_velocity_q  | @ref caros_control_msgs::GripperSetVelocityQ | See @ref caros::GripperServiceInterface::setVelocityQ. |
| stop_movement  | @ref caros_control_msgs::GripperStopMovement | See @ref caros::GripperServiceInterface::stopMovement. |

Clients can make use of the @ref caros::GripperSIProxy to communicate with nodes implementing this service interface.

## Serial Device Service Interface ##
The @ref caros::SerialDeviceServiceInterface represents an interface for a serial device (e.g., an articulated robot arm).

All topics and services of @ref caros::SerialDeviceServiceInterface are using the basic units meters, radian, Newton and Newtonmeter. If you implement a node using this interface please stick to this (if absolutely not possible please document extensively, e.g., do not copy apidoc but replace). When using the corresponding proxy to talk to a device please look at the device node's documentation first to make sure this is true.

The @ref caros::SerialDeviceServiceInterface has one output topic:
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| robot_state  | @ref caros_control_msgs::RobotState | State of the robot (joint pos, joint vel, ...)|

In addition the following services are provided.

| Service | Type | Description |
| ------- | :------: | :-------------: |
| move_lin | @ref caros_control_msgs::SerialDeviceMoveLin | See @ref caros::SerialDeviceServiceInterface::moveLin. |
| move_ptp | @ref caros_control_msgs::SerialDeviceMovePtp | See @ref caros::SerialDeviceServiceInterface::movePtp.  |
| move_ptp_t | @ref caros_control_msgs::SerialDeviceMovePtpT | See @ref caros::SerialDeviceServiceInterface::movePtpT. |
| move_vel_q | @ref caros_control_msgs::SerialDeviceMoveVelQ | See @ref caros::SerialDeviceServiceInterface::moveVelQ. |
| move_vel_t | @ref caros_control_msgs::SerialDeviceMoveVelT | See @ref caros::SerialDeviceServiceInterface::moveVelT. |
| move_servo_q | @ref caros_control_msgs::SerialDeviceMoveServoQ | See @ref caros::SerialDeviceServiceInterface::moveServoQ. |
| move_servo_t | @ref caros_control_msgs::SerialDeviceMoveServoT | See @ref caros::SerialDeviceServiceInterface::moveServoT. |
| move_stop | @ref caros_common_msgs::EmptySrv | See @ref caros::SerialDeviceServiceInterface::moveStop. |

Clients can make use of the @ref caros::SerialDeviceSIProxy to communicate with nodes implementing this service interface.
