cmake_minimum_required(VERSION 2.8.3)
project(caros_control_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  caros_common_msgs
  geometry_msgs
  message_generation
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  GripperGripQ.srv
  GripperMoveQ.srv
  GripperSetForceQ.srv
  GripperSetVelocityQ.srv
  GripperStopMovement.srv
  SerialDeviceForceControlStart.srv
  SerialDeviceForceControlStop.srv
  SerialDeviceForceControlUpdate.srv
  SerialDeviceMoveLin.srv
  SerialDeviceMovePtp.srv
  SerialDeviceMovePtpT.srv
  SerialDeviceMoveServoQ.srv
  SerialDeviceMoveServoT.srv
  SerialDeviceMoveVelQ.srv
  SerialDeviceMoveVelT.srv
)

add_message_files(
  FILES
  GripperState.msg
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  caros_common_msgs
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS caros_common_msgs geometry_msgs message_runtime std_msgs
)
