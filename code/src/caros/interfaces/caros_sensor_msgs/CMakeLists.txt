cmake_minimum_required(VERSION 2.8.3)
project(caros_sensor_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  caros_common_msgs
  geometry_msgs
  message_generation
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

add_message_files(
  FILES
  ButtonSensorState.msg
  PoseSensorState.msg
  TactileArrayData.msg
  TactileArrayState.msg
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
