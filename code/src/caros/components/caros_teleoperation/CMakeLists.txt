cmake_minimum_required(VERSION 2.8.3)
project(caros_teleoperation)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}")

########################################################################
#### Make use of c++11
########################################################################
# See: http://answers.ros.org/question/173947/export-gcc-flags-though-catkin-cfg_extras/
# Should automatically use c++11 due to using caros_control
# Using hardcoded path-strings instead of a variable substitution for the use-c++11.cmake path-name, as catkin_lint doesn't properly do variable substitution (at least in this particular situation)
include("cmake/use-c++11.cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
caros_common
caros_common_msgs
caros_common_robwork
caros_control
caros_control_msgs
caros_sensor
caros_sensor_msgs
geometry_msgs
message_generation
rosconsole
roscpp
roslint
sensor_msgs
std_msgs)

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-runtime/references")
roslint_cpp()

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

################################################
## Declare ROS messages, services and actions ##
################################################
#add_message_files(
#  FILES
#
#)

#add_service_files(
#  FILES
#
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  caros_common
  caros_common_msgs
  caros_control
  caros_control_msgs
  caros_sensor
  caros_sensor_msgs
  geometry_msgs
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  DEPENDS ROBWORK
  )

include_directories(include
  ${ROBWORK_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS})

add_executable(${node_name} src/pose_teleoperate.cpp src/teleoperation.cpp)

target_link_libraries(${node_name}
  ${ROBWORK_LIBRARIES}
  ${ROBWORKHARDWARE_LIBRARIES}
  ${catkin_LIBRARIES})

add_dependencies(${node_name}
  caros_common
  caros_control
  )
