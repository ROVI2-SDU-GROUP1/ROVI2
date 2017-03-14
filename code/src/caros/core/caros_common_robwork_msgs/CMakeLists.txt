cmake_minimum_required(VERSION 2.8.3)
project(caros_common_robwork_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
add_message_files(
  FILES
  RwState.msg
  RwStateData.msg
)

add_service_files(
  FILES
  GetRwState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
