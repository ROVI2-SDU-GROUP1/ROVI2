cmake_minimum_required(VERSION 2.8.3)
project(caros_netft)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")
set(SHOULD_BUILD_CAROS_NETFT 1)

########################################################################
#### Make use of c++11
########################################################################
# See: http://answers.ros.org/question/173947/export-gcc-flags-though-catkin-cfg_extras/
# Should automatically use c++11 due to using caros_control
# Using hardcoded path-strings instead of a variable substitution for the use-c++11.cmake path-name, as catkin_lint doesn't properly do variable substitution (at least in this particular situation)
include("cmake/use-c++11.cmake")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  caros_common
  caros_common_msgs
  caros_common_robwork
  caros_sensor
  caros_sensor_msgs
  geometry_msgs
  message_generation
  roscpp
  roslint
  std_msgs
  )

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-build/namespace")
roslint_cpp()

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

########################################################################
#### RobWorkHardware
########################################################################
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware COMPONENTS netft PATHS "${RWHW_ROOT}")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires the netft component of RobWorkHardware!")
  set(SHOULD_BUILD_CAROS_NETFT 0)
endif()

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD_CAROS_NETFT)
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  #add_service_files(
  #  FILES
  #)

  #generate_messages(
  #  DEPENDENCIES
  #)

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS caros_common caros_common_msgs caros_common_robwork caros_sensor caros_sensor_msgs geometry_msgs message_runtime roscpp std_msgs
    DEPENDS ROBWORK ROBWORKHARDWARE
    )

  ########################################################################
  #### Build
  ########################################################################
  include_directories(
    include
    ${ROBWORK_INCLUDE_DIR}
    ${ROBWORKHARDWARE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  add_executable(${node_name}
    src/main_netft.cpp
    src/netft_node.cpp
    )

  target_link_libraries(${node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${catkin_LIBRARIES}
    )

  add_dependencies(${node_name}
    caros_common
    caros_common_robwork
    caros_control
    )

  ########################################################################
  #### Testing
  ########################################################################

else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
