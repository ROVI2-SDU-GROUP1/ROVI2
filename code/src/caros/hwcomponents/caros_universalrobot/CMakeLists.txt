cmake_minimum_required(VERSION 2.8.3)
project(caros_universalrobot)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")
set(SHOULD_BUILD_CAROS_UNIVERSALROBOT 1)

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
  caros_control
  caros_control_msgs
  geometry_msgs
  message_generation
  rosconsole
  roscpp
  roslint
  std_msgs
)

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

########################################################################
#### RobWorkHardware
########################################################################
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware COMPONENTS universalrobots PATHS "${RWHW_ROOT}")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires the universalrobots component of RobWorkHardware!")
  set(SHOULD_BUILD_CAROS_UNIVERSALROBOT 0)
endif()

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD_CAROS_UNIVERSALROBOT)
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  add_service_files(
    FILES
    UrServiceEmpty.srv
    UrServiceForceModeStart.srv
    UrServiceForceModeStop.srv
    UrServiceForceModeUpdate.srv
    UrServicePayload.srv
    UrServiceServoQ.srv
    UrServiceServoT.srv
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
    INCLUDE_DIRS include
    CATKIN_DEPENDS caros_common caros_common_msgs caros_common_robwork caros_control caros_control_msgs geometry_msgs message_runtime roscpp std_msgs
    DEPENDS ROBWORK ROBWORKHARDWARE
    CFG_EXTRAS "use-c++11.cmake"
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
    src/universal_robots.cpp
    src/ur_main.cpp
    src/ur_service_interface.cpp
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
  if(CATKIN_ENABLE_TESTING)
    add_executable(simple_${PROJECT_NAME}_demo_using_move_servo_q src/test/simple_demo_using_move_servo_q.cpp)
    target_link_libraries(simple_${PROJECT_NAME}_demo_using_move_servo_q ${catkin_LIBRARIES})

    add_executable(simple_${PROJECT_NAME}_demo_using_move_ptp src/test/simple_demo_using_move_ptp.cpp)
    target_link_libraries(simple_${PROJECT_NAME}_demo_using_move_ptp ${catkin_LIBRARIES})
  endif()
else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
