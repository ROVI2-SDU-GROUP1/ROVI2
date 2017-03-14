cmake_minimum_required(VERSION 2.8.3)
project(caros_sdh)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")
set(SHOULD_BUILD_CAROS_SDH 1)

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  caros_common
  caros_control
  rosconsole
  roscpp
  roslint
)

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
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
find_package(RobWorkHardware COMPONENTS sdh serialport PATHS "${RWHW_ROOT}")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires both SDH and Serialport components of RobWorkHardware!")
  set(SHOULD_BUILD_CAROS_SDH 0)
endif()

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD_CAROS_SDH)
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  # add_service_files(
  #   FILES
  # )

  # add_message_files(
  #   FILES
  # )

  # generate_messages(
  #   DEPENDENCIES
  #   std_msgs
  # )

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    # CATKIN_DEPENDS:
    # - rosconsole is not used in the header files, so should not be added to the list here.
    CATKIN_DEPENDS caros_common caros_control roscpp
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
    src/sdh_main.cpp
    src/sdh_node.cpp
    )

  target_link_libraries(${node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${catkin_LIBRARIES}
    )

  add_dependencies(${node_name}
    caros_common
    caros_control
    )

  ########################################################################
  #### Testing
  ########################################################################

else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
