cmake_minimum_required(VERSION 2.8.3)
project(caros_robotiq)

########################################################################
# Local variables
########################################################################
set(robotiq2_node_name "${PROJECT_NAME}2_node")
set(robotiq3_node_name "${PROJECT_NAME}3_node")
set(SHOULD_BUILD_CAROS_ROBOTIQ 1)

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  caros_common
  caros_control
  message_generation
  roscpp
  roslint
  std_msgs
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
find_package(RobWorkHardware COMPONENTS robotiq PATHS "${RWHW_ROOT}")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires the robotiq component of RobWorkHardware!")
  set(SHOULD_BUILD_CAROS_ROBOTIQ 0)
endif()

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD_CAROS_ROBOTIQ)
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
    CATKIN_DEPENDS caros_common caros_control message_runtime roscpp std_msgs
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

  ############
  # Robotiq3
  ############
  add_executable(${robotiq3_node_name}
    src/main_robotiq3.cpp
    src/robotiq_node.cpp
    )

  target_link_libraries(${robotiq3_node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${catkin_LIBRARIES}
    )

  add_dependencies(${robotiq3_node_name}
    caros_common
    caros_control
    )
  
  ############
  # Robotiq2
  ############
  add_executable(${robotiq2_node_name}
    src/main_robotiq2.cpp
    src/robotiq_node.cpp
    )

  target_link_libraries(${robotiq2_node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${catkin_LIBRARIES}
    )

  add_dependencies(${robotiq2_node_name}
    caros_common
    caros_control
    )
  
  ########################################################################
  #### Testing
  ########################################################################
  if(CATKIN_ENABLE_TESTING)
    add_executable(simple_caros_robotiq3_demo src/test/simple_robotiq3_demo.cpp)
    target_link_libraries(simple_caros_robotiq3_demo ${catkin_LIBRARIES})
    
    add_executable(simple_caros_robotiq2_demo src/test/simple_robotiq2_demo.cpp)
    target_link_libraries(simple_caros_robotiq2_demo ${catkin_LIBRARIES})
  endif()
else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
