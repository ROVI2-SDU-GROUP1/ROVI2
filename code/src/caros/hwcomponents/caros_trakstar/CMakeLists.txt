cmake_minimum_required(VERSION 2.8.3)
project(caros_trakstar)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")
set(library_name "${PROJECT_NAME}_lib")
set(SHOULD_BUILD_CAROS_TRAKSTAR 1)

########################################################################
#### Catkin Packages
########################################################################
find_package(
  catkin REQUIRED COMPONENTS
  caros_common
  caros_sensor
  message_generation
  nodelet
  rosconsole
  roscpp
  roslint
  std_msgs
  )

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-build/include")
roslint_cpp()

########################################################################
#### RobWork and RobWorkHardware
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(RobWorkHardware COMPONENTS trakstar PATHS "${RWHW_ROOT}")

if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Requires the trakstar component of RobWorkHardware!")
  set(SHOULD_BUILD_CAROS_TRAKSTAR 0)
endif()

########################################################################
#### TRAKSTAR
########################################################################
find_package(TrakStar)
if (NOT TRAKSTAR_FOUND)
  message(WARNING "Requires the TrakStar library to be installed.")
  set(SHOULD_BUILD_CAROS_TRAKSTAR 0)
else()
  # TrakStar stuff (6D Pose sensors)
  add_definitions( -DLINUX )
  include_directories( /opt/3DGuidance.Rev.E.64/3DGuidanceAPI)
  set(TRAKSTAR_LIBRARIES ATC3DGlib64)
  #LINK_DIRECTORIES(/opt/3DGuidance.Rev.E.64/3DGuidanceAPI )
  find_library(3DGuidanceAPI HINT /opt/3DGuidance.Rev.E.64/)
endif()

########################################################################
#### Build process
########################################################################
if(SHOULD_BUILD_CAROS_TRAKSTAR)
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  #add_service_files(
  #  FILES
  #)

  #generate_messages(
  #  DEPENDENCIES
  #  std_msgs
  #  caros_common
  #)

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${library_name}
    CATKIN_DEPENDS caros_common caros_sensor message_runtime nodelet roscpp std_msgs
    DEPENDS ROBWORK
    )

  ########################################################################
  #### Build
  ########################################################################
  include_directories(
    include
    ${ROBWORK_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    )

  add_library(${library_name}
    src/trakstar.cpp
    src/trakstar_node.cpp
    )

  add_executable(${node_name} src/main.cpp)
  target_link_libraries(${node_name} ${library_name} ${ROBWORK_LIBRARIES} ${TRAKSTAR_LIBRARIES} ${catkin_LIBRARIES})

  ########################################################################
  #### Testing
  ########################################################################

else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
