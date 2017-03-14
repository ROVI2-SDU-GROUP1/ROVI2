cmake_minimum_required(VERSION 2.8.3)
project(caros_camera)


########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")
set(SHOULD_BUILD_CAROS_CAMERA 1)

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
  camera_info_manager
  caros_common
  caros_common_msgs
  caros_common_robwork
  caros_control
  caros_control_msgs
  caros_sensor
  caros_sensor_msgs
  cv_bridge
  dynamic_reconfigure
  eigen_conversions
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  rosconsole
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL 1.8.0)

########################################################################
#### Dynamic Reconfigure config
########################################################################
generate_dynamic_reconfigure_options(
        cfg/Basler.cfg
        cfg/Ensenso.cfg
	    cfg/PointGrey.cfg
)

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-runtime/references")
roslint_cpp()


################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#FILES
#ServiceName.srv
#)

#generate_messages(
#DEPENDENCIES
#caros_common_msgs
#std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS caros_common caros_common_msgs caros_common_robwork caros_control caros_control_msgs caros_sensor caros_sensor_msgs cv_bridge dynamic_reconfigure message_runtime pcl_conversions pcl_ros  roscpp sensor_msgs std_msgs tf_conversions
DEPENDS
CFG_EXTRAS "use-c++11.cmake"
)

###############################################################################
# - Try to find Ensenso SDK (IDS-Imaging)
# Once done this will define
#  ENSENSO_FOUND - System has Ensenso SDK
#  ENSENSO_INCLUDE_DIRS - The Ensenso SDK include directories
#  ENSENSO_LIBRARIES - The libraries needed to use Ensenso SDK
#  ENSENSO_DEFINITIONS - Compiler switches required for using Ensenso SDK
# -----------------------

find_path(ENSENSO_INCLUDE_DIR nxLib.h
        HINTS ${ENSENSO_ABI_HINT}
        /opt/ensenso/development/c
        "$ENV{PROGRAMFILES}/Ensenso/development/c" "$ENV{PROGRAMW6432}/Ensenso/development/c"
        PATH_SUFFIXES include/)

find_library(ENSENSO_LIBRARY QUIET NAMES NxLib64 NxLib32 nxLib64 nxLib32
        HINTS ${ENSENSO_ABI_HINT}
        "$ENV{PROGRAMFILES}/Ensenso/development/c" "$ENV{PROGRAMW6432}/Ensenso/development/c"
        PATH_SUFFIXES lib/)

set(ENSENSO_LIBRARIES ${ENSENSO_LIBRARY})
set(ENSENSO_INCLUDE_DIRS ${ENSENSO_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ENSENSO_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ensenso DEFAULT_MSG
        ENSENSO_LIBRARY ENSENSO_INCLUDE_DIR)

mark_as_advanced(ENSENSO_INCLUDE_DIR ENSENSO_LIBRARY)

if(ENSENSO_FOUND)
  message(STATUS "Ensenso SDK found")
endif(ENSENSO_FOUND)

if(PCL_FOUND)   # true
  message(STATUS "PCL 1.8.0 found")
else() # false
  set(ENSENSO_FOUND FALSE)
  message(STATUS "Ensenso drivers found, but PCL 1.8.0 was not. caros_camera will not support Ensenso")
endif()

###################################
## PointGrey FlyCapture2 Library ##
###################################
find_library(POINTGREY_LIB flycapture)
include_directories(${POINTGREY_INCLUDE_DIR})

if(POINTGREY_LIB)
  message(STATUS "PointGrey Flycapture2 found!")
  set(POINTGREY_LIB_FOUND TRUE)
else()
  message(STATUS "PointGrey Flycapture2 was not found!")
  set(POINTGREY_LIB_FOUND FALSE)
endif()

###################################
## FireWire library and OpenCV   ##
###################################

find_library(DC1394_LIBRARIES dc1394)
set(DC1394_LIBRARIES ${DC1394_LIBRARIES})

if(DC1394_LIBRARIES)
  set(DC1394_FOUND TRUE)
else()
  set(DC1394_FOUND FALSE)
endif()

find_package(OpenCV REQUIRED core imgproc highgui)

###################################
## Pylon GigE Library            ##
###################################

set(PYLON_ENV_FOUND true)

# First check if PYLON_ROOT and GENICAM_ROOT_V2_3 environment variables are defined
if(NOT ENV{PYLON_ROOT})
  message(STATUS "Verify that Pylon has been installed, and that PYLON_ROOT and GENICAM_ROOT_V2_3 has been defined?")
  set(PYLON_ENV_FOUND false)
endif()
if(NOT ENV{GENICAM_ROOT_V2_3})
  message(STATUS "Verify that Pylon has been installed, and that PYLON_ROOT and GENICAM_ROOT_V2_3 has been defined?")
  set(PYLON_ENV_FOUND false)
endif()

if(PYLON_ENV_FOUND)
  set(PYLON_INCLUDE
        $ENV{PYLON_ROOT}/include/
        $ENV{GENICAM_ROOT_V2_3}/library/CPP/include/
        )

  set(PYLON_LIBS
        $ENV{PYLON_ROOT}/lib64/libpylonbase.so
        $ENV{PYLON_ROOT}/lib64/libpylonutility.so
        $ENV{PYLON_ROOT}/lib64/libXerces-C_gcc40_v2_7.so
        $ENV{PYLON_ROOT}/lib64/pylon/tl/pyloncamemu.so
        $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/libGCBase_gcc40_v2_3.so
        $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/libGenApi_gcc40_v2_3.so
        $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/libMathParser_gcc40_v2_3.so
        $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/libLog_gcc40_v2_3.so
        $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/liblog4cpp_gcc40_v2_3.so
        )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PYLON REQUIRED_VARS PYLON_LIBS PYLON_INCLUDE)

  if(PYLON_FOUND)
    message(STATUS "Pylon found")
    set( PYLON_INCLUDE_DIRS ${PYLON_INCLUDE} )
    set( PYLON_LIBRARIES ${PYLON_LIBS} )
  endif()
  mark_as_advanced(PYLON_INCLUDE PYLON_LIBS)

endif()

if (NOT PYLON_FOUND AND NOT ENSENSO_FOUND AND NOT DC1394_FOUND AND NOT POINTGREY_LIB_FOUND)
  message(WARNING "caros_camera requires camera drivers for at least one of the camera types!")
  set(SHOULD_BUILD_CAROS_CAMERA 0)
endif()

########################################################################
#### Build process
########################################################################
if (SHOULD_BUILD_CAROS_CAMERA)
  ########################################################################
  #### Build
  ########################################################################
  set(INCLUDES ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
  set(SOURCES src/camera.cpp
              src/camera_interface.cpp
              src/camera_main.cpp
              src/camera_manager.cpp )
  set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBS} )

  if(PYLON_FOUND)
    add_definitions(-DBASLER_SUPPORTED)
    set(INCLUDES ${INCLUDES} ${PYLON_INCLUDE_DIRS})
    set(SOURCES ${SOURCES} src/basler_camera.cpp)
    set(LIBS ${LIBS} ${PYLON_LIBRARIES})
  endif()
  if(ENSENSO_FOUND)
    add_definitions(-DENSENSO_SUPPORTED)
    set(INCLUDES ${PCL_INCLUDE_DIRS} ${INCLUDES} ${EIGEN3_INCLUDE_DIR})
    set(SOURCES ${SOURCES} src/ensenso_camera.cpp)
    set(LIBS ${LIBS} ${PCL_LIBRARIES})
  endif()
  if(POINTGREY_LIB_FOUND)
    add_definitions(-DBUMBLEBEE_SUPPORTED)
    set(SOURCES ${SOURCES} src/pointgrey_camera.cpp)
    set(LIBS ${LIBS} ${POINTGREY_LIB})
  endif()

  add_definitions(${PCL_DEFINITIONS})

  include_directories(
  include
  ${INCLUDES}
  )

  add_executable(${node_name}
  ${SOURCES}
  ) 

  target_link_libraries(${node_name}
  ${LIBS}
  )

  add_dependencies(${node_name}
  ${PROJECT_NAME}_gencfg
  caros_common
  )

  ########################################################################
  #### Testing
  ########################################################################
else()
  message(WARNING "Skipping ${PROJECT_NAME} due to missing libraries - see earlier warnings.")
endif()
