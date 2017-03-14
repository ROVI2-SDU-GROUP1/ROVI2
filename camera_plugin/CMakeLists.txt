cmake_minimum_required(VERSION 2.8.3)
project(camera_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  message_generation
#
)


#add_service_files(
#  FILES  
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")

  MESSAGE(STATUS "TESt æasldkfjæasdkjfælaskjdfælasdjf")
  MESSAGE(STATUS ${RW_ROOT})
  MESSAGE(STATUS ${catkin_INCLUDE_DIRS})


# Use OpenCV
#	#	SET(OpenCV_DIR ${OpenCV_ROOT})
	#ENDIF()
#FIND_PACKAGE(OpenCV REQUIRED)
#INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
#LINK_DIRECTORIES( ${OpenCV_LIB_DIR} )
#MESSAGE("-- OpenCV found: " ${OpenCV_DIR})
#MESSAGE("--- OpenCV include dir: " ${OpenCV_INCLUDE_DIRS})
#MESSAGE("--- OpenCV lib dir: " ${OpenCV_LIB_DIR})



#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
#The following is a workaround for the old version of RobWork. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

#Include default settings for constructing a robworkst dependent project
FIND_PACKAGE(
  RobWorkStudio
  QUIET
  HINTS
    "${RWSTUDIO_ROOT}")
#The following is a workaround for the old version of RobWorkSt. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORKSTUDIO_FOUND)
  MESSAGE(STATUS "Could not find RobWorkStudio with the new method. Trying the old one.")
  INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
ENDIF(NOT ROBWORKSTUDIO_FOUND)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${catkin_LIBRARIES})


FIND_PACKAGE(Qt5Widgets REQUIRED)

# if we want to use ui files add them here
SET(UIS_FILES src/CameraPlugin.ui )

QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT5_WRAP_CPP(MocSrcFiles src/CameraPlugin.hpp src/qtros.h
OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)
# Rcc the files:

#QT5_WRAP_CPP(sources $

#{moc-sources}
#OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

#QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
set(SrcFiles
	src/CameraPlugin.cpp
        src/qtros.cpp
	${UIS_OUT_H}
)


set(ROS_BUILD_STATIC_LIBS false)
set(ROS_BUILD_SHARED_LIBS true)

#target_link_libraries(RWSPlugin ${catkin_LIBRARIES})
add_library(camera_plugin ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
target_link_libraries(camera_plugin ${catkin_LIBRARIES})
#

## Declare a cpp library
# add_library(beginner_tutorials
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorials beginner_tutorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
