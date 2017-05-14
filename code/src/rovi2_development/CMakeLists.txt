cmake_minimum_required(VERSION 3.0.0)
project(rovi2_development)


#find_package(Qt4)
#include(${QT_USE_FILE})
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  message_generation
)
add_definitions(-DSCENE_FILE="${CMAKE_BINARY_DIR}/../../WorkCell_scenes/WorkStation_3/WC3_Scene.wc.xml")

catkin_python_setup()
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV)

 add_message_files(
   FILES
   Trajectory3D.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

########## RobWork ############
# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
#SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})

# Use RobWork
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( SYSTEM ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Use RobWorkStudio
#MESSAGE("-- RobWorkStudio root: " ${RWSTUDIO_ROOT})
#SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
#FIND_PACKAGE(RobWorkStudio REQUIRED)
#INCLUDE_DIRECTORIES( ${INCLUDE_DIRECTORIES} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
#LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

catkin_package(CATKIN_DEPENDS message_runtime)

# Define path to calibration files used in stereo node
add_definitions(-DCALIBRATION_DIR="${CMAKE_SOURCE_DIR}/rovi2_development/calibrations/")


#add_subdirectory("deps/yaml-cpp/")
#include_directories(SYSTEM "deps/yaml-cpp/include/")

#We don't enable compilerwarnings in the headers, because some of the guys who wrote them are idiots...
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include_directories("headers")

#Add NAZI compiler arguments..
set(CMAKE_CXX_FLAGS "-std=c++11 \
 -O2 \
 -Wall \
 -Wextra \
 -Wshadow \
 -Wcast-qual \
 -Wpointer-arith \
 -Wno-error=unused-function \
 -Wno-error=unused-variable \
 -Wno-error=unused-parameter \
 -ffunction-sections \
 -fdata-sections \
 -g" )

add_definitions(-DSCENE_FILE="${CMAKE_BINARY_DIR}/../../WorkCell_scenes/WorkStation_3/WC3_Scene.wc.xml")

option(YAML_CPP_BUILD_TOOLS "Enable testing and parse tools" OFF)

# Bumblebee node
add_executable(opencv_node src/opencv_node.cpp)
set_target_properties(opencv_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(opencv_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(opencv_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

# Image detector node
add_executable(image_detector src/image_detector_node.cpp src/HsvAdjust.cpp)
add_dependencies(image_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_detector ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)

# YAML library
add_library(YAML SHARED src/YAMLCalibration/YAMLCalibration.cpp)
target_link_libraries(YAML ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)

# Frame transformation test using RobWork
add_executable(frame_trans src/frame_trans.cpp)
add_dependencies(frame_trans ${catkin_EXPORTED_TARGETS})
target_link_libraries(frame_trans ${ROBWORK_LIBRARIES} YAML)


# Hand to eye
add_executable(hand_to_eye src/hand_to_eye_calibration_node.cpp src/Hand_to_eye.cpp)
add_dependencies(hand_to_eye ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_to_eye ${OpenCV_LIBS} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} YAML)

# Stereo node
add_executable(stereo src/stereo_node.cpp)
add_dependencies(stereo ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)

# Kalman node
add_executable(kalman_node src/kalman_estimator_node.cpp)
add_dependencies(kalman_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_node ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES})

# Publish image node
add_executable(publish_image_node src/image_publisher.cpp)
add_dependencies(publish_image_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_image_node ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES})

# Ball trajectory generator for testing kalman node
add_executable(ball_traj_tester src/ball_traj_tester.cpp)
add_dependencies(ball_traj_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_traj_tester ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES})

# Realtime RRT Star tests
add_executable(path_planner_node src/path_planner_node.cpp src/RT_RRT_Star.cpp src/LineSampler.cpp src/UnitBallSampler.cpp src/ElipsisSampler.cpp)
add_dependencies(path_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_node ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})

# If we want to use ui files add them here
#QT4_WRAP_UI(UIS_OUT_H src/RW_Live_plugin/SamplePlugin.ui)

# Moc
#QT4_WRAP_CPP(MocSrcFiles src/RW_Live_plugin/SamplePlugin.hpp)

# Rcc the files:
#QT4_ADD_RESOURCES(RccSrcFiles src/RW_Live_plugin/resources.qrc)

# Add the source files to build (besides the Qt generated source files)
#ADD_LIBRARY(RW_Live_plugin MODULE src/RW_Live_plugin/SamplePlugin.cpp ${UIS_OUT_H} ${MocSrcFiles} ${RccSrcFiles})
#TARGET_LINK_LIBRARIES(RW_Live_plugin ${OpenCV_LIBS} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_install_python(PROGRAMS scripts/trajectory_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
