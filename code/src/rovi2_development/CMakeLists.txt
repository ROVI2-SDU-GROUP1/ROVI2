cmake_minimum_required(VERSION 3.0.0)
project(rovi2_development)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  message_generation
)
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




catkin_package(CATKIN_DEPENDS message_runtime)


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
 -ffunction-sections \
 -fdata-sections \
 -g" )

# Bumblebee node
add_executable(opencv_node src/opencv_node.cpp)
set_target_properties(opencv_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(opencv_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(opencv_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

# Image detector node
add_executable(image_detector src/image_detector_node.cpp)
add_dependencies(image_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_detector ${OpenCV_LIBS} ${catkin_LIBRARIES})

# Stereo node
add_executable(stereo src/stereo_node.cpp)
add_dependencies(stereo ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo ${OpenCV_LIBS} ${catkin_LIBRARIES})

# Kalman node
add_executable(kalman_node src/kalman_estimator_node.cpp)
add_dependencies(kalman_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_node ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES})

# Ball trajectory generator for testing kalman node
add_executable(ball_traj_tester src/ball_traj_tester.cpp)
add_dependencies(ball_traj_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_traj_tester ${OpenCV_LIBS} ${EIGEN3_LIBS} ${catkin_LIBRARIES})
