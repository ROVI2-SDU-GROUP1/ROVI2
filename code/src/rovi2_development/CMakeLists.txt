cmake_minimum_required(VERSION 2.8.3)
project(rovi2_development)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS -std=c++11)

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
