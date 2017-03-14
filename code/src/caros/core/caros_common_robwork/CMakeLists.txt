cmake_minimum_required(VERSION 2.8.3)
project(caros_common_robwork)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

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
  caros_common_robwork_msgs
  message_generation
  roscpp
  roslint
  std_msgs
)

########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-runtime/references,-whitespace/braces,-readability/check")
roslint_cpp()


########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
# )

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS caros_common caros_common_msgs caros_common_robwork_msgs message_runtime roscpp std_msgs
  DEPENDS ROBWORK
  CFG_EXTRAS "use-c++11.cmake"
)

include_directories(
  include
  ${ROBWORK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${library_name}
  src/common_robwork.cpp
)

target_link_libraries(${library_name}
  ${ROBWORK_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(${library_name}
  caros_common_robwork_msgs_generate_messages_cpp
)

########################################################################
#### Testing
########################################################################
if(CATKIN_ENABLE_TESTING)
  # Using WORKING_DIRECTORY to allow specifying a relative path to the test data that is being used.
  catkin_add_gtest(type_conversion_robwork test/type_conversion_robwork.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(type_conversion_robwork ${library_name} ${ROBWORK_LIBRARIES} ${catkin_LIBRARIES})
endif()
