# Adapted from CMakeLists.txt from david_motor of the old robot.

# Needs simplification ~~Alex

cmake_minimum_required(VERSION 3.0.2)
project(motor_bridge)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)
include_directories(include ${catkin_INCLUDE_DIRS} ../../../../teknic/inc/inc-pub)
add_executable(motor_bridge src/motor_bridge_node.cpp)
target_link_libraries(motor_bridge ${catkin_LIBRARIES})

# add_message_files(
#   DIRECTORY msg
#   FILES
#   AugerCmd.msg
#   DumperCmd.msg
#   DepthCmd.msg
# )
# generate_messages(DEPENDENCIES std_msgs)

add_dependencies(motor_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS std_msgs std_srvs)

install(TARGETS motor_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
