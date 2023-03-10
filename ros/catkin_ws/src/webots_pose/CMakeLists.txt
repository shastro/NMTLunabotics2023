# Adapted from CMakeLists.txt from david_motor of the old robot.

# Needs simplification ~~Alex

cmake_minimum_required(VERSION 3.0.2)
project(webots_pose)

catkin_package(CATKIN_DEPENDS std_msgs std_srvs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)
include_directories(include ${catkin_INCLUDE_DIRS} ../../../../teknic/inc/inc-pub)
add_executable(webots_pose src/webots_pose_node.cpp)
target_link_libraries(webots_pose ${catkin_LIBRARIES})

# add_message_files(
#   DIRECTORY msg
#   FILES
#   AugerCmd.msg
#   DumperCmd.msg
#   DepthCmd.msg
# )
# generate_messages(DEPENDENCIES std_msgs)

add_dependencies(webots_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(${catkin_LIBRARY_DIRS})

install(TARGETS webots_pose
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
