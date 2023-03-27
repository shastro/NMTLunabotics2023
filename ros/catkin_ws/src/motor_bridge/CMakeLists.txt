# Adapted from CMakeLists.txt from david_motor of the old robot.

# Needs simplification ~~Alex

cmake_minimum_required(VERSION 3.0.2)
project(motor_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosconsole
  roscpp
  roscpp_serialization
  roslib rostime
  std_msgs
  std_srvs
)

add_message_files(
  DIRECTORY msg
  FILES
  Pitch.msg
  Drive.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs std_srvs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ../../../../teknic/inc/inc-pub)

# pitch_ctrl node
add_executable(pitch_ctrl
  src/pitch_ctrl_node.cpp
  src/can/david.c
  src/can_interface.cpp
)

# loco_ctrl node
add_executable(loco_ctrl
  src/loco_ctrl_node.cpp
  src/can/david.c
  src/can_interface.cpp
)

target_link_libraries(pitch_ctrl loco_ctrl ${catkin_LIBRARIES})
add_dependencies(pitch_ctrl loco_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(${catkin_LIBRARY_DIRS})

install(TARGETS pitch_ctrl loco_ctrl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
