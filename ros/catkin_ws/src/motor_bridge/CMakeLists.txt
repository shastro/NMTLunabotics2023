# Adapted from CMakeLists.txt from david_motor of the old robot.

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
System.msg
    EStop.msg
    PitchCtrl.msg
    PitchAdjust.msg
    PitchTelem.msg
    LocoCtrl.msg
    ExcavCtrl.msg
    StepperCtrl.msg
    StepperCtrlAdjust.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs std_srvs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

# motor bridge node
add_executable(motor_bridge
  src/motor_bridge_node.cpp
  src/can/david.c
  src/can/canshit.cpp
  src/can_interface.cpp
)
add_dependencies(motor_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_bridge ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
install(TARGETS
    motor_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
