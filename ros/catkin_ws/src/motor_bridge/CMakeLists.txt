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
  Stepp.msg
  Digger.msg
  Estop.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs std_srvs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})



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

# stepp_ctrl node
add_executable(stepp_ctrl
  src/stepp_ctrl_node.cpp
  src/can/david.c
  src/can_interface.cpp
)

# excav_ctrl node
add_executable(excav_ctrl
  src/excav_ctrl_node.cpp
  src/can/david.c
  src/can_interface.cpp
)

# estop node
add_executable(estop
  src/estop_node.cpp
  src/can/david.c
  src/can_interface.cpp
)


add_dependencies(pitch_ctrl
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(loco_ctrl
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(stepp_ctrl
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(excav_ctrl
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(estop
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pitch_ctrl ${catkin_LIBRARIES})
target_link_libraries(loco_ctrl ${catkin_LIBRARIES})
target_link_libraries(excav_ctrl ${catkin_LIBRARIES})
target_link_libraries(stepp_ctrl ${catkin_LIBRARIES})
target_link_libraries(estop ${catkin_LIBRARIES})


link_directories(${catkin_LIBRARY_DIRS})

install(TARGETS 
    pitch_ctrl
    loco_ctrl
    stepp_ctrl
    excav_ctrl
    estop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
