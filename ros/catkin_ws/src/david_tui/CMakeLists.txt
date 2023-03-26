cmake_minimum_required(VERSION 3.0.2)
project(david_tui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosconsole
  roslib
  rostime
  motor_bridge
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   geometry_msgs
   motor_bridge
   std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# Nodes this package depends on
add_dependencies(pitch_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# david_tui node
add_executable(david_tui src/tui.cpp)
add_dependencies(david_tui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(david_tui ${catkin_LIBRARIES} ncurses)

install(TARGETS pitch_ctrl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
