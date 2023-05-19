cmake_minimum_required(VERSION 3.0.2)
project(david_navigation)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rosconsole
    roscpp
    std_msgs
    roscpp_serialization
    roslib
    rostime
    david_config
    motor_bridge
    grid_map_core
    grid_map_ros
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  roscpp
  david_config
  motor_bridge
  grid_map_core
  grid_map_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(base_controller
  src/base_controller.cpp
)
add_dependencies(base_controller
  motor_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_controller ${catkin_LIBRARIES})

add_executable(occupancy_grid
  src/occupancy_grid.cpp
)
add_dependencies(occupancy_grid
    grid_map_core
    grid_map_ros
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occupancy_grid ${catkin_LIBRARIES})

link_directories(${catkin_LIBRARY_DIRS})
install(TARGETS
   base_controller
   occupancy_grid
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
