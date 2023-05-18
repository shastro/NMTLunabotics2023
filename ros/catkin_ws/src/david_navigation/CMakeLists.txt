cmake_minimum_required(VERSION 2.8.3)
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
)

catkin_package(CATKIN_DEPENDS std_msgs roscpp david_config)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(base_controller src/base_controller.cpp)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(base_controller ${catkin_LIBRARIES})
add_dependencies(base_controller motor_bridge
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(TARGETS
   base_controller
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
