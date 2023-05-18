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
)

catkin_package(CATKIN_DEPENDS std_msgs roscpp david_config)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(costmap_cvt src/costmap_cvt.cpp)
add_dependencies(costmap_cvt david_config
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(costmap_cvt ${catkin_LIBRARIES})
install(TARGETS
    costmap_cvt
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
