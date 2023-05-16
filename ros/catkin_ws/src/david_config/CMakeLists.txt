cmake_minimum_required(VERSION 2.8.3)
project(david_config)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    roslaunch
)
roslaunch_add_file_check(launch)

catkin_package(CATKIN_DEPENDS std_msgs roscpp nav_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose_pub src/pose_pub.cpp)
add_dependencies(pose_pub
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_pub ${catkin_LIBRARIES})
install(TARGETS
    pose_pub
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
