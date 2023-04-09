cmake_minimum_required(VERSION 2.8.3)
project(pad_recv)

find_package(catkin REQUIRED COMPONENTS
    rosconsole
    roscpp
    std_msgs
    roscpp_serialization
    roslib
    rostime
    pad_send
)

catkin_package(CATKIN_DEPENDS std_msgs roscpp pad_send)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pad_recv src/pad_recv.cpp)
add_dependencies(pad_recv pad_send
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pad_recv ${catkin_LIBRARIES} ncurses)
link_directories(${catkin_LIBRARY_DIRS})
install(TARGETS
    pad_recv
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
