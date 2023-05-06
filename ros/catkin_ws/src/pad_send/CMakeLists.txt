cmake_minimum_required(VERSION 2.8.3)
project(pad_send)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rosconsole
    roscpp
    std_msgs
    roscpp_serialization
    roslib
    rostime
    motor_bridge
)

catkin_package(CATKIN_DEPENDS std_msgs roscpp motor_bridge)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pad_send
    src/pad_send.cpp
    src/joystick/joystick.cpp
    src/joystick/gamepad.cpp)
add_dependencies(pad_send motor_bridge
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pad_send ${catkin_LIBRARIES} ncurses)
install(TARGETS
    pad_send
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
