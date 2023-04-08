cmake_minimum_required(VERSION 3.0.2)
project(talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()
add_executable(talker src/talker_node.cpp)
add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(talker ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})
install(TARGETS talker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

