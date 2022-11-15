cmake_minimum_required(VERSION 2.8.3)
project(crashlab_teleop_key)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  crashlab_motor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES motor_test
  CATKIN_DEPENDS roscpp std_msgs crashlab_motor_msgs geometry_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teleop_key_node src/teleop_key_node.cpp)
add_dependencies(teleop_key_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_key_node ${catkin_LIBRARIES})
