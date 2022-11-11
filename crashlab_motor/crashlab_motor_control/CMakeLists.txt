cmake_minimum_required(VERSION 2.8.3)
project(crashlab_motor_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  crashlab_motor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES motor_test
  CATKIN_DEPENDS roscpp std_msgs crashlab_motor_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_control_node src/motor_control_node.cpp)
add_dependencies(motor_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_control_node ${catkin_LIBRARIES} pigpiod_if2)
