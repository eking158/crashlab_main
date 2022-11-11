cmake_minimum_required(VERSION 2.8.3)
project(crashlab_motor_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  crashlab_motor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES motor_pid_node
  CATKIN_DEPENDS roscpp std_msgs crashlab_motor_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_pid_node src/motor_pid_node.cpp)
add_dependencies(motor_pid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_pid_node ${catkin_LIBRARIES} pigpiod_if2)
