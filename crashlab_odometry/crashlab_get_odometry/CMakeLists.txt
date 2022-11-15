cmake_minimum_required(VERSION 2.8.3)
project(crashlab_get_odometry)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  crashlab_motor_msgs
  geometry_msgs
  roslib
  rospy
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES motor_test
  CATKIN_DEPENDS roscpp std_msgs crashlab_motor_msgs geometry_msgs roslib rospy sensor_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(get_odometry_node src/get_odometry_node.cpp)
add_dependencies(get_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_odometry_node ${catkin_LIBRARIES})