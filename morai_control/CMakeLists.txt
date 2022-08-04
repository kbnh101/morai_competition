cmake_minimum_required(VERSION 3.0.2)
project(morai_control)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  morai_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES morai_control
  CATKIN_DEPENDS ackermann_msgs geometry_msgs morai_msgs nav_msgs roscpp rospy std_msgs tf pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
 include/local_path.h
 src/local_path.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
