cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)