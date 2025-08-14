cmake_minimum_required(VERSION 3.0.2)
project(target_filter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/target_filter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)