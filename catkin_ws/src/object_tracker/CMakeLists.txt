cmake_minimum_required(VERSION 3.0.2)
project(object_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/object_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)