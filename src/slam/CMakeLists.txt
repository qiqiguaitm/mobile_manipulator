cmake_minimum_required(VERSION 3.0.2)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/slam_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)