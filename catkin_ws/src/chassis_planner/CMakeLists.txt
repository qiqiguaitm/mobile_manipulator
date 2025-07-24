cmake_minimum_required(VERSION 3.0.2)
project(chassis_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/chassis_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)