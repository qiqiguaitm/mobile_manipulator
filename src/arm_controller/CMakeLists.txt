cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  arm_planner
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  arm_planner
)

catkin_python_setup()

install(PROGRAMS
  scripts/arm_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)