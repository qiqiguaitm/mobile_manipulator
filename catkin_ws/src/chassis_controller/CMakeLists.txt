cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/chassis_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)