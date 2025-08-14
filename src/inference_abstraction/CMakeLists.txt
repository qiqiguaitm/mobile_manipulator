cmake_minimum_required(VERSION 3.0.2)
project(inference_abstraction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/inference_abstraction_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)