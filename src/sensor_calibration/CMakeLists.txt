cmake_minimum_required(VERSION 3.0.2)
project(sensor_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/sensor_calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)