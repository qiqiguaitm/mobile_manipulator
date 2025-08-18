cmake_minimum_required(VERSION 3.0.2)
project(lidar_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lidar_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)