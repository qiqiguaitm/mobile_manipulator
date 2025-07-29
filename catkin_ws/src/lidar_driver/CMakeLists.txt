cmake_minimum_required(VERSION 3.0.2)
project(lidar_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  rslidar_sdk
  pointcloud_to_laserscan
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  rslidar_sdk
  pointcloud_to_laserscan
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lidar_driver_node.py
  scripts/lidar_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)