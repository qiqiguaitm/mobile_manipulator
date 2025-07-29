cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  camera_info_manager
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)