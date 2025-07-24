cmake_minimum_required(VERSION 3.0.2)
project(arm_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ArmTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_runtime
)

install(PROGRAMS
  scripts/arm_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)