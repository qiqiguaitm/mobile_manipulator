cmake_minimum_required(VERSION 3.0.2)
project(task_mgr NONE)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  task_mgr.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
)

install(PROGRAMS
  scripts/task_mgr_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)