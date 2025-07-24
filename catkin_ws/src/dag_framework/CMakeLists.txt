cmake_minimum_required(VERSION 3.0.2)
project(dag_framework)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_python_setup()

install(PROGRAMS
  scripts/dag_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)