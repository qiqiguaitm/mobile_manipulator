cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy serial sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(imu_driver src/imu_driver_complete.cpp src/imu_data_decode.c src/packet.c)

## Specify libraries to link a library or executable target against
target_link_libraries(imu_driver
  ${catkin_LIBRARIES}
)

## Rename C++ executable without prefix
set_target_properties(imu_driver PROPERTIES OUTPUT_NAME imu_driver PREFIX "")

## Mark executables and/or libraries for installation
install(TARGETS imu_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/imu_driver_node.py
  scripts/imu_test.py
  scripts/test_imu_topic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)