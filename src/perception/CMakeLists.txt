cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  ObjectDetection.msg
  ObjectDetectionArray.msg
  TouchingPoint.msg
  GraspPose.msg
  GraspDetection.msg
  GraspDetectionArray.msg
  TouchingPoint3D.msg
  GraspPose3D.msg
  GraspDetection3D.msg
  GraspDetectionArray3D.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
)

install(PROGRAMS
  scripts/perception_node.py
  scripts/chassis_detector.py
  scripts/hand_grasp_detector.py
  scripts/depth_grasp_processor.py
  scripts/depth_grasp_processor_v2.py
  scripts/grasp_tf_converter.py
  scripts/grasp_3d_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)