cmake_minimum_required(VERSION 3.0.2)
project(arm_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Generate messages
add_message_files(
  FILES
  PiperEulerPose.msg
  PiperStatusMsg.msg
  PosCmd.msg
)

# Generate services
add_service_files(
  FILES
  Enable.srv
  GoZero.srv
  Gripper.srv
  JointMoveitCtrl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    control_msgs
    actionlib
    actionlib_msgs
    message_runtime
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/follow_joint_trajectory_server.py
  scripts/joint_moveit_ctrl_server.py
  scripts/launch_status_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install MoveIt configs
install(DIRECTORY moveit_config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/moveit_config
)