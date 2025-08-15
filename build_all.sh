#!/bin/bash

# Build script for MobileManipulator robot project
# Usage: ./build_all.sh [camera|lidar|imu|arm|chassis|perception|slam|odom|all|list]

echo "Building MobileManipulator robot project..."

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace
cd /home/agilex/MobileManipulator

if [ $# -eq 0 ] || [ "$1" = "all" ]; then
    echo "Building all packages..."
    catkin build
elif [ "$1" = "camera" ]; then
    echo "Building camera driver packages..."
    catkin build realsense2_camera camera_driver
elif [ "$1" = "lidar" ]; then
    echo "Building LiDAR driver packages..."
    catkin build lidar_driver
elif [ "$1" = "imu" ]; then
    echo "Building IMU driver packages..."
    catkin build imu_driver
elif [ "$1" = "arm" ]; then
    echo "Building arm packages..."
    catkin build arm_planner arm_controller
elif [ "$1" = "chassis" ]; then
    echo "Building chassis packages..."
    catkin build chassis_planner chassis_controller
elif [ "$1" = "perception" ]; then
    echo "Building perception packages..."
    catkin build perception object_tracker target_filter inference_abstraction
elif [ "$1" = "slam" ]; then
    echo "Building SLAM packages..."
    catkin build slam
elif [ "$1" = "odom" ]; then
    echo "Building odometry packages..."
    catkin build odom
elif [ "$1" = "list" ]; then
    echo "Available packages:"
    echo "  all          - Build all packages"
    echo "  camera       - Build camera driver packages"
    echo "  lidar        - Build LiDAR driver packages"
    echo "  imu          - Build IMU driver packages"
    echo "  arm          - Build arm planner and controller packages"
    echo "  chassis      - Build chassis planner and controller packages"
    echo "  perception   - Build perception related packages"
    echo "  slam         - Build SLAM packages"
    echo "  odom         - Build odometry packages"
    echo "  list         - Show this list"
else
    echo "Unknown package: $1"
    echo "Use './build_all.sh list' to see available packages"
    echo "Defaulting to build all packages..."
    catkin_make
fi

echo "Build completed!"
