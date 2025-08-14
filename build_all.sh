#!/bin/bash

# Build script for AgileX robot project
# Usage: ./build_all.sh [camera|lidar|imu|arm|chassis|perception|slam|odom|all|list]

echo "Building AgileX robot project..."

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace
cd /home/agilex/AgileXDemo/catkin_ws

if [ $# -eq 0 ] || [ "$1" = "all" ]; then
    echo "Building all packages..."
    catkin_make
elif [ "$1" = "camera" ]; then
    echo "Building camera driver packages..."
    catkin_make --only-pkg-with-deps realsense2_camera
elif [ "$1" = "lidar" ]; then
    echo "Building LiDAR driver packages..."
    catkin_make --only-pkg-with-deps lidar_driver
elif [ "$1" = "imu" ]; then
    echo "Building IMU driver packages..."
    catkin_make --only-pkg-with-deps imu_driver
elif [ "$1" = "arm" ]; then
    echo "Building arm packages..."
    catkin_make --only-pkg-with-deps arm_planner arm_controller
elif [ "$1" = "chassis" ]; then
    echo "Building chassis packages..."
    catkin_make --only-pkg-with-deps chassis_planner chassis_controller
elif [ "$1" = "perception" ]; then
    echo "Building perception packages..."
    catkin_make --only-pkg-with-deps perception object_tracker target_filter
elif [ "$1" = "slam" ]; then
    echo "Building SLAM packages..."
    catkin_make --only-pkg-with-deps slam
elif [ "$1" = "odom" ]; then
    echo "Building odometry packages..."
    catkin_make --only-pkg-with-deps odom lidar_driver
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
