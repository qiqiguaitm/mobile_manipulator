#!/bin/bash

# Run script for AgileX robot project
# Usage: ./run_all.sh [camera|lidar|imu|arm|chassis|perception|slam|odom|all|list]

echo "Running AgileX robot project..."

# Source ROS environment and workspace
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

# Check if a specific driver is requested
if [ $# -eq 0 ] || [ "$1" = "all" ]; then
    echo "Launching all modules..."
    roslaunch /home/agilex/AgileXDemo/catkin_ws/src/agilex_demo.launch
elif [ "$1" = "camera" ]; then
    echo "Launching camera driver..."
    roslaunch realsense2_camera realsense_cameras.launch
elif [ "$1" = "lidar" ]; then
    echo "Launching LiDAR driver..."
    roslaunch lidar_driver lidar_driver_simple.launch
elif [ "$1" = "imu" ]; then
    echo "Launching IMU driver..."
    roslaunch imu_driver imu_driver.launch
elif [ "$1" = "arm" ]; then
    echo "Launching arm modules..."
    roslaunch arm_planner arm_planner.launch
elif [ "$1" = "chassis" ]; then
    echo "Launching chassis modules..."
    roslaunch chassis_planner chassis_planner.launch
elif [ "$1" = "perception" ]; then
    echo "Launching perception modules..."
    roslaunch perception perception.launch
elif [ "$1" = "slam" ]; then
    echo "Launching SLAM modules..."
    roslaunch slam slam.launch
elif [ "$1" = "odom" ]; then
    echo "Launching odometry modules..."
    roslaunch odom odom_test.launch
elif [ "$1" = "list" ]; then
    echo "Available launch options:"
    echo "  all          - Launch all modules"
    echo "  camera       - Launch camera driver"
    echo "  lidar        - Launch LiDAR driver"
    echo "  imu          - Launch IMU driver"
    echo "  arm          - Launch arm modules"
    echo "  chassis      - Launch chassis modules"
    echo "  perception   - Launch perception modules"
    echo "  slam         - Launch SLAM modules"
    echo "  odom         - Launch odometry modules"
    echo "  list         - Show this list"
else
    echo "Unknown option: $1"
    echo "Use './run_all.sh list' to see available options"
    echo "Defaulting to launch all modules..."
    roslaunch /home/agilex/AgileXDemo/catkin_ws/src/agilex_demo.launch
fi
