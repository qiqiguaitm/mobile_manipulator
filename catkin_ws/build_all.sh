#!/bin/bash

# Build script for AgileX robot project
echo "Building AgileX robot project..."

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace
cd /home/agilex/AgileXDemo/catkin_ws
catkin_make

echo "Build completed!"