#!/bin/bash

# Run script for AgileX robot project
echo "Running AgileX robot project..."

# Source ROS environment and workspace
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

# Launch all modules using the full path to the launch file
roslaunch /home/agilex/AgileXDemo/catkin_ws/src/agilex_demo.launch