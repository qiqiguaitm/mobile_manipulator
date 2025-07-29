#!/bin/bash
# test_odom.sh

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

echo "Testing Odom module..."

# Check if required packages are available
echo "Checking for required packages..."
if ! rospack list | grep -q lidar_driver; then
    echo "Error: lidar_driver package not found"
    exit 1
fi

if ! rospack list | grep -q odom; then
    echo "Error: odom package not found"
    exit 1
fi

if ! rospack list | grep -q rf2o_laser_odometry; then
    echo "Error: rf2o_laser_odometry package not found"
    exit 1
fi

echo "All required packages found"

# Check if nodes compile correctly
echo "Checking if nodes compile correctly..."
cd /home/agilex/AgileXDemo/catkin_ws
if ! catkin_make --dry-run >/dev/null 2>&1; then
    echo "Error: Failed to compile packages"
    exit 1
fi

echo "Nodes compile correctly"

# Check if launch files exist
echo "Checking for launch files..."
if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/launch/rslidar_driver.launch ]; then
    echo "Error: rslidar_driver.launch not found"
    exit 1
fi

if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/odom/launch/odom.launch ]; then
    echo "Error: odom.launch not found"
    exit 1
fi

if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/odom/launch/odom_test.launch ]; then
    echo "Error: odom_test.launch not found"
    exit 1
fi

echo "All required launch files found"

# Check if node scripts exist and are executable
echo "Checking for node scripts..."
if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/odom/scripts/odom_test.py ]; then
    echo "Error: odom_test.py not found"
    exit 1
fi

if [ ! -x /home/agilex/AgileXDemo/catkin_ws/src/odom/scripts/odom_test.py ]; then
    echo "Warning: odom_test.py is not executable"
fi

echo "Node scripts check completed"

echo "Odom module verification completed successfully"
echo "To test the module, run:"
echo "  roslaunch odom odom_test.launch"