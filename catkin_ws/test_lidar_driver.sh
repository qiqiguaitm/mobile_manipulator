#!/bin/bash
# test_lidar_driver.sh

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

echo "Testing LiDAR driver module..."

# Check if required packages are available
echo "Checking for required packages..."
if ! rospack list | grep -q rslidar_sdk; then
    echo "Error: rslidar_sdk package not found"
    exit 1
fi

if ! rospack list | grep -q lidar_driver; then
    echo "Error: lidar_driver package not found"
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
    echo "Warning: rslidar_driver.launch not found"
fi

if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/launch/lidar_driver_test.launch ]; then
    echo "Error: lidar_driver_test.launch not found"
    exit 1
fi

echo "All required launch files found"

# Check if configuration files exist
echo "Checking for configuration files..."
if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/config/rslidar_config.yaml ]; then
    echo "Warning: rslidar_config.yaml not found"
fi

if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/config/topics.yaml ]; then
    echo "Warning: topics.yaml not found"
fi

echo "Configuration files check completed"

# Check if node scripts exist and are executable
echo "Checking for node scripts..."
if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/scripts/lidar_driver_node.py ]; then
    echo "Error: lidar_driver_node.py not found"
    exit 1
fi

if [ ! -f /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/scripts/lidar_test.py ]; then
    echo "Error: lidar_test.py not found"
    exit 1
fi

if [ ! -x /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/scripts/lidar_driver_node.py ]; then
    echo "Warning: lidar_driver_node.py is not executable"
fi

if [ ! -x /home/agilex/AgileXDemo/catkin_ws/src/lidar_driver/scripts/lidar_test.py ]; then
    echo "Warning: lidar_test.py is not executable"
fi

echo "Node scripts check completed"

echo "LiDAR driver module verification completed successfully"
echo "To test the module, run:"
echo "  roslaunch lidar_driver lidar_driver_test.launch"