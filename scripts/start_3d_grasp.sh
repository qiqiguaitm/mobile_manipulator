#!/bin/bash
# One-click script to start 3D grasp system
# All Python scripts now have libffi fix integrated

# 设置libffi环境变量以解决库冲突
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libffi.so.7

echo "Starting 3D Grasp System..."

# Clean up
rosnode kill -a 2>/dev/null || true
sleep 2

# Start all components
echo "Launching complete system..."
roslaunch perception grasp_3d.launch &

echo ""
echo "================================"
echo "3D Grasp System Started!"
echo "================================"
echo ""
echo "Place objects in front of camera"
echo "Monitor with: rostopic echo /perception/hand/grasps_3d"
echo ""
echo "Press Ctrl+C to stop..."

# Keep running
trap "rosnode kill -a; exit" INT TERM
while true; do
    sleep 1
done