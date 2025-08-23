#!/bin/bash
# 启动RViz查看3D可视化

echo "Starting RViz for 3D Grasp Visualization..."

# 检查DISPLAY环境变量
if [ -z "$DISPLAY" ]; then
    echo "WARNING: No display detected!"
    echo "Try one of these:"
    echo "  1. Set DISPLAY=:0 if you have a monitor"
    echo "  2. Use VNC: export DISPLAY=:1"
    echo "  3. Use SSH X11 forwarding: ssh -X user@robot"
    exit 1
fi

# 启动RViz
rosrun rviz rviz -d /home/agilex/MobileManipulator/src/perception/config/grasp_3d_simple.rviz &
RVIZ_PID=$!

echo ""
echo "RViz Started!"
echo "You should see:"
echo "  - 3D bounding boxes (colored by confidence)"
echo "  - Grasp poses (arrows showing approach)"
echo "  - Touching points (purple lines)"
echo "  - Object labels"
echo ""
echo "Press Ctrl+C to close..."

trap "kill $RVIZ_PID 2>/dev/null; exit" INT TERM
wait $RVIZ_PID