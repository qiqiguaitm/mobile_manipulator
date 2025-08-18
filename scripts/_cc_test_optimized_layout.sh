#!/bin/bash

echo "测试优化后的RViz布局..."

# 清理ROS进程
pkill -9 roscore
pkill -9 rosmaster
sleep 2

# 启动roscore
roscore &
sleep 3

# 直接启动RViz查看优化后的布局
echo "启动RViz查看布局..."
rosrun rviz rviz -d /home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/launch/moveit_with_stop_panel_main_demo_optimized.rviz &

echo "RViz已启动，请检查Motion Planning面板是否只占左侧1/3空间"
echo "按Ctrl+C退出"

# 等待用户退出
while true; do
    sleep 1
done