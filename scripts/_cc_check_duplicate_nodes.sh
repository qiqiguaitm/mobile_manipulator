#!/bin/bash
# 检查是否有重复的ground_collision_manager节点

echo "检查运行中的ROS节点..."
rosnode list | grep ground_collision

echo -e "\n检查demo.launch中的ground_collision_manager定义..."
grep -n "node.*ground_collision_manager" /home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/launch/demo.launch

echo -e "\n清理建议："
echo "1. 先停止所有ROS节点: rosnode kill -a"
echo "2. 或者: pkill -9 roscore && pkill -9 rosmaster"
echo "3. 然后重新启动: roslaunch arm_planner demo.launch"