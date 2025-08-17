#!/bin/bash

# 测试修改后的停止按钮（使用go_zero_srv和stop_srv）
echo "测试修改后的停止按钮功能（使用服务调用）..."

# 清理现有的ros进程，构建干净环境
echo "清理ROS进程..."
pkill -9 roscore 2>/dev/null
pkill -9 rosmaster 2>/dev/null
pkill -9 rviz 2>/dev/null
pkill -9 python3 2>/dev/null
sleep 3

# 启动roscore
echo "启动roscore..."
roscore &
sleep 3

cd /home/agilex/MobileManipulator
export ROS_PACKAGE_PATH=/home/agilex/MobileManipulator/src:$ROS_PACKAGE_PATH

echo "启动机械臂控制节点（提供stop_srv和go_zero_srv服务）..."
# 启动机械臂控制节点，提供所需服务
rosrun arm_controller piper_ctrl_single_node.py _can_port:=can0 _auto_enable:=false _gripper_exist:=true &
sleep 5

echo "检查服务是否可用..."
rosservice list | grep -E "(stop_srv|go_zero_srv)"

echo ""
echo "启动修改后的停止按钮GUI..."
echo "现在停止按钮会调用stop_srv服务"
echo "GO ZERO按钮会调用go_zero_srv服务"
echo ""

# 启动修改后的停止按钮
python3 src/arm_planner/scripts/simple_stop_button.py

echo "测试完成"