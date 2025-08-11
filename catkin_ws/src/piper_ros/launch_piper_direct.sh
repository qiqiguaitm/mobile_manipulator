#!/bin/bash

# Piper机械臂直接MoveIt控制启动脚本
echo "启动Piper机械臂直接MoveIt控制模式..."

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/AgileXDemo/catkin_ws/devel/setup.bash

# 激活CAN设备
echo "激活CAN设备..."
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash can_activate.sh can0 1000000

# 检查CAN设备是否激活成功
if ! ip link show can0 | grep -q "UP"; then
    echo "错误: CAN设备can0未激活成功!"
    echo "请检查："
    echo "1. USB转CAN模块是否正确连接"
    echo "2. 机械臂是否上电"
    echo "3. 是否有权限访问CAN设备"
    exit 1
fi

echo "CAN设备激活成功，启动直接MoveIt控制..."

# 启动MoveIt with direct control
cd /home/agilex/AgileXDemo/catkin_ws
roslaunch src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/launch/demo_direct.launch \
    can_port:=can0 \
    auto_enable:=true \
    use_rviz:=false