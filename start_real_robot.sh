#!/bin/bash
catkin build

# Piper机械臂实机控制快速启动脚本

echo "=========================================="
echo "    Piper机械臂实机控制启动脚本"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未配置!"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 设置工作空间
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${WORKSPACE_DIR}/devel/setup.bash

echo "启动完整演示模式..."
# 先配置CAN接口
echo "配置CAN接口..."
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 2>/dev/null
sudo ip link set can0 up 2>/dev/null
        
# 检查CAN状态
ip link show can0 | grep "state UP" > /dev/null 2>&1
if [ $? -ne 0 ]; then
            echo "错误: CAN接口启动失败，请检查硬件连接"
            exit 1
fi
echo "CAN接口配置成功"
    
# 启动系统
roslaunch piper_with_gripper_moveit demo.launch mode:=real use_rviz:=true &
  
