#!/bin/bash

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

echo ""
echo "请选择启动模式:"
echo "1. 完整演示模式 (自动运行演示)"
echo "2. 交互控制模式 (手动控制)"
echo "3. 仅启动MoveIt (不启动硬件驱动)"
echo ""
read -p "请输入选择 (1-3): " choice

case $choice in
    1)
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
        roslaunch piper_with_gripper_moveit demo_simple.launch mode:=real &
        LAUNCH_PID=$!
        
        # 等待系统启动
        echo ""
        echo -n "等待系统初始化"
        for i in {1..15}; do
            echo -n "."
            sleep 1
        done
        echo " 完成!"
        
        # 自动使能机器人
        echo ""
        echo "使能机器人..."
        python3 ${WORKSPACE_DIR}/enable_robot.py
        if [ $? -eq 0 ]; then
            echo "机器人已使能，可以开始控制"
        else
            echo "警告: 机器人使能失败，请手动使能"
        fi
        
        # 等待启动完成
        wait $LAUNCH_PID
        ;;
    2)
        echo "启动交互控制模式..."
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
        echo "启动机器人系统..."
        roslaunch piper_with_gripper_moveit demo_simple.launch mode:=real &
        LAUNCH_PID=$!
        
        # 等待系统启动
        echo ""
        echo -n "等待系统初始化"
        for i in {1..15}; do
            echo -n "."
            sleep 1
        done
        echo " 完成!"
        
        # 自动使能机器人
        echo ""
        echo "使能机器人..."
        python3 ${WORKSPACE_DIR}/enable_robot.py
        if [ $? -eq 0 ]; then
            echo "✓ 机器人已使能"
        else
            echo "✗ 机器人使能失败，请在控制程序中手动使能"
        fi
        
        # 启动交互控制
        echo ""
        echo "启动交互控制程序..."
        sleep 2
        rosrun moveit_ctrl simple_real_control.py
        
        # 清理
        kill $LAUNCH_PID 2>/dev/null
        ;;
    3)
        echo "仅启动MoveIt (不自动使能)..."
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
        
        echo ""
        echo "启动系统..."
        echo "注意: 此模式不会自动使能机器人"
        echo "使能命令: python3 ~/ztm/piper_ros/enable_robot.py"
        echo ""
        
        roslaunch piper_with_gripper_moveit demo_simple.launch mode:=real
        ;;
    *)
        echo "无效的选择!"
        exit 1
        ;;
esac