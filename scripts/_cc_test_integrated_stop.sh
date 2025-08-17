#!/bin/bash

# 测试集成到main_demo.launch的停止按钮功能
echo "测试集成后的停止按钮功能..."

# 清理现有的ros进程
echo "清理ROS进程..."
pkill -9 roscore 2>/dev/null
pkill -9 rosmaster 2>/dev/null
pkill -9 rviz 2>/dev/null
sleep 2

# 启动roscore
echo "启动roscore..."
roscore &
sleep 3

echo "选择测试模式:"
echo "1. 测试main_demo.launch带停止按钮 (stop_button:=true)"
echo "2. 测试main_demo.launch不带停止按钮 (stop_button:=false)"
echo "3. 测试独立的停止按钮GUI"

read -p "请选择 (1-3): " choice

cd /home/agilex/MobileManipulator
export ROS_PACKAGE_PATH=/home/agilex/MobileManipulator/src:$ROS_PACKAGE_PATH

case $choice in
    1)
        echo "启动main_demo.launch带停止按钮..."
        echo "使用参数: gripper:=true stop_button:=true"
        roslaunch arm_planner main_demo.launch gripper:=true stop_button:=true
        ;;
        
    2)
        echo "启动main_demo.launch不带停止按钮..."
        echo "使用参数: gripper:=true stop_button:=false"
        roslaunch arm_planner main_demo.launch gripper:=true stop_button:=false
        ;;
        
    3)
        echo "测试独立的停止按钮GUI..."
        # 先启动MoveIt
        roslaunch arm_planner demo_safe.launch &
        sleep 5
        
        # 启动独立的停止按钮
        echo "启动独立停止按钮GUI..."
        python3 src/arm_planner/scripts/simple_stop_button.py &
        sleep 2
        
        # 启动RViz
        echo "启动RViz..."
        rviz -d src/arm_planner/moveit_config/piper_with_gripper/launch/moveit_full.rviz
        ;;
        
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo "测试完成"