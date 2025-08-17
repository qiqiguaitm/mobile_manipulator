#!/bin/bash

# 测试RViz停止按钮功能
echo "开始测试RViz停止按钮功能..."

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
echo "1. 测试独立停止按钮GUI (simple_stop_button.py)"
echo "2. 测试RViz集成停止面板 (rviz_with_stop_button.launch)"
echo "3. 测试新的RViz配置带停止面板 (moveit_with_stop_panel.rviz)"

read -p "请选择 (1-3): " choice

case $choice in
    1)
        echo "启动独立停止按钮GUI..."
        cd /home/agilex/MobileManipulator
        export ROS_PACKAGE_PATH=/home/agilex/MobileManipulator/src:$ROS_PACKAGE_PATH
        
        # 启动机械臂控制
        echo "启动机械臂控制..."
        roslaunch arm_planner demo_safe.launch &
        sleep 5
        
        # 启动独立停止按钮
        echo "启动停止按钮GUI..."
        python3 src/arm_planner/scripts/simple_stop_button.py
        ;;
        
    2)
        echo "启动RViz集成停止面板..."
        cd /home/agilex/MobileManipulator
        export ROS_PACKAGE_PATH=/home/agilex/MobileManipulator/src:$ROS_PACKAGE_PATH
        
        # 启动机械臂控制
        echo "启动机械臂控制..."
        roslaunch arm_planner demo_safe.launch &
        sleep 5
        
        # 启动RViz带停止按钮
        echo "启动RViz带停止按钮..."
        roslaunch arm_planner rviz_with_stop_button.launch
        ;;
        
    3)
        echo "启动新的RViz配置..."
        cd /home/agilex/MobileManipulator
        export ROS_PACKAGE_PATH=/home/agilex/MobileManipulator/src:$ROS_PACKAGE_PATH
        
        # 启动机械臂控制
        echo "启动机械臂控制..."
        roslaunch arm_planner demo_safe.launch &
        sleep 5
        
        # 启动新的RViz配置
        echo "启动RViz新配置..."
        rviz -d src/arm_planner/moveit_config/piper_with_gripper/launch/moveit_with_stop_panel.rviz
        ;;
        
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo "测试完成"