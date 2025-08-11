#!/bin/bash

echo "========== Piper机械臂完整系统测试 =========="

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

# 停止所有ROS进程
echo "停止现有ROS进程..."
pkill -f ros
sleep 3

# 启动roscore
echo "启动roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

# 激活CAN设备
echo "激活CAN设备..."
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash can_activate.sh can0 1000000

# 启动demo.launch
echo "启动MoveIt系统..."
roslaunch piper_with_gripper_moveit demo.launch \
    moveit_controller_manager:=fake \
    real_hardware:=true \
    use_rviz:=false \
    can_port:=can0 \
    auto_enable:=true &
LAUNCH_PID=$!

# 等待系统启动
echo "等待系统启动（30秒）..."
sleep 30

# 检查关键节点
echo "检查关键节点状态..."
echo "ROS节点列表:"
rosnode list

echo -e "\n可用服务列表:"
rosservice list | grep -E "(joint_moveit|enable|go_zero)"

echo -e "\n话题列表:"
rostopic list | grep -E "(joint_states|arm_status|end_pose)"

# 测试基本服务
echo -e "\n========== 测试基本功能 =========="

# 测试使能服务
echo "测试机械臂使能..."
rosservice call /enable_srv "enable_request: true"
sleep 2

# 测试MoveIt服务
echo "测试MoveIt服务可用性..."
if rosservice list | grep -q "joint_moveit_ctrl_endpose"; then
    echo "✅ joint_moveit_ctrl_endpose 服务可用"
    
    # 测试末端位姿控制
    echo "测试末端位姿控制..."
    python3 /home/agilex/AgileXDemo/catkin_ws/src/piper_ros/test_joint_moveit_service.py
else
    echo "❌ joint_moveit_ctrl_endpose 服务不可用"
fi

echo -e "\n========== 系统状态总结 =========="
echo "如果看到✅标记，说明对应功能正常"
echo "如果看到❌标记，请检查对应的配置和连接"
echo -e "\n系统保持运行中..."
echo "按Ctrl+C停止系统"

# 保持脚本运行
trap "echo '正在停止系统...'; kill $LAUNCH_PID $ROSCORE_PID; exit 0" INT
wait