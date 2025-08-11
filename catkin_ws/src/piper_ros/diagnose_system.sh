#!/bin/bash

echo "========== Piper机械臂系统诊断 =========="

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

echo "1. 检查ROS核心状态..."
if pgrep -f roscore > /dev/null; then
    echo "✅ roscore 正在运行"
else
    echo "❌ roscore 未运行"
fi

echo -e "\n2. 检查CAN设备状态..."
if ip link show can0 2>/dev/null | grep -q "UP"; then
    echo "✅ CAN设备 can0 已激活"
    ip -details link show can0 | grep bitrate
else
    echo "❌ CAN设备 can0 未激活或不存在"
fi

echo -e "\n3. 检查关键ROS节点..."
nodes=$(rosnode list 2>/dev/null)
if echo "$nodes" | grep -q piper_ctrl_single_node; then
    echo "✅ piper_ctrl_single_node 正在运行"
else
    echo "❌ piper_ctrl_single_node 未运行"
fi

if echo "$nodes" | grep -q fake_to_real_bridge; then
    echo "✅ fake_to_real_bridge 正在运行"
else
    echo "❌ fake_to_real_bridge 未运行"
fi

if echo "$nodes" | grep -q joint_moveit_ctrl_server; then
    echo "✅ joint_moveit_ctrl_server 正在运行"
else
    echo "❌ joint_moveit_ctrl_server 未运行"
fi

if echo "$nodes" | grep -q move_group; then
    echo "✅ move_group 正在运行"
else
    echo "❌ move_group 未运行"
fi

echo -e "\n4. 检查关键服务..."
services=$(rosservice list 2>/dev/null)
if echo "$services" | grep -q joint_moveit_ctrl_endpose; then
    echo "✅ joint_moveit_ctrl_endpose 服务可用"
else
    echo "❌ joint_moveit_ctrl_endpose 服务不可用"
fi

if echo "$services" | grep -q enable_srv; then
    echo "✅ enable_srv 服务可用"
else
    echo "❌ enable_srv 服务不可用"
fi

echo -e "\n5. 检查关键话题..."
topics=$(rostopic list 2>/dev/null)
if echo "$topics" | grep -q "/joint_states"; then
    echo "✅ /joint_states 话题存在"
else
    echo "❌ /joint_states 话题不存在"
fi

if echo "$topics" | grep -q "/joint_states_single"; then
    echo "✅ /joint_states_single 话题存在"
else
    echo "❌ /joint_states_single 话题不存在"
fi

echo -e "\n========== 诊断完成 =========="
echo "如果看到❌标记，表示对应组件有问题，需要检查配置或重启相关服务"

echo -e "\n推荐解决步骤："
echo "1. 如果CAN设备未激活: bash can_activate.sh can0 1000000"
echo "2. 如果关键节点未运行: 重启 demo.launch"
echo "3. 如果只是MoveIt服务缺失: bash start_moveit_services.sh"