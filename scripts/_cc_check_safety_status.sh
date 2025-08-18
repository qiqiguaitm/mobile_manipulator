#!/bin/bash
# 检查安全系统状态的脚本

echo "======================================"
echo "安全系统状态检查"
echo "======================================"

echo -e "\n1. 检查紧急停止状态："
timeout 2 rostopic echo -n 1 /emergency_stop_signal 2>/dev/null || echo "未发布"

echo -e "\n2. 检查碰撞检测状态："
timeout 2 rostopic echo -n 1 /collision_detected 2>/dev/null || echo "未发布"

echo -e "\n3. 检查安全距离："
timeout 2 rostopic echo -n 1 /collision_distances 2>/dev/null || echo "未发布"

echo -e "\n4. 检查机械臂状态："
timeout 2 rostopic echo -n 1 /joint_states 2>/dev/null | grep -E "velocity:|position:" || echo "未发布"

echo -e "\n5. 可用的恢复服务："
rosservice list | grep -E "reset|enable|stop" || echo "没有找到相关服务"

echo -e "\n======================================"
echo "恢复建议："
echo "1. 如果紧急停止激活，运行: rosservice call /reset_emergency_stop"
echo "2. 重置安全系统: rosservice call /reset_safety_system"
echo "3. 使用控制面板上的复位按钮"
echo "4. 运行自动恢复脚本: rosrun arm_planner _cc_recover_from_estop.py"
echo "======================================