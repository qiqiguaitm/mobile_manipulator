#!/bin/bash

# Piper机械臂正确的rostopic发布示例脚本
# 配合 launch_piper_direct.sh 使用
echo "🎯 Piper机械臂正确的rostopic发布示例"
echo "配合 launch_piper_direct.sh 使用"
echo "================================================"

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/AgileXDemo/catkin_ws/devel/setup.bash

echo ""
echo "注意: 请确保已经运行了 bash launch_piper_direct.sh 启动Piper机械臂系统"
echo ""

echo "=== 正确的rostopic发布方式 ==="
echo ""

echo "📝 方式1: 直接控制机械臂关节 (推荐)"
echo "rostopic pub /joint_ctrl_commands sensor_msgs/JointState \\"
echo "\"{header: {stamp: now, frame_id: ''}, name: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0.5]}\" -r 10"

echo ""
echo "📝 方式2: 发布关节状态 (需要先停止bridge)"
echo "rosnode kill /joint_state_bridge"
echo "rostopic pub /joint_states sensor_msgs/JointState \\"
echo "\"{header: {stamp: now, frame_id: ''}, name: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01,0.01], velocity: [0,0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0.5,0.5]}\" -r 10"

echo ""
echo "📝 方式3: 使用测试话题避免冲突"
echo "rostopic pub /test_joint_states sensor_msgs/JointState \\"
echo "\"{header: {stamp: now, frame_id: ''}, name: ['joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01,0.01], velocity: [0,0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0.5,0.5]}\" -r 10"

echo ""
echo "=== 关节说明 ==="
echo "joint1-joint6: 机械臂关节"
echo "joint7: 夹爪主关节"
echo "joint8: 夹爪对称关节 (与joint7值相同)"
echo ""

echo "=== 常见错误及解决方案 ==="
echo "❌ 错误: 'New joint state is not newer than previous state'"
echo "原因: 多个节点同时发布joint_states或时间戳问题"
echo "解决: 使用 /joint_ctrl_commands 或先停止 joint_state_bridge"
echo ""

echo "❌ 错误: 'Missing joint1, joint2...'"
echo "原因: 关节名称数量不匹配或缺少joint8"
echo "解决: 确保包含所有8个关节名称"
echo ""

echo "❌ 错误: 'Received JointState is X seconds old'"
echo "原因: 使用了固定时间戳 {secs: 0, nsecs: 0}"
echo "解决: 使用 stamp: now"
echo ""

echo "=== 快速测试命令 ==="
echo ""
echo "选择一个命令执行:"
echo ""
echo "A) 直接控制机械臂 (推荐):"
echo "rostopic pub /joint_ctrl_commands sensor_msgs/JointState '{header: {stamp: now, frame_id: \"\"}, name: [\"joint1\",\"joint2\",\"joint3\",\"joint4\",\"joint5\",\"joint6\",\"joint7\"], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0.5]}' -r 10"
echo ""
echo "B) 停止当前发布:"
echo "pkill -f 'rostopic pub'"
echo ""
echo "C) 检查当前话题:"
echo "rostopic list | grep joint"