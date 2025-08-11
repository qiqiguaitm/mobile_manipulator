#!/bin/bash

# 启动MoveIt控制服务的独立脚本
echo "启动MoveIt控制服务..."

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source /home/agilex/AgileXDemo/catkin_ws/devel/setup.bash

# 检查roscore是否运行
if ! pgrep -f roscore > /dev/null; then
    echo "roscore未运行，请先启动roscore或完整的launch文件"
    exit 1
fi

# 启动joint_moveit_ctrl_server
echo "启动joint_moveit_ctrl_server..."
rosrun moveit_ctrl joint_moveit_ctrl_server.py &
SERVER_PID=$!

# 等待服务启动
sleep 5

echo "检查服务状态..."
if rosservice list | grep -q "joint_moveit_ctrl_endpose"; then
    echo "✅ MoveIt控制服务启动成功"
    echo "可用服务:"
    rosservice list | grep joint_moveit
    echo ""
    echo "现在可以使用以下命令测试:"
    echo "rosservice call /joint_moveit_ctrl_endpose \"joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577] max_velocity: 0.5 max_acceleration: 0.5\""
else
    echo "❌ MoveIt控制服务启动失败"
    kill $SERVER_PID
    exit 1
fi

# 保持运行
echo "服务保持运行中，按Ctrl+C停止"
trap "echo '停止服务...'; kill $SERVER_PID; exit 0" INT
wait $SERVER_PID