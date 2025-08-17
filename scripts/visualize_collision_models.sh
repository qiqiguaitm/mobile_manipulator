#!/bin/bash
# 可视化visual和collision模型对比

echo "=== 启动碰撞模型可视化 ==="
echo ""
echo "功能说明："
echo "- 绿色半透明模型：Visual模型（用于显示）"
echo "- 红色/紫色线框：Collision模型（用于碰撞检测）"
echo ""
echo "在RViz中可以："
echo "1. 调整'Visual Robot Model'的Alpha值来改变透明度"
echo "2. 开关'Collision Model'来显示/隐藏碰撞模型"
echo "3. 在'PlanningScene'中查看MoveIt的碰撞检测状态"
echo ""

# 清理已有的ROS进程
echo "清理已有ROS进程..."
pkill -9 roscore
pkill -9 rosmaster
sleep 2

# 启动roscore
echo "启动roscore..."
roscore &
sleep 3

# 启动robot_state_publisher
echo "启动robot_state_publisher..."
rosrun robot_state_publisher robot_state_publisher __name:=robot_state_publisher &
sleep 2

# 启动joint_state_publisher_gui
echo "启动joint_state_publisher_gui..."
rosrun joint_state_publisher_gui joint_state_publisher_gui __name:=joint_state_publisher &
sleep 2

# 启动MoveIt（可选，用于查看碰撞检测）
read -p "是否启动MoveIt查看碰撞检测？[y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动MoveIt..."
    roslaunch arm_planner arm_moveit.launch &
    sleep 5
fi

# 启动RViz
echo "启动RViz..."
rosrun rviz rviz -d /home/agilex/MobileManipulator/src/mobile_manipulator2_description/config/collision_visualization.rviz &

echo ""
echo "=== 启动完成 ==="
echo ""
echo "提示："
echo "1. Visual Robot Model（绿色）- 显示实际外观"
echo "2. Collision Model（线框）- 显示碰撞检测形状"
echo "3. 注意观察两者的差异，确保碰撞模型略大于视觉模型"
echo ""
echo "按Ctrl+C退出所有进程"

# 等待用户退出
wait