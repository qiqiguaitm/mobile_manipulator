#!/bin/bash
# 启动深度投影可视化系统演示

echo "=== 深度投影可视化系统启动 ==="
echo "本脚本将启动："
echo "1. 机器人模型显示"
echo "2. TF坐标系树"
echo "3. 深度投影点云可视化"
echo "4. RViz界面"
echo ""

# 检查依赖
echo ">>> 检查依赖..."
if ! rospack find mobile_manipulator2_description > /dev/null 2>&1; then
    echo "✗ 找不到 mobile_manipulator2_description 包"
    exit 1
fi

if ! rospack find perception > /dev/null 2>&1; then
    echo "✗ 找不到 perception 包"
    exit 1
fi

echo "✓ 依赖检查通过"

# 启动roscore (如果没有运行)
if ! rostopic list > /dev/null 2>&1; then
    echo ">>> 启动 roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
else
    echo ">>> roscore 已运行"
    ROSCORE_PID=""
fi

# 清理函数
cleanup() {
    echo ""
    echo ">>> 清理进程..."
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
    fi
    pkill -f "depth_projection.launch" 2>/dev/null
    pkill -f "rviz" 2>/dev/null
    echo ">>> 清理完成"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

echo ""
echo ">>> 启动深度投影可视化..."
echo "参数配置:"
echo "  - 启用RViz: true"
echo "  - 加载机器人模型: true"
echo "  - 启用底盘相机: true"
echo "  - 启用顶部相机: true"
echo "  - 启用手部相机: false (需要TF链接)"
echo ""

# 启动深度投影launch文件
roslaunch perception depth_projection.launch \
    use_rviz:=true \
    load_robot_description:=true \
    enable_chassis_camera:=false \
    enable_top_camera:=false \
    enable_hand_camera:=false \
    publish_combined:=true &

LAUNCH_PID=$!

# 等待启动
echo ">>> 等待系统启动..."
sleep 5

# 检查进程是否正常
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo "✗ Launch文件启动失败"
    cleanup
fi

echo ""
echo "🎉 系统启动成功！"
echo ""
echo "RViz界面说明："
echo "• 机器人模型: 显示完整的移动机械臂"
echo "• TF坐标系: 显示相机和关节的坐标frame"
echo "• 点云显示:"
echo "  - 蓝色: 底盘相机投影点云"
echo "  - 粉色: 顶部相机投影点云"
echo "  - 绿色: 手部相机投影点云"
echo "  - 白色: 合并点云 (默认关闭)"
echo ""
echo "话题列表:"
echo "• /projected_cloud/chassis_camera - 底盘相机点云"
echo "• /projected_cloud/top_camera - 顶部相机点云"
echo "• /projected_cloud/hand_camera - 手部相机点云"
echo "• /projected_cloud/combined - 合并点云"
echo ""
echo "如要连接真实相机，请先运行:"
echo "  roslaunch camera_driver camera_driver.launch"
echo ""
echo "按 Ctrl+C 退出..."

# 保持运行
wait $LAUNCH_PID