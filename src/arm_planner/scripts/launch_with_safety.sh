#!/bin/bash
# 安全系统快速启动脚本

echo "=========================================="
echo "Piper 机器人安全启动脚本"
echo "=========================================="

# 默认参数
MODE="real"
GRIPPER="true"
SAFETY="enhanced"
PANEL="enhanced"
VIZ="false"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --fake)
            MODE="fake"
            shift
            ;;
        --no-gripper)
            GRIPPER="false"
            shift
            ;;
        --basic-safety)
            SAFETY="basic"
            shift
            ;;
        --no-safety)
            SAFETY="none"
            shift
            ;;
        --visualize-safety)
            VIZ="true"
            shift
            ;;
        --simple-panel)
            PANEL="simple"
            shift
            ;;
        --help)
            echo "使用方法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --fake              使用仿真模式（默认：实机模式）"
            echo "  --no-gripper        不使用夹爪（默认：使用夹爪）"
            echo "  --basic-safety      使用基础安全系统（默认：增强安全）"
            echo "  --no-safety         禁用安全系统"
            echo "  --visualize-safety  启用安全可视化"
            echo "  --simple-panel      使用简单控制面板（默认：增强面板）"
            echo "  --help              显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                          # 实机模式，增强安全"
            echo "  $0 --fake                   # 仿真模式，增强安全"
            echo "  $0 --basic-safety           # 实机模式，基础安全"
            echo "  $0 --fake --no-safety       # 仿真模式，无安全系统"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 设置Python脚本可执行权限
echo "检查脚本权限..."
chmod +x /home/agilex/MobileManipulator/src/arm_planner/scripts/safety/*.py 2>/dev/null

# 显示配置
echo ""
echo "启动配置:"
echo "  - 模式: $MODE"
echo "  - 夹爪: $GRIPPER"
echo "  - 安全系统: $SAFETY"
echo "  - 控制面板: $PANEL"
echo "  - 安全可视化: $VIZ"
echo ""

# 清理现有ROS进程
echo "清理现有ROS进程..."
pkill -9 roscore 2>/dev/null
pkill -9 rosmaster 2>/dev/null
sleep 2

# 启动roscore
echo "启动ROS核心..."
roscore &
sleep 3

# 启动主程序
echo "启动机器人系统..."
roslaunch arm_planner main_demo.launch \
    mode:=$MODE \
    gripper:=$GRIPPER \
    safety_system:=$SAFETY \
    control_panel:=$PANEL \
    enable_safety_visualization:=$VIZ \
    enable_collision_monitoring:=true \
    enable_motion_smoothing:=true \
    safety_check_rate:=50

echo "系统已关闭"