#!/bin/bash
#
# MobileManipulator 主演示启动脚本
# 集成了优化的RViz布局（左侧面板占1/3）和完整的控制系统
#

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
MODE="real"
GRIPPER="true"
USE_RVIZ="true"
RVIZ_LAYOUT="optimized"
CONTROL_PANEL="enhanced"
SAFETY_SYSTEM="enhanced"
ENABLE_COLLISION_MONITORING="true"
ENABLE_MOTION_SMOOTHING="true"
SAFETY_CHECK_RATE="200"
DEBUG="false"
PIPELINE="ompl"

# 显示帮助信息
show_help() {
    echo -e "${BLUE}MobileManipulator 主演示启动脚本${NC}"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -m, --mode <real|sim>          运行模式 (默认: real)"
    echo "  -g, --no-gripper               不使用夹爪"
    echo "  -r, --no-rviz                  不启动RViz"
    echo "  -l, --layout <standard|opt>    RViz布局 (默认: optimized)"
    echo "  -p, --panel <none|simple|enh>  控制面板类型 (默认: enhanced)"
    echo "  -s, --safety <none|basic|enh>  安全系统级别 (默认: enhanced)"
    echo "  --no-collision                 禁用碰撞监控"
    echo "  --no-smoothing                 禁用运动平滑"
    echo "  --rate <hz>                    安全检查频率 (默认: 200)"
    echo "  -d, --debug                    启用调试模式"
    echo "  --pipeline <ompl|chomp|sbpl>  规划器类型 (默认: ompl)"
    echo "  -h, --help                     显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                            # 默认启动（实机+夹爪+增强面板）"
    echo "  $0 -m sim                     # 仿真模式"
    echo "  $0 -p simple                  # 使用简单控制面板"
    echo "  $0 -s basic --no-rviz         # 基础安全系统，无界面"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        -g|--no-gripper)
            GRIPPER="false"
            shift
            ;;
        -r|--no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        -l|--layout)
            case "$2" in
                standard|std) RVIZ_LAYOUT="standard" ;;
                optimized|opt) RVIZ_LAYOUT="optimized" ;;
                *) echo -e "${RED}错误: 无效的布局类型 '$2'${NC}"; exit 1 ;;
            esac
            shift 2
            ;;
        -p|--panel)
            case "$2" in
                none) CONTROL_PANEL="none" ;;
                simple) CONTROL_PANEL="simple" ;;
                enhanced|enh) CONTROL_PANEL="enhanced" ;;
                *) echo -e "${RED}错误: 无效的面板类型 '$2'${NC}"; exit 1 ;;
            esac
            shift 2
            ;;
        -s|--safety)
            case "$2" in
                none) SAFETY_SYSTEM="none" ;;
                basic) SAFETY_SYSTEM="basic" ;;
                enhanced|enh) SAFETY_SYSTEM="enhanced" ;;
                *) echo -e "${RED}错误: 无效的安全系统级别 '$2'${NC}"; exit 1 ;;
            esac
            shift 2
            ;;
        --no-collision)
            ENABLE_COLLISION_MONITORING="false"
            shift
            ;;
        --no-smoothing)
            ENABLE_MOTION_SMOOTHING="false"
            shift
            ;;
        --rate)
            SAFETY_CHECK_RATE="$2"
            shift 2
            ;;
        -d|--debug)
            DEBUG="true"
            shift
            ;;
        --pipeline)
            PIPELINE="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知选项 '$1'${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 清理现有ROS进程
echo -e "${YELLOW}清理现有ROS进程...${NC}"
pkill -9 roscore 2>/dev/null
pkill -9 rosmaster 2>/dev/null
sleep 2

# 启动roscore
echo -e "${GREEN}启动ROS核心...${NC}"
roscore &
ROSCORE_PID=$!
sleep 3

# 检查roscore是否启动成功
if ! ps -p $ROSCORE_PID > /dev/null; then
    echo -e "${RED}错误: roscore启动失败${NC}"
    exit 1
fi

# 显示配置信息
echo -e "${BLUE}========== 配置信息 ==========${NC}"
echo -e "运行模式: ${GREEN}$MODE${NC}"
echo -e "使用夹爪: ${GREEN}$GRIPPER${NC}"
echo -e "启动RViz: ${GREEN}$USE_RVIZ${NC}"
if [ "$USE_RVIZ" == "true" ]; then
    echo -e "RViz布局: ${GREEN}$RVIZ_LAYOUT${NC}"
fi
echo -e "控制面板: ${GREEN}$CONTROL_PANEL${NC}"
echo -e "安全系统: ${GREEN}$SAFETY_SYSTEM${NC}"
if [ "$SAFETY_SYSTEM" != "none" ]; then
    echo -e "  - 碰撞监控: ${GREEN}$ENABLE_COLLISION_MONITORING${NC}"
    echo -e "  - 运动平滑: ${GREEN}$ENABLE_MOTION_SMOOTHING${NC}"
    echo -e "  - 检查频率: ${GREEN}${SAFETY_CHECK_RATE}Hz${NC}"
fi
echo -e "规划器: ${GREEN}$PIPELINE${NC}"
echo -e "调试模式: ${GREEN}$DEBUG${NC}"
echo -e "${BLUE}==============================${NC}"
echo ""

# 构建roslaunch命令
LAUNCH_CMD="roslaunch arm_planner main_demo.launch"
LAUNCH_CMD="$LAUNCH_CMD mode:=$MODE"
LAUNCH_CMD="$LAUNCH_CMD gripper:=$GRIPPER"
LAUNCH_CMD="$LAUNCH_CMD use_rviz:=$USE_RVIZ"
LAUNCH_CMD="$LAUNCH_CMD rviz_layout:=$RVIZ_LAYOUT"
LAUNCH_CMD="$LAUNCH_CMD control_panel:=$CONTROL_PANEL"
LAUNCH_CMD="$LAUNCH_CMD safety_system:=$SAFETY_SYSTEM"
LAUNCH_CMD="$LAUNCH_CMD enable_collision_monitoring:=$ENABLE_COLLISION_MONITORING"
LAUNCH_CMD="$LAUNCH_CMD enable_motion_smoothing:=$ENABLE_MOTION_SMOOTHING"
LAUNCH_CMD="$LAUNCH_CMD safety_check_rate:=$SAFETY_CHECK_RATE"
LAUNCH_CMD="$LAUNCH_CMD debug:=$DEBUG"
LAUNCH_CMD="$LAUNCH_CMD pipeline:=$PIPELINE"

# 启动主程序
echo -e "${GREEN}启动MobileManipulator系统...${NC}"
echo -e "${YELLOW}执行命令: $LAUNCH_CMD${NC}"
echo ""

# 捕获退出信号
trap "echo -e '\n${YELLOW}正在关闭系统...${NC}'; kill $ROSCORE_PID 2>/dev/null; exit" INT TERM

# 执行launch文件
$LAUNCH_CMD

# 等待用户中断
wait