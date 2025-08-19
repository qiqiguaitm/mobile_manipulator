#!/bin/bash
#
# MobileManipulator 模块化启动脚本  
# 基于 Linus "Never break userspace" 原则：保持向后兼容，优化启动流程
# Usage: ./run_all.sh [all|demo|arm|chassis|perception|sensors|slam|stop|list]
#

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 启动配置定义 - 清晰的数据结构
readonly SENSORS_MODULES="camera_driver lidar_driver imu_driver realsense2_camera"
readonly CONTROL_MODULES="arm_controller chassis_controller"  
readonly PLANNING_MODULES="arm_planner chassis_planner"
readonly PERCEPTION_MODULES="perception object_tracker target_filter"
readonly INTEGRATION_MODULES="task_mgr task_router slam odom"

# 环境初始化 - 统一配置点
init_environment() {
    echo "=== 初始化运行环境 ==="
    
    # 清理已有ROS进程 - 保证干净启动环境
    echo "清理现有ROS进程..."
    pkill -f roscore || true
    pkill -f rosmaster || true
    pkill -f roslaunch || true
    sleep 2
    
    # 启动roscore
    echo "启动ROS核心..."
    roscore &
    sleep 3
    
    # 配置环境
    source /opt/ros/noetic/setup.bash
    if [ -f "devel/setup.bash" ]; then
        source devel/setup.bash
        echo "✅ 工作空间环境已加载"
    else
        echo "⚠️  未找到devel/setup.bash，请先运行构建"
    fi
}

# 启动模块组 - 单一职责函数
launch_modules() {
    local module_group="$1"
    local modules="$2"
    
    echo "=== 启动 $module_group ==="
    
    for module in $modules; do
        local launch_file=""
        
        # 智能查找launch文件
        case "$module" in
            "lidar_driver")      launch_file="lidar_driver/launch/lidar_driver_simple.launch" ;;
            "imu_driver")        launch_file="imu_driver/launch/imu_driver.launch" ;;
            "realsense2_camera") launch_file="realsense2_camera/launch/realsense_cameras.launch" ;;
            "arm_controller")    launch_file="arm_controller/launch/start_single_piper.launch" ;;
            "tracer_base") launch_file="chassis_controller/tracer_base/launch/tracer_base.launch" ;;
            "arm_planner")       launch_file="arm_planner/launch/arm_planner.launch" ;;
            "chassis_planner")   launch_file="chassis_planner/launch/chassis_planner.launch" ;;
            "perception")        launch_file="perception/launch/perception.launch" ;;
            "object_tracker")    launch_file="object_tracker/launch/object_tracker.launch" ;;
            "target_filter")     launch_file="target_filter/launch/target_filter.launch" ;;
            "task_mgr")          launch_file="task_mgr/launch/task_mgr.launch" ;;
            "task_router")       launch_file="task_router/launch/task_router.launch" ;;
            "slam")              launch_file="slam/launch/slam.launch" ;;
            "odom")              launch_file="odom/launch/odom.launch" ;;
        esac
        
        if [ -n "$launch_file" ] && [ -f "src/$launch_file" ]; then
            echo "启动 $module..."
            roslaunch "$launch_file" &
            sleep 2  # 避免启动冲突
        else
            echo "⚠️  未找到 $module 的launch文件"
        fi
    done
    
    echo "✅ $module_group 启动完成"
}

# 启动完整机器人系统
launch_full_system() {
    echo "=== 启动完整机器人系统 ==="
    
    # 按依赖顺序启动
    launch_modules "传感器模块" "$SENSORS_MODULES"
    sleep 3
    
    launch_modules "控制模块" "$CONTROL_MODULES"  
    sleep 3
    
    launch_modules "规划模块" "$PLANNING_MODULES"
    sleep 3
    
    launch_modules "感知模块" "$PERCEPTION_MODULES"
    sleep 3
    
    launch_modules "集成模块" "$INTEGRATION_MODULES"
    
    echo "🎉 完整系统启动完成！"
    echo "💡 使用 './run_all.sh stop' 停止所有模块"
}

# 启动演示模式
launch_demo() {
    echo "=== 启动演示模式 ==="
    if [ -f "src/agilex_demo.launch" ]; then
        roslaunch src/agilex_demo.launch
    else
        echo "未找到演示launch文件，启动核心模块..."
        launch_modules "机械臂演示" "arm_controller arm_planner"
    fi
}

# 停止所有模块
stop_all() {
    echo "=== 停止所有ROS模块 ==="
    pkill -f roslaunch || true
    pkill -f roscore || true
    pkill -f rosmaster || true
    echo "✅ 所有模块已停止"
}

# 显示帮助信息
show_help() {
    cat << EOF
MobileManipulator 模块化启动脚本

用法: $0 [选项]

系统启动:
  all        启动完整机器人系统 (推荐)
  demo       启动演示模式

模块组启动:
  sensors    传感器模块 (相机、激光雷达、IMU)
  arm        机械臂模块 (控制器 + 规划器)  
  chassis    底盘模块 (控制器 + 规划器)
  perception 感知模块 (感知、跟踪、过滤)
  slam       SLAM定位导航

系统管理:
  stop       停止所有ROS模块
  list       显示此帮助信息

示例:
  $0         # 启动完整系统
  $0 arm     # 只启动机械臂相关模块
  $0 stop    # 停止所有模块
EOF
}

# 主逻辑 - 清晰的控制流
main() {
    case "${1:-all}" in
        "all")        init_environment; launch_full_system ;;
        "demo")       init_environment; launch_demo ;;
        "sensors")    init_environment; launch_modules "传感器模块" "$SENSORS_MODULES" ;;
        "arm")        init_environment; launch_modules "机械臂模块" "$CONTROL_MODULES $PLANNING_MODULES" ;;
        "chassis")    init_environment; launch_modules "底盘模块" "chassis_controller chassis_planner" ;;
        "perception") init_environment; launch_modules "感知模块" "$PERCEPTION_MODULES" ;;
        "slam")       init_environment; launch_modules "SLAM模块" "slam odom" ;;
        "stop")       stop_all ;;
        "list"|"help"|"-h") show_help ;;
        
        # 向后兼容旧接口 - Never break userspace
        "camera")     init_environment; launch_modules "相机驱动" "camera_driver realsense2_camera" ;;
        "lidar")      init_environment; launch_modules "激光雷达" "lidar_driver" ;;
        "imu")        init_environment; launch_modules "IMU驱动" "imu_driver" ;;
        
        *) 
            echo "❌ 未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

# 信号处理 - 优雅退出
trap 'echo "收到中断信号，正在停止..."; stop_all; exit 0' INT TERM

main "$@"
