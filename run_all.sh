#!/bin/bash
#
# MobileManipulator 模块化启动脚本  
# 基于 Linus "Never break userspace" 原则：保持向后兼容，优化启动流程
# Usage: ./run_all.sh [all|demo|arm|chassis|perception|sensors|slam|stop|list]
#

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 启动配置定义 - 对应标准7层架构
readonly L2_DRIVER_MODULES="camera_driver lidar_driver imu_driver realsense2_camera"
readonly L3_CONTROLLER_MODULES="arm_controller chassis_controller tracer_base"  
readonly L4_ODOM_MODULES="odom slam rf2o_laser_odometry"
readonly L5_PERCEPTION_MODULES="perception object_tracker target_filter"
readonly L6_PLANNER_MODULES="arm_planner chassis_planner"
readonly L7_INTEGRATION_MODULES="task_mgr task_router"

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
            "camera_driver") launch_file="camera_driver/launch/camera_driver.launch" ;;
            "arm_controller")    launch_file="arm_controller/launch/arm_controller.launch" ;;
            "chassis_controller")    launch_file="chassis_controller/launch/chassis_controller.launch" ;;
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

# 启动完整机器人系统 - 按标准7层顺序
launch_full_system() {
    echo "=== 启动完整机器人系统（7层架构）==="
    
    # 按标准7层依赖顺序启动
    echo "启动L2层: 硬件驱动"
    launch_modules "L2-硬件驱动" "$L2_DRIVER_MODULES"
    sleep 3
    
    echo "启动L3层: 控制器"
    launch_modules "L3-控制器" "$L3_CONTROLLER_MODULES"  
    sleep 3
    
    echo "启动L4层: 里程计/SLAM"
    launch_modules "L4-里程计" "$L4_ODOM_MODULES"
    sleep 3
    
    echo "启动L5层: 感知模块"
    launch_modules "L5-感知" "$L5_PERCEPTION_MODULES"
    sleep 3
    
    echo "启动L6层: 规划模块"
    launch_modules "L6-规划" "$L6_PLANNER_MODULES"
    sleep 3
    
    echo "启动L7层: 集成模块"
    launch_modules "L7-集成" "$L7_INTEGRATION_MODULES"
    
    echo "🎉 完整7层系统启动完成！"
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
  all        启动完整7层机器人系统 (推荐)
  demo       启动演示模式

标准7层模块启动:
  l2, drivers     L2层: 硬件驱动模块
  l3, controllers L3层: 控制器模块
  l4, odom        L4层: 里程计/SLAM模块
  l5, perception  L5层: 感知处理模块  
  l6, planners    L6层: 运动规划模块
  l7, integration L7层: 系统集成模块

快捷启动 (向后兼容):
  sensors    传感器模块 (等同于l2)
  arm        机械臂模块 (l3+l6机械臂部分)  
  chassis    底盘模块 (l3+l6底盘部分)
  slam       SLAM定位导航 (等同于l4)

系统管理:
  stop       停止所有ROS模块
  list       显示此帮助信息

示例:
  $0         # 启动完整系统
  $0 arm     # 只启动机械臂相关模块
  $0 stop    # 停止所有模块
EOF
}

# 主逻辑 - 清晰的控制流，符合7层架构
main() {
    case "${1:-all}" in
        "all")                  init_environment; launch_full_system ;;
        "demo")                 init_environment; launch_demo ;;
        
        # 标准7层启动
        "l2"|"drivers")         init_environment; launch_modules "L2-硬件驱动" "$L2_DRIVER_MODULES" ;;
        "l3"|"controllers")     init_environment; launch_modules "L3-控制器" "$L3_CONTROLLER_MODULES" ;;
        "l4"|"odom")           init_environment; launch_modules "L4-里程计" "$L4_ODOM_MODULES" ;;
        "l5"|"perception")      init_environment; launch_modules "L5-感知" "$L5_PERCEPTION_MODULES" ;;
        "l6"|"planners")        init_environment; launch_modules "L6-规划" "$L6_PLANNER_MODULES" ;;
        "l7"|"integration")     init_environment; launch_modules "L7-集成" "$L7_INTEGRATION_MODULES" ;;
        
        # 向后兼容旧接口 - Never break userspace
        "sensors")              init_environment; launch_modules "传感器模块" "$L2_DRIVER_MODULES" ;;
        "arm")                  init_environment; launch_modules "机械臂模块" "arm_controller arm_planner" ;;
        "chassis")              init_environment; launch_modules "底盘模块" "chassis_controller tracer_base chassis_planner" ;;
        "slam")                 init_environment; launch_modules "SLAM模块" "$L4_ODOM_MODULES" ;;
        
        # 向后兼容更细粒度接口
        "camera")               init_environment; launch_modules "相机驱动" "camera_driver realsense2_camera" ;;
        "lidar")                init_environment; launch_modules "激光雷达" "lidar_driver" ;;
        "imu")                  init_environment; launch_modules "IMU驱动" "imu_driver" ;;
        
        "stop")                 stop_all ;;
        "list"|"help"|"-h")     show_help ;;
        
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
