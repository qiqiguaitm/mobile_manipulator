#!/bin/bash
#
# MobileManipulator 分层构建脚本
# 基于 Linus "好品味" 原则：消除特殊情况，统一构建工具
# Usage: ./build_all.sh [all|l1|l2|l3|l4|l5|description|drivers|planners|perception|integration|clean|list]
#

set -euo pipefail  # 严格错误处理

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 分层包定义 - 数据结构即算法
readonly L1_DESCRIPTION="mobile_manipulator2_description piper_description tracer2_description realsense2_description"
readonly L2_DRIVERS="arm_controller chassis_controller camera_driver lidar_driver imu_driver realsense2_camera"  
readonly L3_PLANNERS="arm_planner chassis_planner"
readonly L4_PERCEPTION="perception object_tracker target_filter inference_abstraction"
readonly L5_INTEGRATION="task_mgr task_router slam odom sensor_calibration"

# 环境初始化 - 单一配置点
init_environment() {
    echo "=== 初始化构建环境 ==="
    source /opt/ros/noetic/setup.bash
    
    # 配置catkin工作空间
    if ! catkin config --workspace . >/dev/null 2>&1; then
        echo "初始化catkin工作空间..."
        catkin init
        catkin config --extend /opt/ros/noetic
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin config --jobs "$(nproc)"
    fi
}

# 构建函数 - 单一职责
build_layer() {
    local layer_name="$1"
    local packages="$2"
    
    if [ -z "$packages" ]; then
        echo "警告: $layer_name 层无包可构建"
        return 0
    fi
    
    echo "=== 构建 $layer_name 层 ==="
    echo "包列表: $packages"
    
    # shellcheck disable=SC2086
    catkin build $packages --no-deps
    
    echo "✅ $layer_name 层构建完成"
}

# 清理函数
clean_workspace() {
    echo "=== 清理工作空间 ==="
    catkin clean --yes
    echo "✅ 清理完成"
}

# 构建所有层 - 按依赖顺序
build_all_layers() {
    echo "=== 开始分层构建 ==="
    build_layer "L1-描述文件" "$L1_DESCRIPTION"
    build_layer "L2-硬件驱动" "$L2_DRIVERS"  
    build_layer "L3-规划模块" "$L3_PLANNERS"
    build_layer "L4-感知模块" "$L4_PERCEPTION"
    build_layer "L5-集成模块" "$L5_INTEGRATION"
    echo "🎉 全部构建完成！"
}

# 显示帮助信息
show_help() {
    cat << EOF
MobileManipulator 分层构建脚本

用法: $0 [选项]

分层选项:
  all             完整分层构建 (推荐)
  l1, description L1层: 机器人描述文件
  l2, drivers     L2层: 硬件驱动模块  
  l3, planners    L3层: 运动规划模块
  l4, perception  L4层: 感知处理模块
  l5, integration L5层: 系统集成模块

维护选项:
  clean           清理构建缓存
  list            显示此帮助信息

示例:
  $0              # 完整构建
  $0 l1           # 只构建描述文件层
  $0 clean        # 清理后重新构建
EOF
}

# 主逻辑 - 无条件分支，好品味的体现
main() {
    init_environment
    
    case "${1:-all}" in
        "all")          build_all_layers ;;
        "l1"|"description") build_layer "L1-描述文件" "$L1_DESCRIPTION" ;;
        "l2"|"drivers")     build_layer "L2-硬件驱动" "$L2_DRIVERS" ;;
        "l3"|"planners")    build_layer "L3-规划模块" "$L3_PLANNERS" ;;  
        "l4"|"perception")  build_layer "L4-感知模块" "$L4_PERCEPTION" ;;
        "l5"|"integration") build_layer "L5-集成模块" "$L5_INTEGRATION" ;;
        "clean")        clean_workspace; build_all_layers ;;
        "list"|"help"|"-h") show_help ;;
        *) 
            echo "❌ 未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
