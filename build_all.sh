#!/bin/bash
#
# MobileManipulator 分层构建脚本
# 基于 Linus "好品味" 原则：消除特殊情况，统一构建工具
# Usage: ./build_all.sh [all|l0|l1|l2|l3|l4|l5|description|drivers|planners|perception|integration|clean|list]
#

set -euo pipefail  # 严格错误处理

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 标准7层架构定义 - 数据结构即算法
readonly L1_DESCRIPTION="mobile_manipulator2_description piper_description tracer2_description realsense2_description"
readonly L2_DRIVERS="rslidar_sdk lidar_driver imu_driver realsense2_camera camera_driver"  
readonly L3_CONTROLLERS="ugv_sdk tracer_msgs tracer_base chassis_controller arm_controller"
readonly L4_ODOM="rf2o_laser_odometry odom slam"
readonly L5_PERCEPTION="sensor_calibration inference_abstraction perception object_tracker target_filter"
readonly L6_PLANNERS="arm_planner chassis_planner task_router"
readonly L7_INTEGRATION="task_mgr"

# 颜色定义 - 更清晰的输出
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[0;33m'
readonly NC='\033[0m' # No Color

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# 嵌套包映射表 - 智能包管理
readonly NESTED_PACKAGES=(
    "ugv_sdk:chassis_controller/ugv_sdk"
    "tracer_msgs:chassis_controller/tracer_msgs"
    "tracer_base:chassis_controller/tracer_base"
    "rslidar_sdk:robot_drivers/lidar_driver/rslidar_sdk"
    "realsense2_camera:robot_drivers/camera_driver/realsense2_camera"
    "realsense2_description:robot_drivers/camera_driver/realsense2_description"
)

# 动态符号链接管理 - 优雅解决嵌套包问题
create_temp_links() {
    log_info "创建智能符号链接..."
    for mapping in "${NESTED_PACKAGES[@]}"; do
        local pkg_name="${mapping%%:*}"
        local pkg_path="${mapping##*:}"
        
        if [[ -d "src/$pkg_path" && ! -e "src/$pkg_name" ]]; then
            ln -sf "./$pkg_path" "src/$pkg_name"
            log_info "  ✅ $pkg_name -> $pkg_path"
        fi
    done
}

# 清理临时符号链接
cleanup_temp_links() {
    for mapping in "${NESTED_PACKAGES[@]}"; do
        local pkg_name="${mapping%%:*}"
        if [[ -L "src/$pkg_name" ]]; then
            rm "src/$pkg_name" 2>/dev/null || true
        fi
    done
}

# 环境初始化 - 单一配置点
init_environment() {
    log_info "初始化构建环境"
    
    # 设置清理陷阱 - 确保退出时清理
    trap cleanup_temp_links EXIT INT TERM
    
    # 检查 ROS 环境
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        log_error "ROS Noetic 未安装！"
        exit 1
    fi
    
    source /opt/ros/noetic/setup.bash
    
    # 配置catkin工作空间
    if ! catkin config --workspace . >/dev/null 2>&1; then
        log_info "初始化catkin工作空间..."
        catkin init
        catkin config --extend /opt/ros/noetic
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin config --jobs "$(nproc)"
    fi
    
    # 创建智能链接
    create_temp_links
    
    # 检查必要的系统依赖
    check_dependencies
}

# 依赖检查函数
check_dependencies() {
    local missing_deps=()
    
    # 检查必要的命令
    for cmd in catkin cmake python3; do
        if ! command -v "$cmd" &> /dev/null; then
            missing_deps+=("$cmd")
        fi
    done
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        log_error "缺少必要的依赖: ${missing_deps[*]}"
        log_info "请安装缺失的依赖后重试"
        exit 1
    fi
}

# 构建函数 - 单一职责
build_layer() {
    local layer_name="$1"
    local packages="$2"
    
    if [ -z "$packages" ]; then
        log_warn "$layer_name 层无包可构建"
        return 0
    fi
    
    log_info "构建 $layer_name 层"
    log_info "包列表: $packages"
    
    # 验证包存在性 - 让catkin处理包发现
    local available_packages
    available_packages=$(catkin list 2>/dev/null)
    local valid_packages=""
    
    for pkg in $packages; do
        if echo "$available_packages" | grep -q "^- $pkg$"; then
            valid_packages="$valid_packages $pkg"
        else
            log_warn "包 '$pkg' 未被catkin识别，跳过"
        fi
    done
    
    if [ -z "$valid_packages" ]; then
        log_warn "$layer_name 层没有有效的包可构建"
        return 0
    fi
    
    # shellcheck disable=SC2086
    if catkin build $valid_packages --no-deps; then
        log_info "✅ $layer_name 层构建完成"
    else
        log_error "❌ $layer_name 层构建失败"
        return 1
    fi
}

# 清理函数
clean_workspace() {
    log_info "清理工作空间"
    if catkin clean --yes; then
        log_info "✅ 清理完成"
    else
        log_error "❌ 清理失败"
        return 1
    fi
}

# 构建所有层 - 按标准7层依赖顺序
build_all_layers() {
    log_info "开始标准7层构建"
    
    local layers=(
        "L1-描述文件:$L1_DESCRIPTION"
        "L2-硬件驱动:$L2_DRIVERS"
        "L3-控制器层:$L3_CONTROLLERS"
        "L4-里程计层:$L4_ODOM"
        "L5-感知模块:$L5_PERCEPTION"
        "L6-规划模块:$L6_PLANNERS"
        "L7-集成模块:$L7_INTEGRATION"
    )
    
    local failed=0
    for layer in "${layers[@]}"; do
        IFS=':' read -r name packages <<< "$layer"
        if ! build_layer "$name" "$packages"; then
            ((failed++))
        fi
    done
    
    if [ $failed -eq 0 ]; then
        log_info "🎉 全部7层构建完成！"
    else
        log_error "❌ 有 $failed 个层构建失败"
        return 1
    fi
}

# 显示帮助信息
show_help() {
    cat << EOF
MobileManipulator 分层构建脚本

用法: $0 [选项]

标准7层选项:
  all             完整7层构建 (推荐)
  l1, description L1层: 机器人描述文件
  l2, drivers     L2层: 硬件驱动模块  
  l3, controllers L3层: 控制器模块
  l4, odom        L4层: 里程计/SLAM
  l5, perception  L5层: 感知处理模块
  l6, planners    L6层: 运动规划模块
  l7, integration L7层: 系统集成模块

维护选项:
  clean           清理构建缓存
  list            显示此帮助信息

示例:
  $0              # 完整构建
  $0 l1           # 只构建描述文件层
  $0 clean        # 清理构建缓存

特殊选项:
  快速模式: FAST=1 $0    # 跳过依赖检查
  调试模式: DEBUG=1 $0   # 显示详细日志
EOF
}

# 主逻辑 - 无条件分支，好品味的体现
main() {
    init_environment
    
    case "${1:-all}" in
        "all")              build_all_layers ;;
        "l1"|"description") build_layer "L1-描述文件" "$L1_DESCRIPTION" ;;
        "l2"|"drivers")     build_layer "L2-硬件驱动" "$L2_DRIVERS" ;;
        "l3"|"controllers") build_layer "L3-控制器层" "$L3_CONTROLLERS" ;;  
        "l4"|"odom")        build_layer "L4-里程计层" "$L4_ODOM" ;;
        "l5"|"perception")  build_layer "L5-感知模块" "$L5_PERCEPTION" ;;
        "l6"|"planners")    build_layer "L6-规划模块" "$L6_PLANNERS" ;;
        "l7"|"integration") build_layer "L7-集成模块" "$L7_INTEGRATION" ;;
        "clean")            clean_workspace ;;
        "list"|"help"|"-h") show_help ;;
        *) 
            echo "❌ 未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
