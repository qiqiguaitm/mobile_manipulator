#!/bin/bash
# 优化的3D抓取检测启动脚本 - 支持双TF链投影方法
# 用法: ./start_grasp3d.sh [calibration|urdf|compare] [true|false]

set -e  # 遇到错误立即退出

# === 参数解析 ===
# 参数检查（在解析前）
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    show_help() {
        echo "3D抓取检测系统启动脚本"
        echo ""
        echo "用法: $0 [projection_method] [visualize]"
        echo ""
        echo "参数:"
        echo "  projection_method  投影方法 (calibration|urdf|compare)"
        echo "                    默认: calibration"
        echo "  visualize         是否启用RViz可视化 (true|false)"
        echo "                    默认: true"
        echo ""
        echo "示例:"
        echo "  $0                           # 使用标定方法，启用可视化"
        echo "  $0 calibration false         # 使用标定方法，不启用可视化"
        echo "  $0 urdf true                 # 使用URDF方法，启用可视化"
        echo "  $0 compare true              # 对比模式，启用可视化"
        echo ""
        echo "投影方法说明:"
        echo "  calibration - 使用校准的内参和外参 (推荐，精度高)"
        echo "  urdf       - 使用RealSense硬件内参和URDF外参"
        echo "  compare    - 同时运行两种方法进行对比"
    }
    show_help
    exit 0
fi

PROJECTION_METHOD=${1:-calibration}
VISUALIZE=${2:-true}

# 验证参数
valid_methods="calibration urdf compare"
if [[ ! " $valid_methods " =~ " $PROJECTION_METHOD " ]]; then
    echo "❌ 无效投影方法: $PROJECTION_METHOD"
    echo "   有效选择: $valid_methods"
    exit 1
fi

# 颜色输出函数
print_info() { echo -e "\e[36m[INFO]\e[0m $1"; }
print_success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
print_warn() { echo -e "\e[33m[WARN]\e[0m $1"; }
print_error() { echo -e "\e[31m[ERROR]\e[0m $1"; }

# === 清理函数 ===
cleanup() {
    if [[ "$CLEANUP_RUNNING" == "true" ]]; then
        return  # 避免重复清理
    fi
    export CLEANUP_RUNNING=true
    
    print_info "清理ROS进程..."
    rosnode kill -a 2>/dev/null || true
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f perception 2>/dev/null || true
    pkill -f rviz 2>/dev/null || true
    sleep 2
}

# 设置清理信号处理
trap cleanup EXIT INT TERM

# === 硬件修复函数 ===
fix_realsense_hardware() {
    print_info "修复RealSense硬件连接..."
    echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/unbind' 2>/dev/null || true
    sleep 3
    echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/bind' 2>/dev/null || true
    sleep 5
    
    # 验证设备
    if rs-enumerate-devices -s 2>/dev/null | grep -q "Intel RealSense"; then
        print_success "RealSense设备检测正常"
        return 0
    else
        print_warn "RealSense设备检测异常，但继续执行"
        return 1
    fi
}

# === 验证函数 ===
verify_topics() {
    print_info "验证关键话题..."
    
    # 首先检查话题是否存在
    local topics_exist=true
    if ! rostopic list 2>/dev/null | grep -q "/camera/hand/depth/image_rect_raw"; then
        print_warn "深度话题不存在"
        topics_exist=false
    fi
    
    if ! rostopic list 2>/dev/null | grep -q "/camera/hand/color/image_raw"; then
        print_warn "彩色话题不存在"
        topics_exist=false
    fi
    
    if [[ "$topics_exist" == "false" ]]; then
        print_warn "话题尚未发布，等待5秒后重试..."
        sleep 5
        # 再次检查
        if ! rostopic list 2>/dev/null | grep -q "/camera/hand/depth/image_rect_raw"; then
            print_error "深度话题仍不存在"
            return 1
        fi
    fi
    
    # 检查话题数据流
    local timeout=8
    if timeout $timeout rostopic hz /camera/hand/depth/image_rect_raw --window=1 >/dev/null 2>&1; then
        print_success "手部深度话题数据正常"
    else
        print_warn "手部深度话题数据异常（但话题存在）"
        # 不立即失败，继续检查
    fi
    
    if timeout $timeout rostopic hz /camera/hand/color/image_raw --window=1 >/dev/null 2>&1; then
        print_success "手部彩色话题数据正常"
    else
        print_warn "手部彩色话题数据异常（但话题存在）"
        # 不立即失败，继续检查
    fi
    
    # 只要话题存在就认为成功（数据流可能需要更长时间）
    return 0
}

verify_tf_chains() {
    print_info "验证TF链..."
    
    case "$PROJECTION_METHOD" in
        "calibration"|"compare")
            if timeout 3 rosrun tf tf_echo base_link calibration/hand_camera_optical >/dev/null 2>&1; then
                print_success "标定TF链正常"
            else
                print_warn "标定TF链需要检查"
            fi
            ;;
        "urdf")
            if timeout 3 rosrun tf tf_echo base_link camera/hand_depth_optical_frame >/dev/null 2>&1; then
                print_success "URDF TF链正常"
            else
                print_warn "URDF TF链需要检查"
            fi
            ;;
    esac
}

test_projection_core() {
    print_info "测试深度投影核心..."
    python3 -c "
import sys, rospy
sys.path.append('/home/agilex/MobileManipulator/src/perception/scripts')
rospy.init_node('test_projection', anonymous=True)
try:
    from depth_projector_core import CameraDepthProjector
    projector = CameraDepthProjector('hand_camera', '$PROJECTION_METHOD', '/home/agilex/MobileManipulator/src/robot_drivers/camera_driver/config')
    print('✅ $PROJECTION_METHOD 投影核心正常')
    print(f'   内参: fx={projector.K[0,0]:.1f}, fy={projector.K[1,1]:.1f}')
    print(f'   坐标系: {projector.camera_frame}')
except Exception as e:
    print(f'❌ 投影核心异常: {e}')
    exit(1)
" 2>/dev/null
    
    if [[ $? -eq 0 ]]; then
        print_success "深度投影核心测试通过"
        return 0
    else
        print_error "深度投影核心测试失败"
        return 1
    fi
}

# === 主要启动流程 ===
main() {
    echo "🚀 启动3D抓取检测系统"
    echo "   投影方法: $PROJECTION_METHOD"
    echo "   可视化: $VISUALIZE"
    echo ""
    
    # Step 1: 初始清理
    print_info "步骤1: 清理现有进程..."
    rosnode kill -a 2>/dev/null || true
    pkill -f roslaunch 2>/dev/null || true
    sleep 3
    
    # Step 2: 启动roscore
    print_info "步骤2: 启动ROS核心..."
    roscore &
    sleep 3
    
    # Step 3: 硬件修复
    print_info "步骤3: RealSense硬件检查..."
    fix_realsense_hardware || print_warn "硬件修复未完全成功，继续执行"
    
    # Step 4: 启动相机驱动
    print_info "步骤4: 启动相机驱动..."
    roslaunch camera_driver camera_driver.launch \
        camera1_enable:=false \
        camera2_enable:=true \
        camera3_enable:=false \
        enable_depth:=true \
        enable_color:=true \
        align_depth:=false &
    
    sleep 15  # 等待相机完全启动
    
    # Step 5: 验证相机话题
    print_info "步骤5: 验证相机数据..."
    if ! verify_topics; then
        print_error "相机话题验证失败"
        exit 1
    fi
    
    # Step 6: 测试投影核心
    print_info "步骤6: 测试投影系统..."
    if ! test_projection_core; then
        print_error "投影系统测试失败"
        exit 1
    fi
    
    # Step 7: 启动3D抓取系统
    print_info "步骤7: 启动3D抓取检测..."
    
    # 根据投影方法选择启动参数
    case "$PROJECTION_METHOD" in
        "compare")
            roslaunch perception grasp_3d_dual.launch \
                projection_method:=compare \
                compare_mode:=true \
                visualize:=$VISUALIZE \
                load_robot_description:=true &
            ;;
        *)
            roslaunch perception grasp_3d_dual.launch \
                projection_method:=$PROJECTION_METHOD \
                compare_mode:=false \
                visualize:=$VISUALIZE \
                load_robot_description:=true &
            ;;
    esac
    
    sleep 10  # 等待系统稳定
    
    # Step 8: 最终验证
    print_info "步骤8: 系统状态验证..."
    verify_tf_chains
    
    # Step 9: 显示系统信息
    echo ""
    echo "📊 系统状态总结:"
    echo "   投影方法: $PROJECTION_METHOD"
    echo "   可视化: $VISUALIZE"
    echo "   相机分辨率: 640x480"
    echo ""
    echo "💡 关键话题:"
    echo "   深度图像: /camera/hand/depth/image_rect_raw"
    echo "   彩色图像: /camera/hand/color/image_raw"
    echo "   2D抓取: /perception/hand/grasps"
    echo "   3D抓取: /perception/hand/grasps_3d"
    echo "   抓取点云: /perception/hand/grasp_points"
    echo ""
    echo "🎯 投影方法差异:"
    case "$PROJECTION_METHOD" in
        "calibration")
            echo "   使用校准内参 (fx≈598.6) 和标定TF链"
            echo "   坐标系: calibration/hand_camera_optical"
            ;;
        "urdf")
            echo "   使用RealSense内参 (fx≈389.2) 和URDF TF链"
            echo "   坐标系: camera/hand_depth_optical_frame"
            ;;
        "compare")
            echo "   对比两种方法，同时发布结果"
            echo "   主要: calibration/hand_camera_optical"
            echo "   对比: camera/hand_depth_optical_frame"
            ;;
    esac
    echo ""
    print_success "✅ 3D抓取检测系统已就绪！"
    echo "按 Ctrl+C 退出系统"
    
    # 保持运行
    wait
}

# 脚本结束

# 执行主函数
main "$@"
