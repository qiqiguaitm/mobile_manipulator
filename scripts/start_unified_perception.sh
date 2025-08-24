#!/bin/bash

# 统一感知系统启动脚本
# 使用方法: ./start_unified_perception.sh [projection_method] [camera_config] [api_config] [debug_mode]
# 
# 参数说明:
#   projection_method: urdf, calibration (默认: urdf)
#   camera_config: hand, chassis, top, all, none (默认: hand)  
#   api_config: grasp, refer, all (默认: grasp)
#   debug_mode: true, false (默认: true)
#
# 示例:
#   ./start_unified_perception.sh urdf hand grasp true    # 手部相机 + 抓取检测
#   ./start_unified_perception.sh urdf all all true       # 所有相机 + 所有API
#   ./start_unified_perception.sh calibration chassis grasp false  # 底盘相机 + 抓取检测
#
# 特性:
#   - 自动检测和修复conda环境中的libffi符号冲突问题
#   - 支持640x480分辨率的RealSense相机
#   - 集成HTTP referring_dino API支持

set -e

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ===== libffi修复逻辑 =====
# 修复conda环境中libffi版本冲突导致的图像转换错误
fix_libffi_environment() {
    echo "🔧 检查和修复libffi环境..."
    
    # 检测是否存在conda环境冲突
    if [[ "$PATH" == *"miniconda"* ]] || [[ "$PATH" == *"conda"* ]]; then
        echo "⚠️ 检测到conda环境，应用libffi修复..."
        
        # 保存原始环境变量
        export ORIGINAL_PATH="$PATH"
        export ORIGINAL_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
        
        # 临时调整环境变量，优先使用系统库
        export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
        export LD_LIBRARY_PATH="/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
        export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH"
        
        echo "  📍 使用Python: $(which python3)"
        echo "  📍 Python版本: $(python3 --version)"
        
        # 快速测试cv_bridge是否工作
        if python3 -c "
try:
    from cv_bridge import CvBridge
    import numpy as np
    bridge = CvBridge()
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    ros_image = bridge.cv2_to_imgmsg(img, 'bgr8')
    print('✅ libffi环境修复成功')
except Exception as e:
    print(f'❌ libffi环境修复失败: {e}')
    exit(1)
" 2>/dev/null; then
            echo "  ✅ 图像转换功能正常"
        else
            echo "  ❌ 图像转换功能异常，但继续启动..."
        fi
    else
        echo "  ✅ 环境正常，无需修复"
    fi
}

# 应用libffi修复
fix_libffi_environment

# 参数解析
PROJECTION_METHOD=${1:-"urdf"}    # urdf 或 calibration
CAMERA_CONFIG=${2:-"hand"}        # hand, chassis, top, all, none
API_CONFIG=${3:-"grasp"}          # grasp, refer, all  
DEBUG_MODE=${4:-"true"}           # true 或 false

# 参数验证
validate_params() {
    # 验证投影方法
    case "$PROJECTION_METHOD" in
        "urdf"|"calibration") ;;
        *) echo "❌ 无效的投影方法: $PROJECTION_METHOD (支持: urdf, calibration)"; exit 1 ;;
    esac
    
    # 验证相机配置
    case "$CAMERA_CONFIG" in
        "hand"|"chassis"|"top"|"all"|"none") ;;
        *) echo "❌ 无效的相机配置: $CAMERA_CONFIG (支持: hand, chassis, top, all, none)"; exit 1 ;;
    esac
    
    # 验证API配置
    case "$API_CONFIG" in
        "grasp"|"refer"|"all") ;;
        *) echo "❌ 无效的API配置: $API_CONFIG (支持: grasp, refer, all)"; exit 1 ;;
    esac
    
    # 验证调试模式
    case "$DEBUG_MODE" in
        "true"|"false") ;;
        *) echo "❌ 无效的调试模式: $DEBUG_MODE (支持: true, false)"; exit 1 ;;
    esac
}

# 调用参数验证
validate_params

echo "=================================================="
echo "🚀 启动统一感知系统"
echo "=================================================="
echo "投影方法: $PROJECTION_METHOD"
echo "相机配置: $CAMERA_CONFIG" 
echo "API配置: $API_CONFIG"
echo "调试模式: $DEBUG_MODE"
echo "=================================================="

# 环境检查
echo "🔍 检查环境..."

# ROS环境检查
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS环境未设置"
    exit 1
fi

# 检查工作空间
cd "$PROJECT_DIR"
if [ ! -f "devel/setup.bash" ]; then
    echo "⚠️ 工作空间未构建，执行构建..."
    catkin_make
fi

# source工作空间
source devel/setup.bash

# 检查配置文件
CONFIG_FILE="$PROJECT_DIR/src/perception/config/perception_unified.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "✅ 环境检查完成"

# 清理现有ROS进程
echo "🧹 清理现有ROS进程..."

# 杀死相关节点（温和方式）
rosnode list 2>/dev/null | grep -E "(unified_perception|hand_grasp|depth_grasp|grasp_3d)" | while read node; do
    echo "停止节点: $node"
    rosnode kill "$node" 2>/dev/null || true
done

# 杀死相机驱动节点
rosnode list 2>/dev/null | grep -E "camera" | while read node; do
    echo "停止相机节点: $node"  
    rosnode kill "$node" 2>/dev/null || true
done

# 短暂等待
sleep 2

echo "✅ ROS进程清理完成"

# 动态配置生成
generate_runtime_config() {
    echo "⚙️ 生成运行时配置..."
    
    # 创建临时配置文件
    TEMP_CONFIG="/tmp/perception_unified_runtime.yaml"
    cp "$CONFIG_FILE" "$TEMP_CONFIG"
    
    # Python脚本动态修改配置
    python3 - <<EOF
import yaml
import sys

config_file = "$TEMP_CONFIG"
camera_config = "$CAMERA_CONFIG"
api_config = "$API_CONFIG"

# 加载配置
with open(config_file, 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 相机启用配置
camera_enable = {
    'hand': camera_config in ['hand', 'all'],
    'chassis': camera_config in ['chassis', 'all'], 
    'top': camera_config in ['top', 'all']
}

for camera_name, enabled in camera_enable.items():
    if camera_name in config['cameras']:
        config['cameras'][camera_name]['enabled'] = enabled

# API启用配置  
api_enable = {
    'grasp_anything': api_config in ['grasp', 'all'],
    'referring_dino': api_config in ['refer', 'all']
}

for api_name, enabled in api_enable.items():
    if api_name in config['detection_apis']:
        config['detection_apis'][api_name]['enabled'] = enabled

# 流水线启用配置
# 根据相机、API和投影方法配置动态启用流水线
projection_method = "$PROJECTION_METHOD"

# 确定流水线后缀（投影方法）
if projection_method == "calibration":
    projection_suffix = "calib"
else:
    projection_suffix = "urdf"

pipeline_rules = {
    # 抓取检测流水线
    f'hand_grasp_{projection_suffix}': camera_enable['hand'] and api_enable['grasp_anything'],
    f'chassis_grasp_{projection_suffix}': camera_enable['chassis'] and api_enable['grasp_anything'],  
    f'top_grasp_{projection_suffix}': camera_enable['top'] and api_enable['grasp_anything'],
    
    # Referring检测流水线
    f'hand_refer_{projection_suffix}': camera_enable['hand'] and api_enable['referring_dino'],
    f'chassis_refer_{projection_suffix}': camera_enable['chassis'] and api_enable['referring_dino'],
    f'top_refer_{projection_suffix}': camera_enable['top'] and api_enable['referring_dino']
}

# 首先禁用所有流水线
for pipeline_name in config['pipelines']:
    config['pipelines'][pipeline_name]['enabled'] = False

# 然后只启用匹配的流水线
for pipeline_name, should_enable in pipeline_rules.items():
    if pipeline_name in config['pipelines']:
        config['pipelines'][pipeline_name]['enabled'] = should_enable

# 保存修改后的配置
with open(config_file, 'w', encoding='utf-8') as f:
    yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

print(f"✅ 运行时配置已生成: {config_file}")
print(f"  启用的相机: {[k for k, v in camera_enable.items() if v]}")
print(f"  启用的API: {[k for k, v in api_enable.items() if v]}")  
print(f"  启用的流水线: {[k for k, v in pipeline_rules.items() if v]}")
EOF

    # 更新配置文件路径
    CONFIG_FILE="$TEMP_CONFIG"
}

# 硬件检查和修复（如果启用相机）
hardware_check() {
    if [ "$CAMERA_CONFIG" != "none" ]; then
        echo "🔧 检查和修复RealSense相机硬件..."
        
        # 检查RealSense设备
        if ! rs-enumerate-devices -s >/dev/null 2>&1; then
            echo "⚠️ RealSense设备检测失败，尝试修复..."
            
            # USB Hub复位
            echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/unbind' 2>/dev/null || true
            sleep 3
            echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/bind' 2>/dev/null || true
            sleep 5
            
            # UVC驱动重新加载
            echo "agx" | sudo -S modprobe -r uvcvideo 2>/dev/null || true
            echo "agx" | sudo -S modprobe uvcvideo 2>/dev/null || true
            sleep 3
            
            # 再次检查
            if ! rs-enumerate-devices -s >/dev/null 2>&1; then
                echo "❌ RealSense相机修复失败，但继续启动（可能影响功能）"
            else
                echo "✅ RealSense相机修复成功"
            fi
        else
            echo "✅ RealSense相机状态正常"
        fi
    else
        echo "ℹ️ 无相机模式，跳过硬件检查"
    fi
}

# 生成运行时配置
generate_runtime_config

# 硬件检查
#hardware_check

# 启动统一感知系统
echo "🚀 启动统一感知系统..."

# 确定是否需要启动相机驱动
START_CAMERAS="false"
if [ "$CAMERA_CONFIG" != "none" ]; then
    START_CAMERAS="true"
fi

# 确定相机驱动配置
determine_camera_launch_config() {
    case "$CAMERA_CONFIG" in
        "hand") echo "hand_only" ;;
        "chassis") echo "chassis_only" ;;
        "top") echo "top_only" ;;
        "all") echo "all" ;;
        "none") echo "none" ;;
        *) echo "hand_only" ;;  # 默认
    esac
}

CAMERA_LAUNCH_CONFIG=$(determine_camera_launch_config)

# 构建roslaunch命令
LAUNCH_CMD="roslaunch perception perception_unified.launch"
LAUNCH_CMD="$LAUNCH_CMD projection_method:=$PROJECTION_METHOD"
LAUNCH_CMD="$LAUNCH_CMD start_cameras:=$START_CAMERAS"
LAUNCH_CMD="$LAUNCH_CMD debug_mode:=$DEBUG_MODE"
LAUNCH_CMD="$LAUNCH_CMD enable_visualization:=true"
LAUNCH_CMD="$LAUNCH_CMD config_file:=$CONFIG_FILE"

# 相机驱动配置
if [ "$START_CAMERAS" == "true" ]; then
    LAUNCH_CMD="$LAUNCH_CMD camera_config:=$CAMERA_LAUNCH_CONFIG"
fi

echo "执行命令: $LAUNCH_CMD"
echo "配置文件: $CONFIG_FILE"
echo "=================================================="
echo "📊 RViz显示建议："

# 根据配置显示RViz建议
if [ "$CAMERA_CONFIG" != "none" ]; then
    case "$CAMERA_CONFIG" in
        "hand")
            echo "  • 启用 'Hand Camera' 显示组"
            ;;
        "chassis")
            echo "  • 启用 'Chassis Camera' 显示组"
            ;;
        "top")
            echo "  • 启用 'Top Camera' 显示组"
            ;;
        "all")
            echo "  • 启用所有Camera显示组"
            ;;
    esac
    
    case "$API_CONFIG" in
        "grasp")
            echo "    - 显示Grasp检测结果（绿色）"
            ;;
        "refer")
            echo "    - 显示Referring检测结果（黄色）"
            ;;
        "all")
            echo "    - 显示Grasp检测结果（绿色）"
            echo "    - 显示Referring检测结果（黄色）"
            ;;
    esac
else
    echo "  • 无相机模式 - RViz仅显示机器人模型"
fi

echo "=================================================="

# 执行launch
exec $LAUNCH_CMD
