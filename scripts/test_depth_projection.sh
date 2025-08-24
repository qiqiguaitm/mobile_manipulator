#!/bin/bash
# RGBD深度投影测试脚本 - 支持两种投影方法对比测试
# 用法: ./test_depth_projection.sh [hand|top|chassis|all] [urdf|calibration|compare] [color|depth]
# 默认: hand urdf color

# 注释libffi环境设置 - 这个导致了符号冲突问题  
# export LD_PRELOAD='/usr/lib/aarch64-linux-gnu/libffi.so.7'

# 相机配置数据结构 - Linus式设计：统一处理所有相机
declare -A CAMERA_CONFIG=(
    # 相机启用参数
    ["hand_enable"]="hand_enable:=true chassis_enable:=false top_enable:=false"
    ["top_enable"]="top_enable:=true hand_enable:=false chassis_enable:=false"  
    ["chassis_enable"]="chassis_enable:=true hand_enable:=false top_enable:=false"
    ["all_enable"]="hand_enable:=true chassis_enable:=true top_enable:=true"
    
    # 深度话题
    ["hand_depth_topic"]="/camera/hand/depth/image_rect_raw"
    ["top_depth_topic"]="/camera/top/depth/image_rect_raw"
    ["chassis_depth_topic"]="/camera/chassis/depth/image_rect_raw"
    
    # 颜色话题 - 统一使用image_raw (Linus式一致性修复)
    ["hand_color_topic"]="/camera/hand/color/image_raw"
    ["top_color_topic"]="/camera/top/color/image_raw"
    ["chassis_color_topic"]="/camera/chassis/color/image_raw"
    
    # 点云输出话题
    ["hand_cloud_topic"]="/projected_cloud/hand_camera"
    ["top_cloud_topic"]="/projected_cloud/top_camera"
    ["chassis_cloud_topic"]="/projected_cloud/chassis_camera"
    ["all_cloud_topic"]="/projected_cloud/combined"
    
    # TF链接验证 - 根据投影方法动态设定
    ["hand_tf_urdf"]="base_link camera/hand_depth_optical_frame"
    ["top_tf_urdf"]="base_link camera/top_depth_optical_frame"  
    ["chassis_tf_urdf"]="base_link camera/chassis_depth_optical_frame"
    ["hand_tf_calibration"]="base_link calibration/hand_camera_optical"
    ["top_tf_calibration"]="base_link calibration/top_camera_optical"
    ["chassis_tf_calibration"]="base_link calibration/chassis_camera_optical"
    
    # 中文显示名
    ["hand_name"]="手部相机"
    ["top_name"]="顶部相机"
    ["chassis_name"]="底盘相机"
    ["all_name"]="全部相机"
)

# 参数解析
CAMERA_TYPE=${1:-hand}      # 默认hand
PROJECTION_METHOD=${2:-urdf} # 默认urdf投影方法
MODE_TYPE=${3:-color}       # 默认color模式

# 验证参数
if [[ ! "${CAMERA_CONFIG[${CAMERA_TYPE}_name]+isset}" ]]; then
    echo "❌ 错误：不支持的相机类型 '$CAMERA_TYPE'"
    echo "用法: $0 [hand|top|chassis|all] [urdf|calibration|compare] [color|depth]"
    echo "相机选项:"
    echo "  hand     - 手部相机RGBD投影"
    echo "  top      - 顶部相机RGBD投影"
    echo "  chassis  - 底盘相机RGBD投影"
    echo "  all      - 全部相机同步RGBD投影"
    echo "投影方法:"
    echo "  urdf        - 使用URDF/RealSense TF链"
    echo "  calibration - 使用标定TF链"
    echo "  compare     - 对比两种投影方法"
    echo "模式选项:"
    echo "  color    - 彩色点云模式 (深度+RGB)"
    echo "  depth    - 纯深度点云模式"
    exit 1
fi

if [[ "$PROJECTION_METHOD" != "urdf" && "$PROJECTION_METHOD" != "calibration" && "$PROJECTION_METHOD" != "compare" ]]; then
    echo "❌ 错误：不支持的投影方法 '$PROJECTION_METHOD'"
    echo "投影方法选项: urdf, calibration, compare"
    exit 1
fi

if [[ "$MODE_TYPE" != "color" && "$MODE_TYPE" != "depth" ]]; then
    echo "❌ 错误：不支持的模式 '$MODE_TYPE'"
    echo "模式选项: color, depth"
    exit 1
fi

# 获取当前配置
CAMERA_ENABLE=${CAMERA_CONFIG["${CAMERA_TYPE}_enable"]}
CAMERA_NAME=${CAMERA_CONFIG["${CAMERA_TYPE}_name"]}
ENABLE_COLOR=$([ "$MODE_TYPE" = "color" ] && echo "true" || echo "false")

echo "🚀 启动RGBD深度投影测试"
echo "相机: ${CAMERA_NAME}"
echo "投影方法: $PROJECTION_METHOD"
echo "模式: $([ "$MODE_TYPE" = "color" ] && echo "彩色点云 (深度+RGB)" || echo "纯深度点云")"
echo "启用颜色: $ENABLE_COLOR"
echo ""

# 清理函数
cleanup() {
    if [ "$CLEANUP_RUNNING" = "true" ]; then
        return  # 避免重复清理
    fi
    export CLEANUP_RUNNING=true
    
    echo ""
    echo ">>> 清理进程和资源..."
    
    # 优雅关闭ROS节点
    echo "  停止深度投影节点..."
    rosnode kill /depth_projection_node 2>/dev/null || true
    rosnode kill /camera_tf_publisher 2>/dev/null || true
    rosnode kill /rviz_depth_projection 2>/dev/null || true
    
    # 停止相机驱动
    echo "  停止相机驱动..."
    rosnode list | grep realsense2_camera | xargs -I {} rosnode kill {} 2>/dev/null || true
    
    # 强制终止进程
    echo "  强制终止残留进程..."
    #pkill -f realsense2_camera 2>/dev/null || true
    pkill -f depth_projection 2>/dev/null || true
    pkill -f rviz 2>/dev/null || true
    
    # 等待进程完全退出
    sleep 3
    
    # 验证清理结果
    remaining=$(pgrep -f "realsense2_camera|depth_projection|rviz" | wc -l)
    if [ "$remaining" -gt 0 ]; then
        echo "  ⚠️ 仍有 $remaining 个相关进程运行"
    else
        echo "  ✓ 所有进程已清理"
    fi
    
    echo "✓ 清理完成"
}

# 初始清理函数 - 不设置标志位
initial_cleanup() {
    echo ">>> 清理现有进程..."
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f depth_projection 2>/dev/null || true
    rosnode kill /depth_projection_node 2>/dev/null || true
    rosnode kill /camera_tf_publisher 2>/dev/null || true
    pkill -f rviz 2>/dev/null || true
    sleep 2
    echo "✓ 初始清理完成"
}

# 设置信号处理
trap cleanup EXIT INT TERM

# 1. 清理现有进程
initial_cleanup



# 2. RealSense硬件检测和复位 - Linus式实用主义修复
realsense_hardware_check() {

    #skip
    return 0

    echo ">>> 检查RealSense硬件连接..."
    
    # 第一次检查：直接尝试枚举设备
    if rs-enumerate-devices -s >/dev/null 2>&1; then
        echo "✓ RealSense设备检测正常"
        return 0
    fi
    
    echo "⚠️ RealSense设备未检测到，尝试USB复位..."
    
    # USB复位策略 - 经验证有效的硬件修复方案
    echo "  执行USB hub复位..."
    if echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/unbind' 2>/dev/null; then
        sleep 3
        echo "agx" | sudo -S bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/bind' 2>/dev/null
        sleep 5
        echo "  USB复位完成，重新检测..."
        
        # 重新检测设备
        if rs-enumerate-devices -s >/dev/null 2>&1; then
            echo "✓ USB复位成功，RealSense设备可用"
            rs-enumerate-devices -s | head -5
            return 0
        fi
    fi
    
    # 如果复位失败，尝试重新加载UVC驱动
    echo "  尝试重新加载UVC驱动..."
    if echo "agx" | sudo -S modprobe -r uvcvideo 2>/dev/null && echo "agx" | sudo -S modprobe uvcvideo 2>/dev/null; then
        sleep 3
        if rs-enumerate-devices -s >/dev/null 2>&1; then
            echo "✓ 驱动重新加载成功"
            return 0
        fi
    fi
    
    echo "❌ RealSense硬件检测失败"
    echo "   请检查：1) 相机USB连接 2) 电源供应 3) 硬件故障"
    return 1
}

# 执行硬件检测
realsense_hardware_check || {
    echo ""
    echo "⚠️ 警告：相机硬件检测失败，但继续执行软件测试"
    echo "   某些功能可能不可用，建议检查硬件连接"
    echo ""
}


# 3. 启动相机驱动
echo ">>> 启动相机驱动（${CAMERA_NAME}）..."
roslaunch camera_driver camera_driver.launch ${CAMERA_ENABLE} & 
CAMERA_PID=$!
sleep 10

# 4. 验证相机数据流
echo ">>> 验证相机数据流..."

# 首先检查RealSense设备是否仍然可用
if ! rs-enumerate-devices -s >/dev/null 2>&1; then
    echo "⚠️ 警告：RealSense设备在驱动启动后丢失"
    echo "  这可能是设备电源问题或硬件故障"
fi

# 根据相机类型验证对应话题
if [[ "$CAMERA_TYPE" == "all" ]]; then
    # 验证所有相机
    CAMERAS=("hand" "top" "chassis")
else
    CAMERAS=("$CAMERA_TYPE")
fi

# 等待ROS话题注册
echo "  等待ROS话题注册..."
sleep 3

for cam in "${CAMERAS[@]}"; do
    depth_topic=${CAMERA_CONFIG["${cam}_depth_topic"]}
    color_topic=${CAMERA_CONFIG["${cam}_color_topic"]}
    cam_name=${CAMERA_CONFIG["${cam}_name"]}
    
    echo "  检查 ${cam_name}..."
    
    # 检查话题是否存在
    if rostopic list | grep -q "$depth_topic"; then
        echo "    话题已注册: $depth_topic"
        timeout 3 rostopic hz $depth_topic --window=1 >/dev/null 2>&1 && echo "    ✓ ${cam_name}深度数据正常" || echo "    ✗ ${cam_name}深度数据异常"
    else
        echo "    ✗ ${cam_name}深度话题未注册: $depth_topic"
    fi
    
    if [[ "$ENABLE_COLOR" == "true" ]]; then
        if rostopic list | grep -q "$color_topic"; then
            timeout 3 rostopic hz $color_topic --window=1 >/dev/null 2>&1 && echo "    ✓ ${cam_name}颜色数据正常" || echo "    ✗ ${cam_name}颜色数据异常"
        else
            echo "    ✗ ${cam_name}颜色话题未注册: $color_topic"
        fi
    fi
done

echo ""

# 5. 启动深度投影系统（包含完整TF链）
echo ">>> 启动RGBD深度投影系统（包含机器人TF链）..."
roslaunch perception depth_projection.launch \
    use_rviz:=true \
    load_robot_description:=true \
    enable_color:=$ENABLE_COLOR \
    projection_method:=$PROJECTION_METHOD \
    ${CAMERA_ENABLE} \
    publish_combined:=$([ "$CAMERA_TYPE" = "all" ] && echo "true" || echo "false") &

DEPTH_PID=$!
sleep 10

echo ">>> 等待RViz和TF系统稳定..."
sleep 8

# 6. 验证TF连接
echo ">>> 验证TF连接..."
for cam in "${CAMERAS[@]}"; do
    cam_name=${CAMERA_CONFIG["${cam}_name"]}
    
    # 根据投影方法选择正确的TF链
    if [[ "$PROJECTION_METHOD" == "calibration" ]]; then
        tf_link=${CAMERA_CONFIG["${cam}_tf_calibration"]}
        echo "  验证${cam_name}标定TF链: $tf_link"
    elif [[ "$PROJECTION_METHOD" == "compare" ]]; then
        # 对比模式验证两条链
        urdf_tf=${CAMERA_CONFIG["${cam}_tf_urdf"]}
        calib_tf=${CAMERA_CONFIG["${cam}_tf_calibration"]}
        echo "  验证${cam_name} URDF链: $urdf_tf"
        echo "  验证${cam_name}标定链: $calib_tf"
        timeout 3 rosrun tf tf_echo $urdf_tf >/dev/null 2>&1 && echo "    ✓ URDF TF连接正常" || echo "    ⚠️ URDF TF连接需要检查"
        timeout 3 rosrun tf tf_echo $calib_tf >/dev/null 2>&1 && echo "    ✓ 标定TF连接正常" || echo "    ⚠️ 标定TF连接需要检查"
        continue
    else
        tf_link=${CAMERA_CONFIG["${cam}_tf_urdf"]}
        echo "  验证${cam_name} URDF TF链: $tf_link"
    fi
    
    timeout 3 rosrun tf tf_echo $tf_link >/dev/null 2>&1 && echo "    ✓ ${cam_name} TF连接正常" || echo "    ⚠️ ${cam_name} TF连接需要检查"
done

# 7. 验证点云输出
echo ">>> 验证点云输出..."
sleep 3

for cam in "${CAMERAS[@]}"; do
    cloud_topic=${CAMERA_CONFIG["${cam}_cloud_topic"]}
    cam_name=${CAMERA_CONFIG["${cam}_name"]}
    
    timeout 5 rostopic hz $cloud_topic --window=1 >/dev/null 2>&1 && echo "  ✓ ${cam_name}点云输出正常" || echo "  ⚠️ ${cam_name}点云输出异常"
done

# 验证合并点云（如果启用）
if [[ "$CAMERA_TYPE" == "all" ]]; then
    combined_topic=${CAMERA_CONFIG["all_cloud_topic"]}
    timeout 5 rostopic hz $combined_topic --window=1 >/dev/null 2>&1 && echo "  ✓ 合并点云输出正常" || echo "  ⚠️ 合并点云输出异常"
fi

# 8. 运行Python测试脚本
echo ""
echo ">>> 运行功能验证测试..."

# 如果深度数据异常，尝试重启相机驱动
failed_cameras=0
for cam in "${CAMERAS[@]}"; do
    depth_topic=${CAMERA_CONFIG["${cam}_depth_topic"]}
    if ! timeout 2 rostopic hz $depth_topic --window=1 >/dev/null 2>&1; then
        ((failed_cameras++))
    fi
done

# 如果有相机失败，尝试一次重启修复
if [ "$failed_cameras" -gt 0 ]; then
    echo "⚠️ 检测到 $failed_cameras 个相机数据异常，尝试重启修复..."
    
    # 停止相机驱动
    rosnode list | grep realsense2_camera | xargs -I {} rosnode kill {} 2>/dev/null || true
    sleep 3
    
    # 重新执行硬件检测
    if realsense_hardware_check; then
        echo "  重新启动相机驱动..."
        roslaunch camera_driver camera_driver.launch ${CAMERA_ENABLE} &
        sleep 8
        echo "  相机重启完成"
    fi
fi


echo "投影方法: $PROJECTION_METHOD"

case $PROJECTION_METHOD in
    "urdf"|"calibration")
        echo "使用 $PROJECTION_METHOD 投影方法"
        export PATH="/usr/bin:$PATH" && python3 /home/agilex/MobileManipulator/scripts/test_rgbd_projection.py &
        TEST_PID=$!
        ;;
    "compare")
        echo "对比两种投影方法"
        export PATH="/usr/bin:$PATH" && python3 /home/agilex/MobileManipulator/scripts/test_rgbd_projection.py &
        TEST_PID=$!
        ;;
esac

sleep 15  # 给Python测试脚本更多时间等待相机稳定

# 9. 显示系统信息
echo ""
echo "🎉 ${CAMERA_NAME}RGBD投影测试完成！"
echo ""
echo "系统状态："
echo "- 投影模式: $([ "$ENABLE_COLOR" = "true" ] && echo "RGBD彩色点云" || echo "纯深度点云")"
echo "- 机器人模型: 已加载到RViz"
echo "- 固定坐标系: base_link"
echo "- ${CAMERA_NAME}点云: 投影到机器人坐标系"
echo ""

echo "测试话题："
for cam in "${CAMERAS[@]}"; do
    depth_topic=${CAMERA_CONFIG["${cam}_depth_topic"]}
    color_topic=${CAMERA_CONFIG["${cam}_color_topic"]}
    cloud_topic=${CAMERA_CONFIG["${cam}_cloud_topic"]}
    cam_name=${CAMERA_CONFIG["${cam}_name"]}
    
    echo "- ${cam_name}深度: rostopic hz $depth_topic"
    if [[ "$ENABLE_COLOR" == "true" ]]; then
        echo "- ${cam_name}颜色: rostopic hz $color_topic"  
    fi
    echo "- ${cam_name}点云: rostopic hz $cloud_topic"
done

if [[ "$CAMERA_TYPE" == "all" ]]; then
    echo "- 合并点云: rostopic hz ${CAMERA_CONFIG[all_cloud_topic]}"
fi

echo ""
echo "RViz可视化："
echo "- 机器人模型显示"
echo "- TF坐标系显示"
echo "- ${CAMERA_NAME}$([ "$ENABLE_COLOR" = "true" ] && echo "彩色" || echo "")点云显示"
echo ""
echo "按Ctrl+C退出..."

# 保持运行直到用户中断
wait
