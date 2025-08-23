#!/bin/bash
# 多相机深度可视化测试脚本
# 用法: ./visualize_depth_test.sh [hand|top|chassis]
# 默认: hand

# 设置libffi环境
export LD_PRELOAD='/usr/lib/aarch64-linux-gnu/libffi.so.7'

# 相机配置数据结构 - Linus式设计：消除所有特殊情况
declare -A CAMERA_CONFIG=(
    # 相机启用参数
    ["hand_enable"]="enable_hand_camera:=true enable_chassis_camera:=false enable_top_camera:=false"
    ["top_enable"]="enable_top_camera:=true enable_hand_camera:=false enable_chassis_camera:=false"  
    ["chassis_enable"]="enable_chassis_camera:=true enable_hand_camera:=false enable_top_camera:=false"
    
    # 深度话题
    ["hand_depth_topic"]="/camera/hand/depth/image_rect_raw"
    ["top_depth_topic"]="/camera/top/depth/image_rect_raw"
    ["chassis_depth_topic"]="/camera/chassis/depth/image_rect_raw"
    
    # 点云输出话题
    ["hand_cloud_topic"]="/projected_cloud/hand_camera"
    ["top_cloud_topic"]="/projected_cloud/top_camera"
    ["chassis_cloud_topic"]="/projected_cloud/chassis_camera"
    
    # TF链接
    ["hand_tf"]="base_link hand_camera_link"
    ["top_tf"]="base_link top_camera_link"  
    ["chassis_tf"]="base_link chassis_camera_link"
    
    # 中文显示名
    ["hand_name"]="手部相机"
    ["top_name"]="顶部相机"
    ["chassis_name"]="底盘相机"
)

# 参数解析：默认hand，支持向后兼容
CAMERA_TYPE=${1:-hand}

# 验证参数
if [[ ! "${CAMERA_CONFIG[${CAMERA_TYPE}_name]+isset}" ]]; then
    echo "❌ 错误：不支持的相机类型 '$CAMERA_TYPE'"
    echo "用法: $0 [hand|top|chassis]"
    echo "默认: hand"
    exit 1
fi

# 获取当前相机配置
CAMERA_ENABLE=${CAMERA_CONFIG["${CAMERA_TYPE}_enable"]}
DEPTH_TOPIC=${CAMERA_CONFIG["${CAMERA_TYPE}_depth_topic"]}
CLOUD_TOPIC=${CAMERA_CONFIG["${CAMERA_TYPE}_cloud_topic"]}
TF_LINK=${CAMERA_CONFIG["${CAMERA_TYPE}_tf"]}
CAMERA_NAME=${CAMERA_CONFIG["${CAMERA_TYPE}_name"]}

# 清理现有进程
echo ">>> 清理现有进程..."
pkill -f realsense2_camera 2>/dev/null || true
rosnode kill /depth_projection_node 2>/dev/null || true
rosnode kill /camera_tf_publisher 2>/dev/null || true
sleep 3

# 1. 启动相机驱动
echo ">>> 启动相机驱动（${CAMERA_NAME}）..."
roslaunch camera_driver camera_driver.launch ${CAMERA_ENABLE} &

CAMERA_PID=$!
sleep 10

# 2. 启动深度投影系统
echo ">>> 启动深度投影系统..."
roslaunch perception depth_projection.launch \
    use_rviz:=true \
    load_robot_description:=true \
    ${CAMERA_ENABLE} \
    publish_combined:=false &

DEPTH_PID=$!
sleep 8

echo ">>> 等待RViz和TF系统稳定..."
sleep 5

# 3. 验证数据流
echo ">>> 验证深度数据流..."
timeout 3 rostopic hz ${DEPTH_TOPIC} --window=1 && echo "✓ 深度数据正常" || echo "✗ 深度数据异常"

echo ">>> 验证TF连接..."
timeout 3 rosrun tf tf_echo ${TF_LINK} && echo "✓ TF连接正常" || echo "⚠️ TF连接需要检查"

# 4. 验证点云输出
echo ">>> 验证点云输出..."
sleep 5
timeout 5 rostopic hz ${CLOUD_TOPIC} --window=1 && echo "✓ 点云输出正常" || echo "⚠️ 点云输出异常"

echo ">>> 显示点云信息..."
timeout 3 rostopic echo ${CLOUD_TOPIC} --noarr | head -15

echo ""
echo "🎉 ${CAMERA_NAME}深度投影测试完成！"
echo ""
echo "系统状态："
echo "- 机器人模型: 已加载到RViz"
echo "- 固定坐标系: base_link"
echo "- ${CAMERA_NAME}点云: 投影到机器人坐标系"
echo ""
echo "测试话题："
echo "- 深度图像: rostopic hz ${DEPTH_TOPIC}"
echo "- 点云输出: rostopic hz ${CLOUD_TOPIC}"
echo "- 点云信息: rostopic echo ${CLOUD_TOPIC} --noarr"
echo ""
echo "RViz可视化："
echo "- 机器人模型显示"
echo "- TF坐标系显示"
echo "- ${CAMERA_NAME}点云显示"
echo ""
echo "按Ctrl+C退出..."

# 保持运行
#wait $DEPTH_PID
