#!/bin/bash
#
# 检查并解决joint_states发布冲突
#

echo "检查joint_states发布者..."

# 获取所有发布到/joint_states的节点
publishers=$(rostopic info /joint_states 2>/dev/null | grep -A 100 "Publishers:" | grep -E "^\s+\*" | awk '{print $2}')

if [ -z "$publishers" ]; then
    echo "未找到joint_states发布者"
    exit 0
fi

# 计算发布者数量
num_publishers=$(echo "$publishers" | wc -l)

echo "找到 $num_publishers 个发布者:"
echo "$publishers"

if [ $num_publishers -gt 1 ]; then
    echo ""
    echo "⚠️  警告: 检测到多个joint_states发布者!"
    echo "这会导致'not newer than previous state'警告"
    echo ""
    echo "解决方案:"
    echo "1. 在real模式下，确保只运行piper_ctrl_single_node"
    echo "2. 在fake模式下，确保只运行joint_state_publisher"
    echo "3. 检查是否意外启动了多个控制器"
    echo ""
    
    # 检查是否同时运行了real和fake模式
    if echo "$publishers" | grep -q "joint_state_publisher" && echo "$publishers" | grep -q "piper_ctrl_single_node"; then
        echo "❌ 错误: 同时检测到real模式和fake模式的发布者!"
        echo "建议: 停止joint_state_publisher"
        echo ""
        echo "执行: rosnode kill /joint_state_publisher"
    fi
else
    echo "✅ 只有一个发布者，配置正确"
fi

# 显示发布频率
echo ""
echo "检查发布频率..."
rostopic hz /joint_states -w 3 2>/dev/null | head -5