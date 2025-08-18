#!/bin/bash
# Piper快速使能修复脚本

echo "======================================"
echo "Piper机械臂快速使能修复"
echo "======================================"

echo -e "\n选择修复方式："
echo "1. 软件修复（推荐）"
echo "2. 重启驱动节点"
echo "3. 完整诊断"
echo -n "请选择 (1-3): "
read choice

case $choice in
    1)
        echo -e "\n执行软件修复..."
        # 方法1：重置并使能
        echo "- 发送停止命令"
        rosservice call /stop_srv "{}" 2>/dev/null
        sleep 1
        
        echo "- 重置紧急停止"
        rosservice call /reset_emergency_stop "{}" 2>/dev/null
        sleep 0.5
        
        echo "- 发送使能消息"
        rostopic pub -1 /enable_flag std_msgs/Bool "data: true"
        sleep 1
        
        echo "- 调用使能服务"
        rosservice call /enable_srv "enable_request: true"
        
        echo -e "\n检查使能状态..."
        timeout 2 rostopic echo -n 1 /piper_status | grep "arm_status" || echo "无法获取状态"
        ;;
        
    2)
        echo -e "\n重启驱动节点..."
        echo "- 停止当前驱动"
        rosnode kill /piper_driver 2>/dev/null
        sleep 2
        
        echo "- 重新启动驱动"
        echo "请在新终端运行："
        echo "roslaunch arm_controller start_single_piper.launch"
        ;;
        
    3)
        echo -e "\n执行完整诊断..."
        
        echo -e "\n1. ROS节点状态："
        rosnode list | grep -E "piper|arm" || echo "未找到相关节点"
        
        echo -e "\n2. 相关服务："
        rosservice list | grep -E "enable|stop|reset" || echo "未找到相关服务"
        
        echo -e "\n3. 关节状态："
        timeout 2 rostopic echo -n 1 /joint_states | grep -E "position:|velocity:" || echo "无关节数据"
        
        echo -e "\n4. CAN接口状态："
        ip link show can0 2>/dev/null || echo "CAN接口未找到"
        
        echo -e "\n5. 尝试自动修复..."
        python3 /home/agilex/MobileManipulator/scripts/_cc_fix_piper_enable.py
        ;;
        
    *)
        echo "无效选择"
        ;;
esac

echo -e "\n======================================"
echo "提示："
echo "- 如果使能后仍无响应，检查急停按钮"
echo "- 确保CAN线连接正常"
echo "- 可以使用控制面板的ENABLE按钮再次尝试"
echo "======================================