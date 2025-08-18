#!/usr/bin/env python3
"""
机械臂控制问题最终解决方案
根据诊断结果，问题是：
1. 软件使能标志未设置
2. CAN通信存在但数据处理有问题
"""
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time

def main():
    rospy.init_node('fix_arm_control')
    
    print("=== 机械臂控制修复程序 ===\n")
    
    # 1. 持续发布使能标志
    print("1. 持续发布使能标志...")
    enable_pub = rospy.Publisher('/enable_flag', Bool, queue_size=1)
    
    # 2. 创建关节控制发布器
    joint_pub = rospy.Publisher('/joint_ctrl', JointState, queue_size=1)
    
    # 等待发布器就绪
    rospy.sleep(1)
    
    # 3. 持续使能和控制
    rate = rospy.Rate(50)  # 50Hz
    cmd = JointState()
    cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    print("2. 开始控制循环...")
    print("   - 持续发送enable_flag=True")
    print("   - 发送关节控制指令")
    print("   按Ctrl+C停止\n")
    
    start_time = time.time()
    while not rospy.is_shutdown():
        # 发布使能
        enable_pub.publish(Bool(data=True))
        
        # 获取当前时间用于生成正弦运动
        t = time.time() - start_time
        
        # 生成简单的正弦运动（仅第一个关节）
        cmd.header.stamp = rospy.Time.now()
        cmd.position = [
            0.3 * np.sin(0.5 * t),  # joint1: ±0.3弧度正弦运动
            0.0,  # joint2: 保持不动
            0.0,  # joint3: 保持不动
            0.0,  # joint4: 保持不动
            0.0,  # joint5: 保持不动
            0.0   # joint6: 保持不动
        ]
        
        # 设置速度（可选）
        cmd.velocity = [50] * 6  # 速度50
        
        # 发布控制指令
        joint_pub.publish(cmd)
        
        # 每5秒打印一次状态
        if int(t) % 5 == 0 and int(t*10) % 10 == 0:
            try:
                current = rospy.wait_for_message('/joint_states', JointState, timeout=0.5)
                print(f"[{int(t)}s] Joint1当前位置: {current.position[0]:.3f}, 目标: {cmd.position[0]:.3f}")
            except:
                print(f"[{int(t)}s] 无法获取关节状态")
        
        rate.sleep()

if __name__ == "__main__":
    import numpy as np
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序已停止")