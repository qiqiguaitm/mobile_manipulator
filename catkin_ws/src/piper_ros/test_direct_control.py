#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def test_direct_control():
    """直接测试机械臂控制"""
    rospy.init_node('test_direct_control')
    
    print("========== 直接控制测试 ==========")
    
    # 创建发布者
    pub = rospy.Publisher('/joint_ctrl_commands', JointState, queue_size=10)
    
    # 等待发布者连接
    print("等待发布者连接...")
    time.sleep(2)
    
    # 创建测试命令
    joint_cmd = JointState()
    joint_cmd.header = Header()
    joint_cmd.header.stamp = rospy.Time.now()
    joint_cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    
    # 测试1: 小幅度运动
    print("测试1: 发送小幅度关节运动命令...")
    joint_cmd.position = [0.1, 0.1, -0.1, 0.1, -0.1, 0.1, 0.01]
    joint_cmd.velocity = [0.0] * 7
    joint_cmd.effort = [0.0] * 7
    
    for i in range(5):
        joint_cmd.header.stamp = rospy.Time.now()
        pub.publish(joint_cmd)
        print(f"发送命令 {i+1}/5: {[round(p, 3) for p in joint_cmd.position]}")
        time.sleep(1)
    
    print("等待5秒观察机械臂运动...")
    time.sleep(5)
    
    # 测试2: 回到零位
    print("测试2: 发送归零命令...")
    joint_cmd.position = [0.0] * 7
    
    for i in range(5):
        joint_cmd.header.stamp = rospy.Time.now()
        pub.publish(joint_cmd)
        print(f"发送归零命令 {i+1}/5")
        time.sleep(1)
    
    print("测试完成！请观察机械臂是否有运动")

if __name__ == '__main__':
    try:
        test_direct_control()
    except rospy.ROSInterruptException:
        print("测试被中断")