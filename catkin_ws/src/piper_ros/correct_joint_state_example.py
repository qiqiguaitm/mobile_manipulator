#!/usr/bin/env python3
"""
正确的关节状态发布示例
解决时间戳和关节数量问题
"""

import rospy
from sensor_msgs.msg import JointState

def publish_correct_joint_states():
    """发布正确格式的关节状态"""
    
    # 初始化ROS节点
    rospy.init_node('correct_joint_state_publisher', anonymous=True)
    
    # 创建发布者
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(50)  # 50Hz
    
    print("🎯 开始发布正确格式的关节状态...")
    print("包含所有8个关节，带有当前时间戳")
    
    while not rospy.is_shutdown():
        # 创建关节状态消息
        joint_state = JointState()
        
        # 设置消息头 - 使用当前时间戳
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = ""
        
        # 设置关节名称 - 必须包含所有8个关节
        joint_state.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 
            'joint5', 'joint6', 'joint7', 'joint8'
        ]
        
        # 设置关节位置 - 8个值对应8个关节
        joint_state.position = [
            0.0,    # joint1
            0.0,    # joint2  
            0.0,    # joint3
            0.0,    # joint4
            0.0,    # joint5
            0.0,    # joint6
            0.01,   # joint7 (夹爪)
            0.01    # joint8 (夹爪)
        ]
        
        # 可选：设置速度和力矩（如果需要）
        joint_state.velocity = [0.0] * 8
        joint_state.effort = [0.0] * 8
        
        # 发布消息
        joint_pub.publish(joint_state)
        
        # 按频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_correct_joint_states()
    except rospy.ROSInterruptException:
        print("关节状态发布被中断")
    except Exception as e:
        print(f"发布出错: {e}")