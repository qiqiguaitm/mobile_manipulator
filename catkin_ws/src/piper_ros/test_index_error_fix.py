#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
import time

def test_various_joint_commands():
    """测试各种长度的关节命令，验证IndexError修复"""
    rospy.init_node('test_index_error_fix')
    
    print("========== 测试IndexError修复 ==========")
    
    # 创建发布者
    pub = rospy.Publisher('/joint_ctrl_commands', JointState, queue_size=10)
    
    # 等待发布者连接
    print("等待发布者连接...")
    time.sleep(2)
    
    test_cases = [
        {
            "name": "单关节命令（夹爪）",
            "positions": [0.02],
            "names": ["joint7"]
        },
        {
            "name": "部分关节命令（前3个关节）",
            "positions": [0.1, 0.1, -0.1],
            "names": ["joint1", "joint2", "joint3"]
        },
        {
            "name": "完整关节命令（7个关节）",
            "positions": [0.1, 0.1, -0.1, 0.1, -0.1, 0.1, 0.01],
            "names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        },
        {
            "name": "乱序关节命令",
            "positions": [0.05, 0.02, 0.1],
            "names": ["joint7", "joint1", "joint3"]
        }
    ]
    
    for i, test_case in enumerate(test_cases):
        print(f"\n测试 {i+1}: {test_case['name']}")
        
        # 创建关节状态消息
        joint_cmd = JointState()
        joint_cmd.header = Header()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = test_case['names']
        joint_cmd.position = test_case['positions']
        joint_cmd.velocity = [0.0] * len(test_case['positions'])
        joint_cmd.effort = [0.0] * len(test_case['positions'])
        
        print(f"  发送关节: {test_case['names']}")
        print(f"  发送位置: {[round(p, 4) for p in test_case['positions']]}")
        
        # 发送命令
        for j in range(3):
            joint_cmd.header.stamp = rospy.Time.now()
            pub.publish(joint_cmd)
            time.sleep(0.5)
        
        print(f"  ✅ 测试 {i+1} 完成，无IndexError")
        time.sleep(2)
    
    print("\n========== 测试MoveIt服务 ==========")
    
    # 测试MoveIt服务调用
    service_name = '/joint_moveit_ctrl_endpose'
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        print(f"✅ {service_name} 服务可用")
        
        client = rospy.ServiceProxy(service_name, JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_endpose = [0.1, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
        request.max_velocity = 0.3
        request.max_acceleration = 0.3
        
        print("发送MoveIt末端位姿命令...")
        response = client(request)
        
        if response.status:
            print("✅ MoveIt命令执行成功，无IndexError")
        else:
            print(f"❌ MoveIt命令执行失败: {response.error_code}")
            
    except rospy.ROSException:
        print(f"❌ {service_name} 服务不可用")
    
    print("\n========== 测试完成 ==========")
    print("如果没有看到IndexError，说明修复成功！")

if __name__ == '__main__':
    try:
        test_various_joint_commands()
    except rospy.ROSInterruptException:
        print("测试被中断")