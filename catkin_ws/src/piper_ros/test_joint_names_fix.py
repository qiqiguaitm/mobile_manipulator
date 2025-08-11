#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import time

def test_joint_names():
    """测试关节名称修复"""
    rospy.init_node('test_joint_names_fix')
    
    print("========== 测试关节名称修复 ==========")
    
    joint_states_received = False
    joint_states_single_received = False
    
    def joint_states_callback(msg):
        global joint_states_received
        joint_states_received = True
        print(f"✅ /joint_states 话题数据:")
        print(f"   关节数量: {len(msg.name)}")
        print(f"   关节名称: {msg.name}")
        print(f"   关节位置: {[round(p, 4) for p in msg.position]}")
        print(f"   时间戳: {msg.header.stamp}")
        
        # 检查是否包含所有期望的关节
        expected_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        missing_joints = [j for j in expected_joints if j not in msg.name]
        if missing_joints:
            print(f"   ❌ 缺失关节: {missing_joints}")
        else:
            print(f"   ✅ 所有基本关节都存在")
            
        if 'joint8' in msg.name:
            print(f"   ✅ joint8 (夹爪对称关节) 存在")
        
        # 检查是否还有旧的'gripper'关节名
        if 'gripper' in msg.name:
            print(f"   ❌ 仍包含旧的'gripper'关节名")
        else:
            print(f"   ✅ 不再包含旧的'gripper'关节名")
    
    def joint_states_single_callback(msg):
        global joint_states_single_received
        joint_states_single_received = True
        print(f"\n✅ /joint_states_single 话题数据:")
        print(f"   关节数量: {len(msg.name)}")
        print(f"   关节名称: {msg.name}")
        print(f"   关节位置: {[round(p, 4) for p in msg.position]}")
        
        # 检查是否使用了正确的关节名
        if 'gripper' in msg.name:
            print(f"   ❌ /joint_states_single 仍使用'gripper'关节名")
        elif 'joint7' in msg.name:
            print(f"   ✅ /joint_states_single 使用了正确的'joint7'关节名")
    
    # 订阅话题
    sub1 = rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    sub2 = rospy.Subscriber('/joint_states_single', JointState, joint_states_single_callback)
    
    print("等待关节状态数据...")
    
    # 等待10秒收集数据
    start_time = time.time()
    while time.time() - start_time < 10:
        if joint_states_received and joint_states_single_received:
            break
        rospy.sleep(0.1)
    
    if not joint_states_received:
        print("❌ 10秒内未接收到/joint_states数据")
    
    if not joint_states_single_received:
        print("❌ 10秒内未接收到/joint_states_single数据")
    
    # 清理
    sub1.unregister()
    sub2.unregister()
    
    print("\n========== 测试完成 ==========")
    
    if joint_states_received and joint_states_single_received:
        print("✅ 关节名称修复测试完成")
        print("现在可以测试MoveIt服务了")
    else:
        print("❌ 部分测试失败，请检查系统状态")

if __name__ == '__main__':
    try:
        test_joint_names()
    except rospy.ROSInterruptException:
        print("测试被中断")