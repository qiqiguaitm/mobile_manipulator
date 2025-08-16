#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

def check_start_state_collision():
    """检查起始状态的碰撞问题"""
    rospy.init_node('check_start_collision', anonymous=True)
    
    print("\n=== 检查起始状态碰撞问题 ===\n")
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # 等待服务
    rospy.wait_for_service('/check_state_validity', timeout=5.0)
    check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    
    # 获取当前关节状态
    joint_states = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    print("当前关节状态:")
    for i, name in enumerate(joint_states.name):
        if 'joint' in name:
            print(f"  {name}: {joint_states.position[i]:.3f}")
    
    # 检查不同组的碰撞
    groups = ['arm', 'piper', 'arm_with_gripper']
    
    for group_name in groups:
        if group_name in robot.get_group_names():
            print(f"\n检查组 '{group_name}':")
            group = moveit_commander.MoveGroupCommander(group_name)
            
            # 获取当前状态
            current_state = robot.get_current_state()
            
            # 创建状态有效性请求
            req = GetStateValidityRequest()
            req.robot_state = current_state
            req.group_name = group_name
            
            try:
                # 检查状态有效性
                resp = check_state_validity(req)
                if resp.valid:
                    print(f"  ✅ 当前状态有效")
                else:
                    print(f"  ❌ 当前状态无效！")
                    if resp.contacts:
                        print(f"  碰撞信息:")
                        for contact in resp.contacts:
                            print(f"    - {contact.contact_body_1} <-> {contact.contact_body_2}")
                    else:
                        print("  无具体碰撞信息")
            except Exception as e:
                print(f"  ⚠️  无法检查状态: {e}")
    
    # 检查特定的碰撞对
    print("\n=== 检查具体碰撞位置 ===")
    
    # 检查base_link和link2的相对位置
    try:
        import tf
        tf_listener = tf.TransformListener()
        rospy.sleep(1.0)
        
        (trans, rot) = tf_listener.lookupTransform('base_link', 'link2', rospy.Time(0))
        distance = (trans[0]**2 + trans[1]**2 + trans[2]**2)**0.5
        print(f"\nbase_link到link2的距离: {distance:.3f}m")
        print(f"相对位置: x={trans[0]:.3f}, y={trans[1]:.3f}, z={trans[2]:.3f}")
        
        if distance < 0.5:
            print("⚠️  警告：link2离base_link太近，可能发生碰撞！")
    except Exception as e:
        print(f"无法获取TF变换: {e}")
    
    # 建议的解决方案
    print("\n=== 建议的解决方案 ===")
    print("1. 减小base_link的碰撞网格尺寸")
    print("2. 调整机械臂的初始位置（增加z轴偏移）")
    print("3. 在SRDF中添加base_link和link2的碰撞禁用（如果它们永远不会碰撞）")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        check_start_state_collision()
    except rospy.ROSInterruptException:
        pass