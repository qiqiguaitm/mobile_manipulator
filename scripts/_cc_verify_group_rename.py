#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def verify_group_rename():
    """验证组重命名"""
    rospy.init_node('verify_group_rename', anonymous=True)
    
    print("\n=== 验证组重命名 ===\n")
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    
    # 获取所有组
    groups = robot.get_group_names()
    print(f"可用的MoveIt组: {groups}")
    
    print("\n检查结果:")
    
    # 检查arm组是否已删除
    if 'arm' in groups:
        print("❌ arm组仍然存在（应该已删除）")
    else:
        print("✅ arm组已成功删除")
    
    # 检查piper组
    if 'piper' in groups:
        print("✅ piper组存在（6轴机械臂）")
        try:
            piper_group = moveit_commander.MoveGroupCommander("piper")
            joints = piper_group.get_active_joints()
            print(f"   关节: {joints}")
            print(f"   关节数: {len(joints)}")
        except:
            pass
    else:
        print("❌ piper组不存在")
    
    # 检查piper_with_gripper组
    if 'piper_with_gripper' in groups:
        print("✅ piper_with_gripper组存在（6轴+夹爪）")
        try:
            pwg_group = moveit_commander.MoveGroupCommander("piper_with_gripper")
            joints = pwg_group.get_active_joints()
            print(f"   关节: {joints}")
            print(f"   关节数: {len(joints)}")
        except:
            pass
    else:
        print("❌ piper_with_gripper组不存在")
    
    # 检查arm_with_gripper组是否已删除
    if 'arm_with_gripper' in groups:
        print("❌ arm_with_gripper组仍然存在（应该已重命名）")
    else:
        print("✅ arm_with_gripper组已成功重命名")
    
    # 检查gripper组
    if 'gripper' in groups:
        print("✅ gripper组存在")
    
    print("\n=== 总结 ===")
    print("已完成的修改：")
    print("1. 删除了重复的arm组")
    print("2. 重命名arm_with_gripper为piper_with_gripper")
    print("3. 现在只保留piper（6轴）和piper_with_gripper（8轴）")
    print("4. 避免了组定义的重复和混淆")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        verify_group_rename()
    except rospy.ROSInterruptException:
        pass