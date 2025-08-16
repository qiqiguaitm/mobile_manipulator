#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def verify_arm_group():
    """验证arm组修复"""
    rospy.init_node('verify_arm_fix', anonymous=True)
    
    try:
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        
        print("\n=== 验证ARM组修复 ===\n")
        
        # 获取所有组
        groups = robot.get_group_names()
        print(f"可用的MoveIt组: {groups}")
        
        # 检查arm组
        if 'arm' in groups:
            print("\n✅ arm组存在")
            
            try:
                arm_group = moveit_commander.MoveGroupCommander("arm")
                joints = arm_group.get_active_joints()
                print(f"arm组关节: {joints}")
                print(f"关节数量: {len(joints)}")
                
                if len(joints) == 6:
                    print("✅ arm组现在是6轴运动链（正确）")
                else:
                    print(f"❌ arm组有{len(joints)}个关节（应该是6个）")
                
                # 测试运动学求解器
                print("\n测试运动学求解器...")
                eef_link = arm_group.get_end_effector_link()
                print(f"末端执行器链接: {eef_link}")
                
                # 尝试获取当前位姿
                try:
                    current_pose = arm_group.get_current_pose()
                    print("✅ 成功获取当前位姿")
                except Exception as e:
                    print(f"❌ 无法获取当前位姿: {e}")
                    
            except Exception as e:
                print(f"❌ 无法初始化arm组: {e}")
        else:
            print("❌ arm组不存在")
            
        # 检查arm_with_gripper组
        if 'arm_with_gripper' in groups:
            print("\n✅ arm_with_gripper组存在")
            try:
                awg_group = moveit_commander.MoveGroupCommander("arm_with_gripper")
                joints = awg_group.get_active_joints()
                print(f"arm_with_gripper组关节: {joints}")
                print(f"关节数量: {len(joints)}")
            except Exception as e:
                print(f"注意: arm_with_gripper组可能没有运动学求解器（这是正常的）")
                
        # 检查gripper组
        if 'gripper' in groups:
            print("\n✅ gripper组存在")
            gripper_group = moveit_commander.MoveGroupCommander("gripper")
            joints = gripper_group.get_active_joints()
            print(f"gripper组关节: {joints}")
            
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        verify_arm_group()
    except rospy.ROSInterruptException:
        pass