#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
夹爪控制示例

这个脚本展示了如何在当前机械臂位置下控制夹爪，
而不会让机械臂回到零位。

推荐的三种方法：
1. 使用独立的gripper组（最简单）
2. 使用gripper控制服务（适合集成到其他系统）
3. 直接设置关节值（最灵活）
"""

import rospy
import moveit_commander
import sys
from std_srvs.srv import SetBool
from std_msgs.msg import Float32

def method1_gripper_group():
    """方法1：使用独立的gripper组"""
    print("\n=== 方法1：使用gripper组 ===")
    
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    
    # 打开夹爪
    print("打开夹爪...")
    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)
    gripper_group.stop()
    
    rospy.sleep(1)
    
    # 关闭夹爪
    print("关闭夹爪...")
    gripper_group.set_named_target("close")
    gripper_group.go(wait=True)
    gripper_group.stop()
    
    print("✅ 完成")

def method2_gripper_service():
    """方法2：使用gripper服务（需要先运行gripper_control_service.py）"""
    print("\n=== 方法2：使用gripper服务 ===")
    
    try:
        # 等待服务
        rospy.wait_for_service('gripper/open', timeout=2.0)
        rospy.wait_for_service('gripper/close', timeout=2.0)
        
        # 创建服务代理
        open_gripper = rospy.ServiceProxy('gripper/open', SetBool)
        close_gripper = rospy.ServiceProxy('gripper/close', SetBool)
        
        # 打开夹爪
        print("通过服务打开夹爪...")
        resp = open_gripper(True)
        print(f"结果: {resp.message}")
        
        rospy.sleep(1)
        
        # 关闭夹爪
        print("通过服务关闭夹爪...")
        resp = close_gripper(True)
        print(f"结果: {resp.message}")
        
        # 设置到特定位置
        print("设置夹爪到半开位置...")
        position_pub = rospy.Publisher('gripper/set_position', Float32, queue_size=1)
        rospy.sleep(0.5)  # 等待发布器建立
        position_pub.publish(Float32(0.0175))  # 半开
        
        print("✅ 完成")
        
    except rospy.ROSException:
        print("❌ 服务未运行，请先运行: rosrun arm_controller gripper_control_service.py")

def method3_direct_control():
    """方法3：直接控制关节值"""
    print("\n=== 方法3：直接设置关节值 ===")
    
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    # 获取当前关节值
    current_joints = arm_group.get_current_joint_values()
    
    # 打开夹爪（只修改关节7和8）
    print("直接设置关节值打开夹爪...")
    target_joints = list(current_joints)
    target_joints[6] = 0.035  # joint7
    target_joints[7] = 0.035  # joint8
    
    arm_group.set_joint_value_target(target_joints)
    arm_group.go(wait=True)
    arm_group.stop()
    
    rospy.sleep(1)
    
    # 设置到任意位置（比如1/3开度）
    print("设置夹爪到1/3开度...")
    target_joints[6] = 0.012  # joint7
    target_joints[7] = 0.012  # joint8
    
    arm_group.set_joint_value_target(target_joints)
    arm_group.go(wait=True)
    arm_group.stop()
    
    rospy.sleep(1)
    
    # 关闭夹爪
    print("关闭夹爪...")
    target_joints[6] = 0.0  # joint7
    target_joints[7] = 0.0  # joint8
    
    arm_group.set_joint_value_target(target_joints)
    arm_group.go(wait=True)
    arm_group.stop()
    
    print("✅ 完成")

def main():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gripper_control_example', anonymous=True)
    
    # 先移动机械臂到一个非零位置
    print("\n准备：移动机械臂到ready位置...")
    arm_group = moveit_commander.MoveGroupCommander("arm")
    arm_group.set_named_target("ready")
    arm_group.go(wait=True)
    arm_group.stop()
    
    print("机械臂已就位，现在演示三种夹爪控制方法")
    rospy.sleep(2)
    
    # 演示三种方法
    try:
        method1_gripper_group()
        rospy.sleep(2)
        
        method2_gripper_service()
        rospy.sleep(2)
        
        method3_direct_control()
        
        print("\n\n✅ 所有演示完成！")
        print("\n推荐：")
        print("- 简单使用：方法1（gripper组）")
        print("- 系统集成：方法2（服务）")
        print("- 精确控制：方法3（直接设置）")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()