#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def simple_collision_test():
    """简单测试碰撞检测功能"""
    rospy.init_node('simple_collision_test', anonymous=True)
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # 使用piper组（6轴）
    move_group = moveit_commander.MoveGroupCommander("piper")
    
    print("\n=== 简单碰撞检测测试 ===")
    print("使用规划组: piper (6轴)")
    
    # 获取当前关节值
    current_joints = move_group.get_current_joint_values()
    print("\n当前关节角度:")
    for i, angle in enumerate(current_joints):
        print("  joint%d: %.3f rad (%.1f度)" % (i+1, angle, angle * 180 / 3.14159))
    
    # 测试1: 尝试让机械臂向前伸展（可能碰到lifting_Link）
    print("\n测试1: 向前伸展测试")
    test_joints = list(current_joints)
    test_joints[1] = -1.0  # joint2向前倾斜
    test_joints[2] = 0.5   # joint3伸展
    
    move_group.set_joint_value_target(test_joints)
    success, plan, planning_time, error_code = move_group.plan()
    
    if success:
        print("✓ 规划成功 (%.2f秒)" % planning_time)
    else:
        print("✗ 规划失败 - 可能检测到碰撞")
        print("  错误代码: %s" % error_code)
    
    # 测试2: 尝试让机械臂向下（可能碰到base_link或box_Link）
    print("\n测试2: 向下运动测试")
    test_joints = list(current_joints)
    test_joints[1] = 1.5   # joint2向下
    test_joints[2] = 1.5   # joint3向下
    
    move_group.set_joint_value_target(test_joints)
    success, plan, planning_time, error_code = move_group.plan()
    
    if success:
        print("✓ 规划成功 (%.2f秒)" % planning_time)
    else:
        print("✗ 规划失败 - 可能检测到碰撞")
        print("  错误代码: %s" % error_code)
    
    # 显示当前场景中的碰撞对象
    print("\n当前场景中的对象:")
    known_objects = scene.get_known_object_names()
    if known_objects:
        for obj in known_objects:
            print("  - %s" % obj)
    else:
        print("  没有额外的碰撞对象")
    
    # 获取碰撞检测信息
    print("\n机器人链接信息:")
    links = robot.get_link_names()
    collision_prone_links = ["lifting_Link", "lidar_Link", "box_Link", "base_link"]
    for link in collision_prone_links:
        if link in links:
            print("  ✓ %s 存在于机器人模型中" % link)
        else:
            print("  ✗ %s 不在机器人模型中" % link)
    
    print("\n提示：")
    print("1. 在RViz中查看Motion Planning → Scene Robot")
    print("2. 勾选'Show Robot Collision'查看碰撞网格")
    print("3. 拖动交互标记测试碰撞检测")
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        simple_collision_test()
    except rospy.ROSInterruptException:
        pass