#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

def test_collision_detection():
    """测试碰撞检测功能是否正常工作"""
    rospy.init_node('collision_detection_test', anonymous=True)
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    
    # 获取规划组
    group_name = "piper_with_gripper"
    try:
        move_group = moveit_commander.MoveGroupCommander(group_name)
    except:
        group_name = "piper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
    
    print("\n=== 碰撞检测测试 ===")
    print("使用规划组: %s" % group_name)
    
    # 获取当前状态
    current_pose = move_group.get_current_pose().pose
    print("\n当前位置:")
    print("  x: %.3f, y: %.3f, z: %.3f" % (current_pose.position.x, 
                                           current_pose.position.y, 
                                           current_pose.position.z))
    
    # 获取碰撞检测状态
    planning_scene = scene.get_planning_scene(moveit_commander.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    print("\n当前规划场景中的碰撞对象:")
    known_objects = scene.get_known_object_names()
    if known_objects:
        for obj in known_objects:
            print("  - %s" % obj)
    else:
        print("  没有检测到碰撞对象")
    
    # 测试一个可能与lifting_Link碰撞的位置
    print("\n测试碰撞检测:")
    test_pose = Pose()
    test_pose.position.x = 0.3  # 向前移动，可能碰到lifting_Link
    test_pose.position.y = 0.0
    test_pose.position.z = 0.4
    test_pose.orientation.w = 1.0
    
    print("尝试移动到可能碰撞的位置: x=0.3, y=0.0, z=0.4")
    
    # 设置目标位置
    move_group.set_pose_target(test_pose)
    
    # 尝试规划
    success, plan, planning_time, error_code = move_group.plan()
    
    if success:
        print("✓ 规划成功 - 警告：碰撞检测可能没有正常工作！")
        print("  规划时间: %.3f秒" % planning_time)
    else:
        print("✗ 规划失败 - 碰撞检测正常工作")
        print("  错误代码: %s" % error_code)
    
    # 检查当前的碰撞矩阵
    print("\n允许碰撞矩阵 (ACM) 信息:")
    robot_state = robot.get_current_state()
    
    # 测试特定链接之间的碰撞检测
    test_pairs = [
        ("link6", "lifting_Link"),
        ("link5", "box_Link"),
        ("link4", "lidar_Link"),
        ("gripper_finger1_right", "base_link")
    ]
    
    print("\n测试特定链接对的碰撞检测:")
    for link1, link2 in test_pairs:
        print("  %s <-> %s" % (link1, link2))
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_collision_detection()
    except rospy.ROSInterruptException:
        pass