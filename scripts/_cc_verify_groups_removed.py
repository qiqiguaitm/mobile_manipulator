#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def verify_groups():
    """验证chassis和sensors组是否已被移除"""
    rospy.init_node('verify_groups_removed', anonymous=True)
    
    print("\n" + "="*60)
    print("验证move_group配置")
    print("="*60)
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    
    # 获取所有组名
    group_names = robot.get_group_names()
    
    print("\n当前可用的规划组:")
    for group in group_names:
        print("  - %s" % group)
    
    # 检查chassis和sensors
    removed_groups = []
    still_exists = []
    
    if "chassis" in group_names:
        still_exists.append("chassis")
    else:
        removed_groups.append("chassis")
        
    if "sensors" in group_names:
        still_exists.append("sensors")
    else:
        removed_groups.append("sensors")
    
    print("\n" + "="*60)
    print("结果:")
    
    if removed_groups:
        print("\n✓ 已成功移除的组:")
        for group in removed_groups:
            print("  - %s" % group)
    
    if still_exists:
        print("\n✗ 仍然存在的组（需要重启MoveIt）:")
        for group in still_exists:
            print("  - %s" % group)
        print("\n请重启MoveIt以使更改生效:")
        print("  1. Ctrl+C 停止当前进程")
        print("  2. roslaunch arm_planner arm_planner_demo.launch")
    else:
        print("\n✓ chassis和sensors组已成功移除!")
        print("  MoveIt现在只加载机械臂相关的规划组")
        print("  不会再出现相关的警告信息")
    
    print("="*60 + "\n")
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        verify_groups()
    except rospy.ROSInterruptException:
        pass