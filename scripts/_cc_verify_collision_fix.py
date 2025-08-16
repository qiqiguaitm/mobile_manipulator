#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import sys

def verify_collision_fix():
    """验证碰撞检测修复是否生效"""
    rospy.init_node('verify_collision_fix', anonymous=True)
    
    print("\n" + "="*60)
    print("验证碰撞检测修复")
    print("="*60)
    
    # 等待规划场景服务
    rospy.wait_for_service('/get_planning_scene', timeout=5.0)
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    
    # 获取允许碰撞矩阵
    req = PlanningSceneComponents()
    req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    resp = get_planning_scene(req)
    acm = resp.scene.allowed_collision_matrix
    
    # 检查关键碰撞对
    critical_pairs = [
        ("link6", "lifting_Link"),
        ("link5", "lifting_Link"),
        ("link4", "lifting_Link"),
        ("link6", "lidar_Link"),
        ("link5", "lidar_Link"),
        ("link4", "lidar_Link"),
        ("link6", "box_Link"),
        ("link5", "box_Link"),
        ("link4", "box_Link")
    ]
    
    print("\n关键碰撞对检查:")
    fixed_count = 0
    still_disabled = 0
    
    for link1, link2 in critical_pairs:
        if link1 in acm.entry_names and link2 in acm.entry_names:
            idx1 = acm.entry_names.index(link1)
            idx2 = acm.entry_names.index(link2)
            
            if idx2 < len(acm.entry_values[idx1].enabled):
                allowed = acm.entry_values[idx1].enabled[idx2]
                if allowed:
                    print("  ✗ %s <-> %s: 仍然允许碰撞（需要重启MoveIt）" % (link1, link2))
                    still_disabled += 1
                else:
                    print("  ✓ %s <-> %s: 碰撞检测已启用！" % (link1, link2))
                    fixed_count += 1
    
    print("\n" + "="*60)
    print("结果汇总:")
    print("- 已修复的碰撞对: %d" % fixed_count)
    print("- 仍需修复的碰撞对: %d" % still_disabled)
    
    if still_disabled > 0:
        print("\n⚠ 警告: SRDF已更新，但MoveIt需要重启才能生效!")
        print("\n请执行以下步骤:")
        print("1. 停止当前的MoveIt演示: Ctrl+C")
        print("2. 重新启动: roslaunch arm_planner arm_planner_demo.launch")
        print("3. 再次运行此脚本验证")
    else:
        print("\n✓ 所有关键碰撞对已正确配置！")
        print("现在机械臂应该能够正确避开lifting_Link、lidar_Link和box_Link了。")
    
    print("="*60 + "\n")

if __name__ == '__main__':
    try:
        verify_collision_fix()
    except rospy.ROSInterruptException:
        pass