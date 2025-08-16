#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Check collision detection between arm and chassis
"""

import rospy
import moveit_commander
import sys
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

def check_chassis_collision():
    rospy.init_node('check_chassis_collision')
    
    # Initialize
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("arm")
    
    # Get planning scene service
    rospy.wait_for_service('/get_planning_scene', timeout=5)
    get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    
    rospy.sleep(2)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Chassis Collision Detection Check")
    rospy.loginfo("=" * 60)
    
    # Get ACM
    try:
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        resp = get_scene(req)
        acm = resp.scene.allowed_collision_matrix
        
        # Check critical pairs
        critical_pairs = [
            ("link1", "box_Link", "机械臂第1节 ↔ 底盘箱体"),
            ("link2", "box_Link", "机械臂第2节 ↔ 底盘箱体"),
            ("link3", "box_Link", "机械臂第3节 ↔ 底盘箱体"),
            ("link1", "lifting_Link", "机械臂第1节 ↔ 立柱"),
            ("link2", "lifting_Link", "机械臂第2节 ↔ 立柱"),
            ("link3", "lifting_Link", "机械臂第3节 ↔ 立柱"),
            ("link1", "base_link", "机械臂第1节 ↔ 底座"),
            ("link2", "base_link", "机械臂第2节 ↔ 底座"),
            ("link3", "base_link", "机械臂第3节 ↔ 底座"),
        ]
        
        rospy.loginfo("\n🔍 关键碰撞对检测状态：")
        active_count = 0
        disabled_count = 0
        
        for link1, link2, desc in critical_pairs:
            # Check if links exist in ACM
            if link1 not in acm.entry_names or link2 not in acm.entry_names:
                rospy.loginfo("  ✅ %s: 碰撞检测【激活】（默认）", desc)
                active_count += 1
                continue
            
            # Check if collision is disabled
            try:
                idx1 = acm.entry_names.index(link1)
                idx2 = acm.entry_names.index(link2)
                
                if idx1 < len(acm.entry_values) and idx2 < len(acm.entry_values[idx1].enabled):
                    if not acm.entry_values[idx1].enabled[idx2]:
                        rospy.logerr("  ❌ %s: 碰撞检测【禁用】（危险！）", desc)
                        disabled_count += 1
                    else:
                        rospy.loginfo("  ✅ %s: 碰撞检测【激活】", desc)
                        active_count += 1
            except:
                rospy.loginfo("  ✅ %s: 碰撞检测【激活】（默认）", desc)
                active_count += 1
        
        rospy.loginfo("\n📊 统计：")
        rospy.loginfo("  激活的碰撞检测: %d", active_count)
        rospy.loginfo("  禁用的碰撞检测: %d", disabled_count)
        
        if disabled_count > 0:
            rospy.logerr("\n⚠️  警告：部分底盘碰撞检测被禁用！")
        else:
            rospy.loginfo("\n✅ 所有底盘碰撞检测都已激活")
            
    except Exception as e:
        rospy.logerr("Failed to check ACM: %s", str(e))
    
    # Test dangerous poses
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("测试危险姿态")
    rospy.loginfo("=" * 60)
    
    test_poses = [
        {
            "name": "机械臂向后弯（撞立柱）",
            "joints": [0, 2.0, -2.0, 0, 0, 0, 0, 0]
        },
        {
            "name": "机械臂向下（撞底盘）", 
            "joints": [0, 1.8, 0, 0, 1.57, 0, 0, 0]
        }
    ]
    
    for test in test_poses:
        rospy.loginfo("\n测试: %s", test["name"])
        move_group.set_joint_value_target(test["joints"])
        
        plan = move_group.plan()
        if plan[0]:
            rospy.logerr("  ❌ 规划成功 - 可能撞到底盘！")
        else:
            rospy.loginfo("  ✅ 规划失败 - 碰撞检测工作正常")
        
        move_group.clear_pose_targets()
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        check_chassis_collision()
    except rospy.ROSInterruptException:
        pass