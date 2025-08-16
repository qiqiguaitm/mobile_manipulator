#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import sys

def check_collision_matrix():
    """检查当前的碰撞矩阵状态"""
    rospy.init_node('check_collision_matrix', anonymous=True)
    
    print("\n" + "="*60)
    print("检查碰撞矩阵状态")
    print("="*60)
    
    try:
        # 初始化MoveIt
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        
        # 等待场景初始化
        rospy.sleep(2.0)
        
        # 获取碰撞矩阵
        acm = scene.get_planning_scene().allowed_collision_matrix
        
        # 检查关键连杆对
        key_pairs = [
            ("lifting_Link", "link1"),
            ("lifting_Link", "link2"), 
            ("lifting_Link", "link3"),
            ("lifting_Link", "link4"),
            ("lifting_Link", "link5"),
            ("lifting_Link", "link6"),
            ("lifting_Link", "arm_base"),
            ("lidar_Link", "link1"),
            ("lidar_Link", "link2"),
            ("lidar_Link", "link3"),
            ("box_Link", "link3"),
            ("box_Link", "link4"),
        ]
        
        print("\n碰撞检测状态（✓=启用检测, ✗=禁用检测）：")
        print("-" * 40)
        
        for link1, link2 in key_pairs:
            # 检查是否在ACM中
            if hasattr(acm, 'get_entry'):
                entry = acm.get_entry(link1, link2)
                if entry and entry.enabled:
                    status = "✗ 禁用"
                else:
                    status = "✓ 启用"
            else:
                # 尝试其他方法
                status = "? 未知"
            
            print(f"{link1:15} <-> {link2:15} : {status}")
        
        # 列出所有禁用的碰撞对
        print("\n" + "="*60)
        print("所有禁用的碰撞对：")
        print("-" * 40)
        
        if hasattr(acm, 'get_all_entry_names'):
            all_entries = acm.get_all_entry_names()
            for i, name1 in enumerate(all_entries):
                for j, name2 in enumerate(all_entries):
                    if i < j:  # 避免重复
                        entry = acm.get_entry(name1, name2)
                        if entry and entry.enabled:
                            print(f"  {name1} <-> {name2}")
        
    except Exception as e:
        print(f"\n错误: {str(e)}")
        print("\n提示：请确保MoveIt已正确启动")

if __name__ == '__main__':
    try:
        check_collision_matrix()
    except rospy.ROSInterruptException:
        pass