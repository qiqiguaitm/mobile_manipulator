#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, CollisionObject
import sys

def debug_collision_issue():
    """深入调试碰撞检测问题"""
    rospy.init_node('debug_collision_issue', anonymous=True)
    
    print("\n" + "="*80)
    print("调试碰撞检测问题")
    print("="*80)
    
    try:
        # 初始化MoveIt
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        
        # 等待场景服务
        rospy.wait_for_service('/get_planning_scene', timeout=5.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        
        # 获取完整的规划场景
        req = moveit_msgs.msg.PlanningSceneComponents()
        req.components = (
            moveit_msgs.msg.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            moveit_msgs.msg.PlanningSceneComponents.LINK_PADDING_AND_SCALING |
            moveit_msgs.msg.PlanningSceneComponents.ROBOT_STATE
        )
        
        resp = get_planning_scene(req)
        planning_scene = resp.scene
        
        # 1. 检查ACM (Allowed Collision Matrix)
        print("\n1. 检查碰撞矩阵 (ACM):")
        print("-" * 60)
        
        acm = planning_scene.allowed_collision_matrix
        print(f"  ACM条目数: {len(acm.entry_names)}")
        print(f"  链接名称: {', '.join(acm.entry_names[:10])}...")
        
        # 查找lifting_Link相关的条目
        lifting_entries = []
        for i, name in enumerate(acm.entry_names):
            if name == "lifting_Link":
                lifting_idx = i
                print(f"\n  lifting_Link索引: {i}")
                
                # 检查与其他链接的碰撞设置
                for j, other_name in enumerate(acm.entry_names):
                    if other_name.startswith("link") and other_name != "lifting_Link":
                        # 获取碰撞矩阵值
                        if i < len(acm.entry_values) and j < len(acm.entry_values[i].enabled):
                            enabled = acm.entry_values[i].enabled[j]
                            if enabled:
                                print(f"    ✗ {name} <-> {other_name}: 碰撞检测被禁用")
                            else:
                                print(f"    ✓ {name} <-> {other_name}: 碰撞检测启用")
                
        # 2. 检查链接填充和缩放
        print("\n2. 检查链接填充 (Link Padding):")
        print("-" * 60)
        
        padding = planning_scene.link_padding
        scale = planning_scene.link_scale
        
        for link_pad in padding:
            if "lifting" in link_pad.link_name or "link" in link_pad.link_name:
                print(f"  {link_pad.link_name}: padding = {link_pad.padding}")
        
        # 3. 检查机器人当前状态
        print("\n3. 检查机器人当前状态:")
        print("-" * 60)
        
        current_state = robot.get_current_state()
        print(f"  关节位置:")
        for name, pos in zip(current_state.joint_state.name, current_state.joint_state.position):
            if name.startswith("joint"):
                print(f"    {name}: {pos:.3f}")
        
        # 4. 测试碰撞检测
        print("\n4. 测试碰撞检测:")
        print("-" * 60)
        
        # 创建一个测试组
        try:
            group = MoveGroupCommander("piper")
            
            # 获取当前位置
            current_pose = group.get_current_pose().pose
            print(f"  当前末端执行器位置: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            
            # 检查当前状态是否有碰撞
            robot_state = robot.get_current_state()
            
            # 创建一个碰撞检查请求
            collision_req = moveit_msgs.msg.GetStateValidity.Request()
            collision_req.robot_state = robot_state
            collision_req.group_name = "piper"
            
            print("\n  尝试检查当前状态的碰撞...")
            
        except Exception as e:
            print(f"  组操作错误: {str(e)}")
        
        # 5. 检查URDF中的碰撞几何体
        print("\n5. 检查碰撞几何体:")
        print("-" * 60)
        
        links = robot.get_link_names()
        critical_links = ["lifting_Link", "link1", "link2", "link3", "arm_base"]
        
        for link in critical_links:
            if link in links:
                print(f"  ✓ {link} 存在于机器人模型中")
            else:
                print(f"  ✗ {link} 不存在于机器人模型中")
        
        # 6. 检查场景中的碰撞对象
        print("\n6. 检查场景中的碰撞对象:")
        print("-" * 60)
        
        known_objects = scene.get_known_object_names()
        if known_objects:
            print(f"  场景对象: {', '.join(known_objects)}")
        else:
            print("  场景中没有额外的碰撞对象")
        
        # 7. 建议
        print("\n" + "="*80)
        print("诊断总结和建议:")
        print("="*80)
        
        print("\n可能的问题:")
        print("1. lifting_Link与机械臂链接之间的碰撞检测可能没有正确配置")
        print("2. 碰撞几何体可能太小或位置不正确")
        print("3. MoveIt可能没有正确加载SRDF配置")
        
        print("\n建议的解决方案:")
        print("1. 确保重新构建和加载了修改后的SRDF")
        print("2. 检查URDF中lifting_Link的碰撞几何体定义")
        print("3. 手动测试碰撞检测功能")
        
    except Exception as e:
        print(f"\n错误: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    try:
        debug_collision_issue()
    except rospy.ROSInterruptException:
        pass