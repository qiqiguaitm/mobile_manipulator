#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import tf2_ros

def diagnose_collision_detection():
    """诊断碰撞检测系统的问题"""
    rospy.init_node('diagnose_collision', anonymous=True)
    
    print("\n" + "="*60)
    print("MoveIt碰撞检测系统诊断")
    print("="*60)
    
    # 1. 检查TF树
    print("\n1. 检查TF树完整性:")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)  # 等待TF数据
    
    important_frames = ["world_link", "base_link", "lifting_Link", "lidar_Link", 
                       "box_Link", "link1", "link6", "gripper_base"]
    
    for frame in important_frames:
        try:
            trans = tf_buffer.lookup_transform("world_link", frame, rospy.Time(0))
            print("  ✓ %s -> world_link: OK" % frame)
        except:
            print("  ✗ %s -> world_link: 缺失或错误" % frame)
    
    # 2. 检查规划场景服务
    print("\n2. 检查MoveIt规划场景服务:")
    try:
        rospy.wait_for_service('/get_planning_scene', timeout=5.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        
        # 获取允许碰撞矩阵
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        resp = get_planning_scene(req)
        
        acm = resp.scene.allowed_collision_matrix
        print("  ✓ 规划场景服务可用")
        print("  - ACM条目数: %d" % len(acm.entry_names))
        
        # 检查关键链接是否在ACM中
        key_links = ["lifting_Link", "lidar_Link", "box_Link", "base_link"]
        missing_links = []
        for link in key_links:
            if link not in acm.entry_names:
                missing_links.append(link)
        
        if missing_links:
            print("  ✗ 以下链接不在碰撞矩阵中:")
            for link in missing_links:
                print("    - %s" % link)
        else:
            print("  ✓ 所有关键链接都在碰撞矩阵中")
            
        # 检查碰撞对
        print("\n3. 检查关键碰撞对设置:")
        collision_pairs = [
            ("link6", "lifting_Link"),
            ("link5", "box_Link"),
            ("link4", "lidar_Link")
        ]
        
        for link1, link2 in collision_pairs:
            if link1 in acm.entry_names and link2 in acm.entry_names:
                idx1 = acm.entry_names.index(link1)
                idx2 = acm.entry_names.index(link2)
                
                if idx2 < len(acm.entry_values[idx1].enabled):
                    allowed = acm.entry_values[idx1].enabled[idx2]
                    if allowed:
                        print("  ⚠ %s <-> %s: 允许碰撞 (这可能是问题所在!)" % (link1, link2))
                    else:
                        print("  ✓ %s <-> %s: 碰撞检测启用" % (link1, link2))
                else:
                    print("  ? %s <-> %s: 无法确定状态" % (link1, link2))
            else:
                print("  ✗ %s <-> %s: 链接不在ACM中" % (link1, link2))
                
    except Exception as e:
        print("  ✗ 规划场景服务错误: %s" % str(e))
    
    # 3. 检查SRDF配置
    print("\n4. 检查SRDF碰撞禁用规则:")
    try:
        srdf_param = rospy.get_param("/robot_description_semantic", "")
        if "disable_collisions" in srdf_param:
            # 计算禁用的碰撞对数量
            disable_count = srdf_param.count("<disable_collisions")
            print("  - SRDF中禁用的碰撞对数: %d" % disable_count)
            
            # 检查是否过度禁用了碰撞
            if "lifting_Link" in srdf_param and "disable_collisions" in srdf_param:
                lifting_disabled = srdf_param.count('link1="lifting_Link"') + srdf_param.count('link2="lifting_Link"')
                print("  - lifting_Link相关的禁用规则: %d条" % lifting_disabled)
                if lifting_disabled > 5:
                    print("  ⚠ 警告: lifting_Link的碰撞检测可能被过度禁用!")
    except:
        print("  ✗ 无法获取SRDF参数")
    
    # 4. 提供解决方案
    print("\n" + "="*60)
    print("诊断结果和建议:")
    print("="*60)
    print("\n如果碰撞避免不工作，可能的原因:")
    print("1. SRDF中disable_collisions规则过多")
    print("2. 碰撞网格尺寸设置不当")
    print("3. MoveIt配置未正确加载")
    print("\n建议的解决步骤:")
    print("1. 检查并减少SRDF中的disable_collisions规则")
    print("2. 确保碰撞网格正确加载")
    print("3. 在RViz中手动设置Scene Robot的碰撞显示")
    print("4. 使用Motion Planning的Query功能测试碰撞")
    print("="*60 + "\n")

if __name__ == '__main__':
    try:
        diagnose_collision_detection()
    except rospy.ROSInterruptException:
        pass