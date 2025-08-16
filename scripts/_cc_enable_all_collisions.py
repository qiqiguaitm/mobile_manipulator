#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.msg import PlanningSceneComponents

def enable_critical_collisions():
    """动态启用关键的碰撞检测"""
    rospy.init_node('enable_critical_collisions', anonymous=True)
    
    print("\n" + "="*60)
    print("动态启用碰撞检测")
    print("="*60)
    
    # 等待服务
    rospy.wait_for_service('/get_planning_scene')
    rospy.wait_for_service('/apply_planning_scene')
    
    get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    
    # 获取当前场景
    req = PlanningSceneComponents()
    req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    resp = get_scene(req)
    
    current_acm = resp.scene.allowed_collision_matrix
    
    # 创建新的ACM
    new_acm = AllowedCollisionMatrix()
    new_acm.entry_names = current_acm.entry_names
    new_acm.entry_values = []
    
    # 定义需要启用碰撞检测的关键对
    critical_pairs = [
        ("lifting_Link", "link1"),
        ("lifting_Link", "link2"),
        ("lifting_Link", "link3"),
        ("lifting_Link", "link4"),
        ("lifting_Link", "link5"),
        ("lifting_Link", "link6"),
        ("lifting_Link", "gripper_base"),
        ("lifting_Link", "link7"),
        ("lifting_Link", "link8"),
        ("lidar_Link", "link1"),
        ("lidar_Link", "link2"),
        ("lidar_Link", "link3"),
        ("lidar_Link", "link4"),
        ("lidar_Link", "link5"),
        ("lidar_Link", "link6"),
    ]
    
    # 复制现有的ACM，但修改关键对
    print("\n修改碰撞矩阵...")
    modified_count = 0
    
    for i, name1 in enumerate(new_acm.entry_names):
        entry = AllowedCollisionEntry()
        entry.enabled = []
        
        for j, name2 in enumerate(new_acm.entry_names):
            # 检查是否是关键对
            is_critical = False
            for pair in critical_pairs:
                if (name1 == pair[0] and name2 == pair[1]) or \
                   (name1 == pair[1] and name2 == pair[0]):
                    is_critical = True
                    break
            
            if is_critical:
                # 启用碰撞检测（False表示不允许碰撞）
                entry.enabled.append(False)
                if i < len(current_acm.entry_values) and j < len(current_acm.entry_values[i].enabled):
                    if current_acm.entry_values[i].enabled[j]:
                        print(f"  启用: {name1} <-> {name2}")
                        modified_count += 1
            else:
                # 保持原有设置
                if i < len(current_acm.entry_values) and j < len(current_acm.entry_values[i].enabled):
                    entry.enabled.append(current_acm.entry_values[i].enabled[j])
                else:
                    entry.enabled.append(False)
        
        new_acm.entry_values.append(entry)
    
    print(f"\n修改了 {modified_count} 个碰撞对")
    
    # 应用新的场景
    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = new_acm
    
    print("\n应用新的碰撞矩阵...")
    
    result = apply_scene(scene)
    
    if result.success:
        print("✓ 成功应用新的碰撞矩阵")
        
        # 验证修改
        print("\n验证修改...")
        resp2 = get_scene(req)
        updated_acm = resp2.scene.allowed_collision_matrix
        
        # 检查关键对
        success_count = 0
        for pair in critical_pairs[:5]:  # 只检查前5个
            if pair[0] in updated_acm.entry_names and pair[1] in updated_acm.entry_names:
                idx1 = updated_acm.entry_names.index(pair[0])
                idx2 = updated_acm.entry_names.index(pair[1])
                
                if idx1 < len(updated_acm.entry_values) and idx2 < len(updated_acm.entry_values[idx1].enabled):
                    if not updated_acm.entry_values[idx1].enabled[idx2]:
                        print(f"  ✓ {pair[0]} <-> {pair[1]}: 碰撞检测已启用")
                        success_count += 1
                    else:
                        print(f"  ✗ {pair[0]} <-> {pair[1]}: 仍然允许碰撞")
        
        print(f"\n成功启用 {success_count} 个关键碰撞检测")
    else:
        print("✗ 应用失败")
    
    print("\n" + "="*60)
    print("完成！现在应该能正确检测碰撞了。")
    print("请在RViz中测试机械臂运动。")
    print("="*60)

if __name__ == '__main__':
    try:
        enable_critical_collisions()
    except rospy.ROSInterruptException:
        pass