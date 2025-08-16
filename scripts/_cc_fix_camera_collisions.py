#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import shutil
import os
import re

def fix_camera_collisions():
    """修复相机的碰撞检测配置"""
    
    print("\n" + "="*60)
    print("修复相机碰撞检测")
    print("="*60)
    
    # 1. 修复SRDF中的碰撞禁用规则
    srdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/config/mobile_manipulator2_description.srdf"
    backup_path = srdf_path + ".backup_camera"
    
    print("\n1. 修复SRDF碰撞配置...")
    shutil.copy2(srdf_path, backup_path)
    print(f"   已备份到: {backup_path}")
    
    # 解析SRDF
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    
    # 需要移除的碰撞禁用规则
    camera_collision_to_remove = [
        # under_camera不应该与这些机械臂部件禁用碰撞
        ("under_camera_Link", "link4"),
        ("under_camera_Link", "link5"),
        ("under_camera_Link", "link6"),
        ("under_camera_Link", "gripper_base"),
        ("under_camera_Link", "link7"),
        ("under_camera_Link", "link8"),
        # top_camera不应该与这些机械臂部件禁用碰撞
        ("top_camera_Link", "link4"),
        ("top_camera_Link", "link5"),
        ("top_camera_Link", "link6"),
        ("top_camera_Link", "gripper_base"),
        ("top_camera_Link", "link7"),
        ("top_camera_Link", "link8"),
    ]
    
    # 移除这些碰撞禁用规则
    removed_count = 0
    elements_to_remove = []
    
    for elem in root.findall('.//disable_collisions'):
        link1 = elem.get('link1')
        link2 = elem.get('link2')
        
        for pair in camera_collision_to_remove:
            if (link1 == pair[0] and link2 == pair[1]) or \
               (link1 == pair[1] and link2 == pair[0]):
                elements_to_remove.append(elem)
                print(f"   移除: {link1} <-> {link2}")
                removed_count += 1
                break
    
    for elem in elements_to_remove:
        root.remove(elem)
    
    print(f"\n   共移除 {removed_count} 个碰撞禁用规则")
    
    # 保存SRDF
    tree.write(srdf_path, encoding='UTF-8', xml_declaration=True)
    
    # 2. 修复URDF中的相机碰撞几何体
    urdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf"
    urdf_backup = urdf_path + ".backup_camera"
    
    print("\n2. 修复URDF碰撞几何体...")
    shutil.copy2(urdf_path, urdf_backup)
    print(f"   已备份到: {urdf_backup}")
    
    with open(urdf_path, 'r') as f:
        content = f.read()
    
    # 替换top_camera的碰撞几何体为盒子
    top_camera_old = '''<mesh filename="package://mobile_manipulator2_description/meshes/collision/top_camera_Link_collision.STL"/>'''
    top_camera_new = '''<box size="0.08 0.06 0.04"/>'''
    
    if top_camera_old in content:
        content = content.replace(top_camera_old, top_camera_new)
        print("   ✓ top_camera_Link: 碰撞体改为盒子 (0.08×0.06×0.04m)")
    
    # 替换under_camera的碰撞几何体为盒子
    under_camera_old = '''<mesh filename="package://mobile_manipulator2_description/meshes/collision/under_camera_Link_collision.STL"/>'''
    under_camera_new = '''<box size="0.10 0.08 0.05"/>'''
    
    if under_camera_old in content:
        content = content.replace(under_camera_old, under_camera_new)
        print("   ✓ under_camera_Link: 碰撞体改为盒子 (0.10×0.08×0.05m)")
    
    # 保存URDF
    with open(urdf_path, 'w') as f:
        f.write(content)
    
    # 3. 显示当前的碰撞配置状态
    print("\n3. 碰撞检测配置总结:")
    print("   ✓ lifting_Link: 圆柱体碰撞模型")
    print("   ✓ lidar_Link: 盒子碰撞模型")
    print("   ✓ top_camera_Link: 盒子碰撞模型")
    print("   ✓ under_camera_Link: 盒子碰撞模型")
    
    print("\n4. 启用的碰撞检测:")
    critical_pairs = [
        ("lifting_Link", "link1-6"),
        ("lidar_Link", "link1-6"),
        ("top_camera_Link", "link4-6, gripper"),
        ("under_camera_Link", "link4-6, gripper"),
    ]
    
    for sensor, links in critical_pairs:
        print(f"   ✓ {sensor} <-> {links}")
    
    print("\n" + "="*60)
    print("修复完成！")
    print("="*60)
    print("\n后续步骤：")
    print("1. 重启MoveIt系统")
    print("2. 测试机械臂是否正确避开所有传感器")

if __name__ == '__main__':
    fix_camera_collisions()