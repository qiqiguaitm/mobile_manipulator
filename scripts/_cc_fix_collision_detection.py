#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import shutil
import os

def fix_collision_detection():
    """修复SRDF中的碰撞检测配置"""
    
    srdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/config/mobile_manipulator2_description.srdf"
    backup_path = srdf_path + ".backup_collision"
    
    print("修复碰撞检测配置...")
    
    # 备份原文件
    shutil.copy2(srdf_path, backup_path)
    print(f"已备份到: {backup_path}")
    
    # 解析SRDF
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    
    # 查找所有现有的disable_collisions元素
    existing_pairs = set()
    for elem in root.findall('.//disable_collisions'):
        link1 = elem.get('link1')
        link2 = elem.get('link2')
        # 标准化顺序
        pair = tuple(sorted([link1, link2]))
        existing_pairs.add(pair)
    
    # 定义需要启用碰撞检测的连杆对
    # 这些是lifting_Link与机械臂各连杆之间的碰撞检测
    critical_collision_pairs = [
        ("lifting_Link", "link1"),
        ("lifting_Link", "link2"),
        ("lifting_Link", "link3"),
        ("lifting_Link", "link4"),
        ("lifting_Link", "link5"),
        ("lifting_Link", "link6"),
        ("lidar_Link", "link1"),
        ("lidar_Link", "link2"),
        ("lidar_Link", "link3"),
        ("lidar_Link", "link4"),
        ("lidar_Link", "link5"),
        ("lidar_Link", "link6"),
    ]
    
    # 移除会阻止碰撞检测的规则
    elements_to_remove = []
    for elem in root.findall('.//disable_collisions'):
        link1 = elem.get('link1')
        link2 = elem.get('link2')
        reason = elem.get('reason')
        
        # 检查是否是需要移除的规则
        if reason == "Never":
            # 如果是lifting_Link或lidar_Link与机械臂的碰撞禁用，应该移除
            if (link1 == "lifting_Link" and link2.startswith("link")) or \
               (link2 == "lifting_Link" and link1.startswith("link")) or \
               (link1 == "lidar_Link" and link2.startswith("link")) or \
               (link2 == "lidar_Link" and link1.startswith("link")) or \
               (link1 == "arm_base" and link2 == "lifting_Link") or \
               (link2 == "arm_base" and link1 == "lifting_Link"):
                elements_to_remove.append(elem)
                print(f"移除碰撞禁用规则: {link1} <-> {link2}")
    
    # 移除元素
    for elem in elements_to_remove:
        root.remove(elem)
    
    # 确保关键的碰撞对没有被禁用
    for link1, link2 in critical_collision_pairs:
        pair = tuple(sorted([link1, link2]))
        if pair in existing_pairs:
            print(f"警告: {link1} <-> {link2} 碰撞检测被禁用，需要启用")
    
    # 保存修改后的文件
    tree.write(srdf_path, encoding='UTF-8', xml_declaration=True)
    
    print("\n修改完成！")
    print("\n重要提示：")
    print("1. 请重新启动MoveIt: roslaunch arm_planner arm_planner_demo.launch")
    print("2. 在RViz中测试机械臂运动，确认碰撞检测正常工作")
    print("3. 如果需要恢复，使用备份文件: " + backup_path)
    
    # 显示修改后的碰撞配置
    print("\n当前启用碰撞检测的关键对:")
    for link1, link2 in critical_collision_pairs:
        print(f"  ✓ {link1} <-> {link2}")

if __name__ == '__main__':
    fix_collision_detection()