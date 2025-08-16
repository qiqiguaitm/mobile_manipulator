#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import shutil
import os

def fix_lifting_collision_geometry():
    """修复lifting_Link的碰撞几何体为简单圆柱体"""
    
    urdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf"
    backup_path = urdf_path + ".backup_collision_geom"
    
    print("修复lifting_Link碰撞几何体...")
    
    # 备份
    shutil.copy2(urdf_path, backup_path)
    print(f"已备份到: {backup_path}")
    
    # 读取文件
    with open(urdf_path, 'r') as f:
        content = f.read()
    
    # 查找并替换lifting_Link的碰撞定义
    # 原始的碰撞定义使用mesh
    old_collision = '''    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://mobile_manipulator2_description/meshes/collision/lifting_Link_collision.STL"/>
      </geometry>
    </collision>'''
    
    # 新的碰撞定义使用圆柱体
    # lifting_Link是一个垂直支柱，高度约0.56m，直径约0.06m
    new_collision = '''    <collision>
      <origin rpy="0 0 0" xyz="0 0 0.28"/>
      <geometry>
        <cylinder radius="0.04" length="0.56"/>
      </geometry>
    </collision>'''
    
    if old_collision in content:
        content = content.replace(old_collision, new_collision)
        print("✓ 已将lifting_Link碰撞几何体替换为圆柱体")
        print("  - 半径: 0.04m")
        print("  - 高度: 0.56m")
        print("  - 中心: z=0.28m")
    else:
        print("✗ 未找到预期的碰撞定义，尝试其他方法...")
        
        # 使用正则表达式查找
        import re
        pattern = r'(<link name="lifting_Link">.*?</link>)'
        match = re.search(pattern, content, re.DOTALL)
        
        if match:
            link_content = match.group(1)
            # 查找collision部分
            collision_pattern = r'(<collision>.*?</collision>)'
            collision_match = re.search(collision_pattern, link_content, re.DOTALL)
            
            if collision_match:
                old_collision_found = collision_match.group(1)
                new_link_content = link_content.replace(old_collision_found, new_collision)
                content = content.replace(link_content, new_link_content)
                print("✓ 使用正则表达式成功替换碰撞几何体")
    
    # 同时修复lidar_Link的碰撞几何体
    print("\n同时修复lidar_Link的碰撞几何体...")
    
    # lidar的碰撞定义
    lidar_old = '''<mesh filename="package://mobile_manipulator2_description/meshes/collision/lidar_Link_collision.STL"/>'''
    lidar_new = '''<box size="0.18 0.14 0.09"/>'''
    
    if lidar_old in content:
        content = content.replace(lidar_old, lidar_new)
        print("✓ 已将lidar_Link碰撞几何体替换为盒子")
        print("  - 尺寸: 0.18 x 0.14 x 0.09m")
    
    # 保存修改
    with open(urdf_path, 'w') as f:
        f.write(content)
    
    print("\n修改完成！")
    print("\n后续步骤：")
    print("1. 重新加载robot_description参数")
    print("2. 重启MoveIt节点")
    print("3. 再次测试碰撞检测")
    
    # 生成加载脚本
    load_script = '''#!/bin/bash
# 重新加载robot_description
source ~/MobileManipulator/devel/setup.bash
rosparam load ~/MobileManipulator/src/robot_description/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf robot_description
echo "已重新加载robot_description"
'''
    
    script_path = "/home/agilex/MobileManipulator/scripts/_cc_reload_robot_description.sh"
    with open(script_path, 'w') as f:
        f.write(load_script)
    os.chmod(script_path, 0o755)
    print(f"\n创建了重新加载脚本: {script_path}")

if __name__ == '__main__':
    fix_lifting_collision_geometry()