#!/usr/bin/env python3

import re
import os

def update_mesh_references(file_path):
    """更新URDF文件中的piper网格引用，使其指向本地副本"""
    
    with open(file_path, 'r') as f:
        content = f.read()
    
    original_content = content
    
    # 定义替换规则
    replacements = [
        # DAE视觉网格
        (r'package://piper_description/meshes/dae/base_link\.dae',
         'package://mobile_manipulator2_description/meshes/visual/dae/piper_base_link.dae'),
        (r'package://piper_description/meshes/dae/link(\d)\.dae',
         r'package://mobile_manipulator2_description/meshes/visual/dae/piper_link\1.dae'),
        (r'package://piper_description/meshes/dae/camera_v3\.dae',
         'package://mobile_manipulator2_description/meshes/visual/dae/piper_camera_v3.dae'),
        (r'package://piper_description/meshes/dae/gripper_base_100_v2\.dae',
         'package://mobile_manipulator2_description/meshes/visual/dae/piper_gripper_base_100_v2.dae'),
        
        # STL碰撞网格 - 标准版本
        (r'package://piper_description/meshes/base_link\.STL',
         'package://mobile_manipulator2_description/meshes/visual/stl/piper_base_link.STL'),
        (r'package://piper_description/meshes/link(\d)\.STL',
         r'package://mobile_manipulator2_description/meshes/visual/stl/piper_link\1.STL'),
        (r'package://piper_description/meshes/gripper_base\.STL',
         'package://mobile_manipulator2_description/meshes/visual/stl/piper_gripper_base.STL'),
        
        # 碰撞优化网格
        (r'package://piper_description/meshes/collision/base_link_collision\.STL',
         'package://mobile_manipulator2_description/meshes/collision/piper_base_link_collision.STL'),
        (r'package://piper_description/meshes/collision/link(\d)_collision\.STL',
         r'package://mobile_manipulator2_description/meshes/collision/piper_link\1_collision.STL'),
        (r'package://piper_description/meshes/collision/gripper_base_collision\.STL',
         'package://mobile_manipulator2_description/meshes/collision/piper_gripper_base_collision.STL'),
    ]
    
    # 执行替换
    for pattern, replacement in replacements:
        content = re.sub(pattern, replacement, content)
    
    # 检查是否有改动
    if content != original_content:
        # 备份原文件
        backup_path = file_path + '.bak'
        with open(backup_path, 'w') as f:
            f.write(original_content)
        
        # 写入新内容
        with open(file_path, 'w') as f:
            f.write(content)
        
        print(f"已更新: {file_path}")
        print(f"备份保存在: {backup_path}")
        
        # 统计替换数量
        changes = 0
        for pattern, _ in replacements:
            original_matches = len(re.findall(pattern, original_content))
            if original_matches > 0:
                changes += original_matches
        print(f"共替换了 {changes} 处引用\n")
    else:
        print(f"无需更新: {file_path}\n")

def main():
    # URDF文件列表
    urdf_files = [
        "/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf",
        "/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_performance.urdf",
        "/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/urdf/mobile_manipulator2_safety.urdf"
    ]
    
    print("开始更新URDF文件中的piper网格引用...\n")
    
    for urdf_file in urdf_files:
        if os.path.exists(urdf_file):
            update_mesh_references(urdf_file)
        else:
            print(f"文件不存在: {urdf_file}\n")
    
    print("更新完成！")
    print("\n注意：link6的碰撞模型应该使用link6_collision.STL（简化的碰撞网格），而不是link6.STL（高精度视觉网格）")

if __name__ == "__main__":
    main()