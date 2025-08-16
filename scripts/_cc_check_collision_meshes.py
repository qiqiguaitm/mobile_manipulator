#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess

def analyze_stl_file(file_path):
    """分析STL文件的顶点和面数"""
    try:
        # 使用grep统计STL文件中的面数
        result = subprocess.run(['grep', '-c', 'endfacet', file_path], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            face_count = int(result.stdout.strip())
            vertex_count = face_count * 3  # 每个三角面有3个顶点（粗略估计）
            file_size = os.path.getsize(file_path) / 1024  # KB
            return {
                'faces': face_count,
                'vertices': vertex_count,
                'size_kb': file_size
            }
    except:
        pass
    return None

def main():
    """检查所有碰撞网格的复杂度"""
    print("\n=== 碰撞网格复杂度分析 ===\n")
    
    # 需要检查的网格文件
    meshes_to_check = [
        # Mobile manipulator2描述中使用的网格
        ("base_link", "tracer2_description/meshes/base_link.STL"),
        ("box_Link", "mobile_manipulator2_description/meshes/box_Link.STL"),
        ("lifting_Link", "mobile_manipulator2_description/meshes/lifting_Link.STL"),
        ("lidar_Link", "mobile_manipulator2_description/meshes/lidar_Link.STL"),
        ("camera_Link", "mobile_manipulator2_description/meshes/camera_Link.STL"),
        
        # Piper机械臂网格
        ("arm_base", "piper_description/meshes/arm_base.STL"),
        ("link1", "piper_description/meshes/link1.STL"),
        ("link2", "piper_description/meshes/link2.STL"),
        ("link3", "piper_description/meshes/link3.STL"),
        ("link4", "piper_description/meshes/link4.STL"),
        ("link5", "piper_description/meshes/link5.STL"),
        ("link6", "piper_description/meshes/link6.STL"),
        ("gripper_base", "piper_description/meshes/gripper_base.STL"),
        ("link7", "piper_description/meshes/link7.STL"),
        ("link8", "piper_description/meshes/link8.STL"),
        
        # 检查是否有简化版本
        ("arm_base_collision", "piper_description/meshes/collision/arm_base_collision.STL"),
        ("link1_collision", "piper_description/meshes/collision/link1_collision.STL"),
        ("link2_collision", "piper_description/meshes/collision/link2_collision.STL"),
        ("link3_collision", "piper_description/meshes/collision/link3_collision.STL"),
        ("link4_collision", "piper_description/meshes/collision/link4_collision.STL"),
        ("link5_collision", "piper_description/meshes/collision/link5_collision.STL"),
        ("link6_collision", "piper_description/meshes/collision/link6_collision.STL"),
        ("gripper_base_collision", "piper_description/meshes/collision/gripper_base_collision.STL"),
        ("link7_collision", "piper_description/meshes/collision/link7_collision.STL"),
        ("link8_collision", "piper_description/meshes/collision/link8_collision.STL"),
    ]
    
    base_path = "/home/agilex/MobileManipulator/src/robot_description"
    
    high_poly_meshes = []
    
    print("| Link | 文件 | 面数 | 顶点数(估计) | 文件大小 | 状态 |")
    print("|------|------|------|--------------|----------|------|")
    
    for link_name, mesh_path in meshes_to_check:
        full_path = os.path.join(base_path, mesh_path)
        if os.path.exists(full_path):
            info = analyze_stl_file(full_path)
            if info:
                status = "⚠️ 过于复杂" if info['faces'] > 1000 else "✅ 正常"
                if info['faces'] > 1000 and "_collision" not in link_name:
                    high_poly_meshes.append((link_name, info))
                
                print(f"| {link_name} | {mesh_path.split('/')[-1]} | "
                      f"{info['faces']:,} | {info['vertices']:,} | "
                      f"{info['size_kb']:.1f} KB | {status} |")
    
    print("\n=== 需要优化的高面数网格 ===\n")
    if high_poly_meshes:
        for link_name, info in sorted(high_poly_meshes, key=lambda x: x[1]['faces'], reverse=True):
            print(f"❌ {link_name}: {info['faces']:,} 个面，{info['size_kb']:.1f} KB")
            
            # 检查是否已有碰撞版本
            if "_collision" not in link_name:
                collision_name = link_name + "_collision"
                has_collision = any(name == collision_name for name, _ in meshes_to_check)
                if has_collision:
                    print(f"   ✅ 已有简化碰撞网格: {collision_name}")
                else:
                    print(f"   ⚠️  缺少简化碰撞网格!")
    else:
        print("✅ 所有网格都在合理范围内")
    
    print("\n=== 建议 ===")
    print("1. MoveIt推荐碰撞网格面数 < 1000")
    print("2. 可以使用MeshLab或Blender简化网格")
    print("3. 简化率建议：保留5-10%的面数")

if __name__ == '__main__':
    main()