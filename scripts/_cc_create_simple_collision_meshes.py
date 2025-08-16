#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np

def create_box_stl(width, height, depth, filename):
    """创建一个简单的立方体STL文件"""
    
    # 定义立方体的8个顶点
    vertices = np.array([
        [-width/2, -height/2, -depth/2],
        [ width/2, -height/2, -depth/2],
        [ width/2,  height/2, -depth/2],
        [-width/2,  height/2, -depth/2],
        [-width/2, -height/2,  depth/2],
        [ width/2, -height/2,  depth/2],
        [ width/2,  height/2,  depth/2],
        [-width/2,  height/2,  depth/2]
    ])
    
    # 定义12个三角形面（每个立方体面由2个三角形组成）
    faces = [
        # 底面
        [0, 1, 2], [0, 2, 3],
        # 顶面
        [4, 6, 5], [4, 7, 6],
        # 前面
        [0, 4, 5], [0, 5, 1],
        # 后面
        [2, 6, 7], [2, 7, 3],
        # 左面
        [0, 3, 7], [0, 7, 4],
        # 右面
        [1, 5, 6], [1, 6, 2]
    ]
    
    # 写入ASCII STL文件
    with open(filename, 'w') as f:
        f.write("solid box\n")
        
        for face in faces:
            # 计算法向量
            v0 = vertices[face[0]]
            v1 = vertices[face[1]]
            v2 = vertices[face[2]]
            
            edge1 = v1 - v0
            edge2 = v2 - v0
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal)
            
            f.write(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
            f.write("    outer loop\n")
            for vertex_idx in face:
                v = vertices[vertex_idx]
                f.write(f"      vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        
        f.write("endsolid box\n")

def create_cylinder_stl(radius, height, segments, filename):
    """创建一个简单的圆柱体STL文件"""
    
    vertices = []
    faces = []
    
    # 创建顶点
    # 底部中心
    vertices.append([0, 0, -height/2])
    # 底部圆周
    for i in range(segments):
        angle = 2 * np.pi * i / segments
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        vertices.append([x, y, -height/2])
    
    # 顶部中心
    vertices.append([0, 0, height/2])
    # 顶部圆周
    for i in range(segments):
        angle = 2 * np.pi * i / segments
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        vertices.append([x, y, height/2])
    
    vertices = np.array(vertices)
    
    # 创建面
    # 底面
    for i in range(segments):
        next_i = (i + 1) % segments
        faces.append([0, i + 1, next_i + 1])
    
    # 顶面
    top_center = segments + 1
    for i in range(segments):
        next_i = (i + 1) % segments
        faces.append([top_center, top_center + next_i + 1, top_center + i + 1])
    
    # 侧面
    for i in range(segments):
        next_i = (i + 1) % segments
        # 下三角
        faces.append([i + 1, next_i + 1, top_center + i + 1])
        # 上三角
        faces.append([next_i + 1, top_center + next_i + 1, top_center + i + 1])
    
    # 写入STL文件
    with open(filename, 'w') as f:
        f.write("solid cylinder\n")
        
        for face in faces:
            # 计算法向量
            v0 = vertices[face[0]]
            v1 = vertices[face[1]]
            v2 = vertices[face[2]]
            
            edge1 = v1 - v0
            edge2 = v2 - v0
            normal = np.cross(edge1, edge2)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            
            f.write(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
            f.write("    outer loop\n")
            for vertex_idx in face:
                v = vertices[vertex_idx]
                f.write(f"      vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        
        f.write("endsolid cylinder\n")

def main():
    """创建简化的碰撞网格"""
    
    # 创建collision目录
    collision_dir = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/meshes/collision"
    os.makedirs(collision_dir, exist_ok=True)
    
    print("创建简化碰撞网格...")
    
    # 1. box_Link - 用更小的立方体代替（避免与机械臂碰撞）
    # 根据实际尺寸估计（宽x高x深）- 减小尺寸
    box_file = os.path.join(collision_dir, "box_Link_collision.STL")
    create_box_stl(0.3, 0.15, 0.25, box_file)  # 缩小尺寸
    print(f"✅ 创建 box_Link_collision.STL")
    
    # 2. lidar_Link - 用圆柱体代替
    # 激光雷达通常是圆柱形
    lidar_file = os.path.join(collision_dir, "lidar_Link_collision.STL")
    create_cylinder_stl(0.04, 0.06, 8, lidar_file)  # 半径4cm，高6cm，8边形
    print(f"✅ 创建 lidar_Link_collision.STL")
    
    # 3. lifting_Link - 用细长的立方体代替
    lifting_file = os.path.join(collision_dir, "lifting_Link_collision.STL")
    create_box_stl(0.06, 0.4, 0.06, lifting_file)  # 6cm x 40cm x 6cm
    print(f"✅ 创建 lifting_Link_collision.STL")
    
    # 4. base_link - 用立方体代替（减小高度避免与link2碰撞）
    base_file = os.path.join(collision_dir, "base_link_collision.STL")
    create_box_stl(0.5, 0.08, 0.35, base_file)  # 50cm x 8cm x 35cm
    print(f"✅ 创建 base_link_collision.STL")
    
    # 5. camera links - 用小立方体代替
    for camera in ["top_camera_Link", "under_camera_Link"]:
        camera_file = os.path.join(collision_dir, f"{camera}_collision.STL")
        create_box_stl(0.06, 0.04, 0.08, camera_file)  # 6cm x 4cm x 8cm
        print(f"✅ 创建 {camera}_collision.STL")
    
    # 检查文件大小
    print("\n文件大小检查:")
    for filename in os.listdir(collision_dir):
        if filename.endswith('.STL'):
            filepath = os.path.join(collision_dir, filename)
            size = os.path.getsize(filepath) / 1024  # KB
            print(f"  {filename}: {size:.1f} KB")
    
    print("\n✅ 所有简化碰撞网格创建完成！")
    print("这些简化网格将大幅提升MoveIt的性能。")

if __name__ == '__main__':
    main()