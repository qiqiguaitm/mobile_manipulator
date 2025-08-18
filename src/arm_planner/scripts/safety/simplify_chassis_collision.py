#!/usr/bin/env python3
"""
底盘碰撞模型简化工具 - 为Tracer2底盘创建简化的碰撞几何
"""

import os
import trimesh
import numpy as np
from pathlib import Path

def simplify_mesh(input_path, output_path, target_faces=1000, use_convex_hull=False):
    """简化网格模型"""
    try:
        # 加载原始网格
        mesh = trimesh.load_mesh(input_path)
        print(f"Original mesh: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
        
        if use_convex_hull:
            # 使用凸包
            simplified = mesh.convex_hull
            print("Using convex hull approximation")
        else:
            # 使用quadric decimation简化
            simplified = mesh.simplify_quadric_decimation(target_faces)
            print(f"Simplified to {target_faces} target faces")
        
        # 确保网格是watertight
        if not simplified.is_watertight:
            simplified.fill_holes()
        
        # 保存简化后的网格
        simplified.export(output_path)
        print(f"Simplified mesh: {len(simplified.vertices)} vertices, {len(simplified.faces)} faces")
        
        # 计算简化率
        reduction = (1 - len(simplified.faces) / len(mesh.faces)) * 100
        print(f"Reduction: {reduction:.1f}%")
        
        return True
        
    except Exception as e:
        print(f"Error processing {input_path}: {e}")
        return False

def create_primitive_collision(component_name, output_path, dimensions):
    """为组件创建基本几何形状的碰撞模型"""
    
    if component_name == "base":
        # 底盘主体 - 使用盒子
        box = trimesh.creation.box(extents=dimensions)
        box.export(output_path)
        
    elif "wheel" in component_name:
        # 轮子 - 使用圆柱体
        cylinder = trimesh.creation.cylinder(
            radius=dimensions[0], 
            height=dimensions[1],
            sections=16  # 降低分辨率
        )
        cylinder.export(output_path)
        
    elif "suspension" in component_name:
        # 悬挂 - 使用简化的盒子
        box = trimesh.creation.box(extents=dimensions)
        box.export(output_path)
    
    print(f"Created primitive collision for {component_name}")

def process_chassis_models():
    """处理所有底盘碰撞模型"""
    
    # 路径配置
    base_path = Path("/home/agilex/MobileManipulator/src/robot_desc")
    tracer_path = base_path / "tracer2_description/meshes"
    
    # 创建碰撞模型目录
    collision_dir = tracer_path / "collision"
    collision_dir.mkdir(exist_ok=True)
    
    # 需要简化的组件列表
    components = {
        "base_link.STL": {
            "target_faces": 500,
            "use_convex_hull": True,
            "primitive": {"type": "box", "dimensions": [0.6, 0.4, 0.2]}
        },
        "fl_wheel_link.STL": {
            "target_faces": 200,
            "primitive": {"type": "cylinder", "dimensions": [0.15, 0.08]}
        },
        "fr_wheel_link.STL": {
            "target_faces": 200,
            "primitive": {"type": "cylinder", "dimensions": [0.15, 0.08]}
        },
        "rl_wheel_link.STL": {
            "target_faces": 200,
            "primitive": {"type": "cylinder", "dimensions": [0.15, 0.08]}
        },
        "rr_wheel_link.STL": {
            "target_faces": 200,
            "primitive": {"type": "cylinder", "dimensions": [0.15, 0.08]}
        },
        "lifting_link.STL": {
            "target_faces": 300,
            "use_convex_hull": True,
            "primitive": {"type": "box", "dimensions": [0.4, 0.3, 0.05]}
        }
    }
    
    # 处理每个组件
    for filename, config in components.items():
        input_file = tracer_path / filename
        
        if input_file.exists():
            # 方法1: 网格简化
            output_file = collision_dir / filename
            success = simplify_mesh(
                input_file, 
                output_file,
                target_faces=config.get("target_faces", 500),
                use_convex_hull=config.get("use_convex_hull", False)
            )
            
            # 方法2: 创建基本几何形状（备选）
            if "primitive" in config:
                primitive_file = collision_dir / f"primitive_{filename}"
                component_name = filename.replace(".STL", "").lower()
                create_primitive_collision(
                    component_name,
                    primitive_file,
                    config["primitive"]["dimensions"]
                )
        else:
            print(f"Warning: {input_file} not found")
    
    print("\n=== 简化完成 ===")
    print(f"碰撞模型保存在: {collision_dir}")
    
    # 生成URDF更新建议
    print("\n=== URDF更新建议 ===")
    print("在URDF文件中，将碰撞几何替换为简化版本：")
    print("```xml")
    print("<collision>")
    print("  <geometry>")
    print('    <mesh filename="package://tracer2_description/meshes/collision/base_link.STL"/>')
    print("  </geometry>")
    print("</collision>")
    print("```")

if __name__ == "__main__":
    # 检查依赖
    try:
        import trimesh
        print("Trimesh version:", trimesh.__version__)
    except ImportError:
        print("请先安装trimesh: pip3 install trimesh")
        exit(1)
    
    # 执行简化
    process_chassis_models()