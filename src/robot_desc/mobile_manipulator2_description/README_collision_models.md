# Mobile Manipulator2 双碰撞模型系统

## 概述

实现了两套碰撞模型，可根据不同场景选择：

- **PERFORMANCE模式**（默认）：简化碰撞模型，快速加载和计算
- **SAFETY模式**：原始精确模型，适合精密操作

## 性能对比

| 模式 | 文件大小 | MoveIt加载时间 | 碰撞检测速度 | 适用场景 |
|------|----------|----------------|--------------|----------|
| PERFORMANCE | 88KB | <5秒 | 快10-100倍 | 路径规划、仿真、日常操作 |
| SAFETY | 30MB | >30秒 | 基准速度 | 精密操作、安全验证 |

## 使用方法

### 1. 默认启动（PERFORMANCE模式）
```bash
roslaunch mobile_manipulator2_description load_description.launch
```

### 2. 指定SAFETY模式
```bash
roslaunch mobile_manipulator2_description load_description.launch collision_mode:=safety
```

### 3. 在其他launch文件中使用
```xml
<include file="$(find mobile_manipulator2_description)/launch/load_description.launch">
  <arg name="collision_mode" value="performance"/>
</include>
```

## 简化策略说明

| 组件 | 简化方法 | 安全裕度 |
|------|----------|----------|
| base_link | 凸包 | +20mm |
| box_Link | 凸包 | +15mm |
| lifting_Link | 复合模型（结构+显示器保护） | +15/25mm |
| lidar_Link | 圆柱体 | +30mm |
| cameras | 长方体 | +15mm |
| piper links | 使用官方collision模型 | +5-10mm |

## 技术实现

- 原始模型：`meshes/visual/`
- 简化模型：`meshes/collision/`
- URDF文件：
  - `urdf/mobile_manipulator2_safety.urdf`
  - `urdf/mobile_manipulator2_performance.urdf`