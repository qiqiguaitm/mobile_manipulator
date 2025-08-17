# 红色碰撞模型显示说明

## 概述
由于 RViz 的 RobotModel 插件限制，碰撞模型默认显示为紫色/品红色，无法直接改变颜色。但我们提供了两种显示碰撞模型的方式。

## 显示选项

### 1. Collision Model（紫色，默认）
- **类型**：rviz/RobotModel
- **颜色**：紫色/品红色（硬编码，无法更改）
- **特点**：线框显示，轻量级
- **用途**：快速查看碰撞几何形状

### 2. Collision Model (Red)（红色）
- **类型**：moveit_rviz_plugin/PlanningScene
- **颜色**：红色（MoveIt标准碰撞颜色）
- **特点**：实体显示，集成MoveIt功能
- **用途**：更准确的碰撞检测可视化

## 使用方法

启动后，在 RViz 的 Displays 面板中：

1. **查看紫色碰撞模型**：
   - 勾选 "Collision Model"
   - 这会显示紫色线框的碰撞模型

2. **查看红色碰撞模型**：
   - 勾选 "Collision Model (Red)"
   - 这会显示红色实体的碰撞模型
   - 注意：需要 MoveIt 正在运行

3. **同时查看两种模型**：
   - 可以同时勾选两个选项
   - 建议调整 Alpha 值避免视觉混乱

## 技术说明

### 为什么有两种显示方式？
- RobotModel 的碰撞显示颜色是硬编码的
- PlanningScene 使用 MoveIt 的碰撞检测系统，自然显示为红色
- 两种方式各有优势：
  - RobotModel：轻量、快速、不依赖 MoveIt
  - PlanningScene：准确、红色警示、集成碰撞检测

### MotionPlanning 面板中的碰撞显示
在 MotionPlanning > Scene Robot 中：
- "Show Robot Collision" = true：显示碰撞模型
- "Show Robot Visual" = false：隐藏视觉模型
- 默认已配置为显示红色碰撞模型

## 推荐配置

1. **日常开发**：使用 "Visual Robot Model" + "Collision Model"
2. **碰撞调试**：使用 "Visual Robot Model" + "Collision Model (Red)"
3. **路径规划**：使用 MotionPlanning 面板的内置显示