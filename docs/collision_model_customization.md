# 碰撞模型颜色和透明度自定义指南

## 概述
碰撞模型的显示属性（颜色、透明度）现在是可调节的，不再是固定值。用户可以根据需要在 RViz 中实时调整。

## 调整方法

### 1. Collision Model（默认紫色）
在 RViz 的 Displays 面板中找到 "Collision Model"：

- **Alpha**：调整整体透明度（0-1，默认0.5）
  - 0 = 完全透明
  - 1 = 完全不透明
  
- **Links**：展开后可以看到所有链接
  - 点击任意链接（如 link1, link2 等）
  - 每个链接都可以单独设置：
    - Alpha：单独调整该链接的透明度
    - Show Axes：显示坐标轴
    - Show Trail：显示轨迹
    - Value：启用/禁用该链接的显示

### 2. Collision Model (Red)
在 "Collision Model (Red)" > Scene Robot 中：

- **Robot Alpha**：调整机器人整体透明度（默认0.5）
- **Attached Body Color**：调整附加物体的颜色
- **Links**：同样可以展开调整每个链接的属性

### 3. MotionPlanning 中的碰撞显示
在 "MotionPlanning" > Scene Robot 中：

- **Robot Alpha**：调整透明度
- **Show Robot Collision**：显示/隐藏碰撞模型
- **Show Robot Visual**：显示/隐藏视觉模型

## 颜色说明

### RobotModel 的碰撞颜色
- 默认颜色：紫色/品红色
- 这是 RViz 内部硬编码的颜色
- 无法通过配置文件更改
- 但可以通过调整 Alpha 值改变视觉效果

### PlanningScene 的碰撞颜色
- 默认颜色：红色（MoveIt 标准）
- 表示潜在的碰撞危险
- 在实际碰撞时会变得更亮

## 推荐设置

### 清晰对比视图
- Visual Robot Model: Alpha = 0.8（半透明绿色）
- Collision Model: Alpha = 0.5（半透明紫色）
- 两者同时显示，便于对比

### 碰撞调试视图
- Visual Robot Model: 禁用
- Collision Model (Red): Alpha = 0.7
- 只显示碰撞模型，专注于碰撞检测

### 路径规划视图
- 使用 MotionPlanning 面板
- Scene Robot > Robot Alpha = 0.5
- Show Robot Collision = true
- Show Robot Visual = false

## 技巧

1. **快速切换**：使用复选框快速启用/禁用不同的显示
2. **保存配置**：调整好后，可以通过 File > Save Config 保存当前设置
3. **重置默认**：如果调乱了，可以重新加载配置文件恢复默认设置