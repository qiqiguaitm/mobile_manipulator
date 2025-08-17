# 碰撞模型可视化集成指南

## 概述
碰撞模型可视化功能已经集成到标准的 `moveit_full.rviz` 配置文件中。现在可以在统一的界面中使用 Motion Planning 面板和碰撞模型显示。

## 使用方法

### 启动MoveIt演示
```bash
# 仿真模式
roslaunch arm_planner main_demo.launch mode:=fake

# 实机模式
roslaunch arm_planner main_demo.launch mode:=real

# 不带夹爪
roslaunch arm_planner main_demo.launch gripper:=false
```

### 在RViz中切换显示

1. **Visual Robot Model**（视觉模型）
   - 默认启用，显示机器人的实际外观
   - Alpha值设为0.8，半透明显示
   - 可以通过取消勾选来隐藏

2. **Collision Model**（碰撞模型）
   - 默认禁用，需要手动勾选启用
   - 显示碰撞检测使用的简化模型
   - Alpha值设为0.5，线框模式显示
   - 红色/紫色线框表示碰撞几何体

3. **MotionPlanning**（运动规划）
   - 包含完整的MoveIt功能
   - Scene Robot部分也可以显示碰撞模型
   - 可以实时查看碰撞检测状态

## 切换技巧

1. **快速切换视图**：
   - 只看视觉模型：启用 Visual Robot Model，禁用 Collision Model
   - 只看碰撞模型：禁用 Visual Robot Model，启用 Collision Model
   - 对比视图：同时启用两者

2. **Motion Planning中的碰撞显示**：
   - 在 MotionPlanning > Scene Robot 中
   - 勾选 "Show Robot Collision" 显示碰撞
   - 取消勾选 "Show Robot Visual" 隐藏视觉模型

3. **调整透明度**：
   - 修改 Alpha 值来调整模型透明度
   - 建议值：Visual Model (0.8), Collision Model (0.5)

## 优势

1. **统一界面**：不需要切换不同的配置文件
2. **灵活控制**：可以随时开关各种显示
3. **完整功能**：保留所有 Motion Planning 功能
4. **实时对比**：方便验证碰撞模型的准确性

## 注意事项

- 碰撞模型应该略大于视觉模型，提供安全缓冲
- 默认情况下 Collision Model 是关闭的，需要手动启用
- 在进行路径规划时，MoveIt使用的是碰撞模型，不是视觉模型