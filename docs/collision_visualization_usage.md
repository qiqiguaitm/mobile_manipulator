# 碰撞模型可视化使用指南

## 概述
本功能允许在RViz中同时显示机器人的视觉模型（Visual Model）和碰撞模型（Collision Model），方便验证碰撞检测的准确性和安全性。

## 使用方法

### 1. 通过 main_demo.launch 启动（推荐）

```bash
# 标准MoveIt可视化（默认）
roslaunch arm_planner main_demo.launch mode:=fake

# 启用碰撞模型可视化
roslaunch arm_planner main_demo.launch mode:=fake show_collision:=true

# 实机模式 + 碰撞可视化
roslaunch arm_planner main_demo.launch mode:=real show_collision:=true

# 不带夹爪 + 碰撞可视化
roslaunch arm_planner main_demo.launch gripper:=false show_collision:=true
```

### 2. 独立启动可视化

```bash
# 方法1：使用launch文件
roslaunch mobile_manipulator2_description visualize_collision.launch

# 方法2：使用shell脚本
./scripts/visualize_collision_models.sh
```

## 可视化说明

在RViz中显示两个机器人模型：

1. **Visual Robot Model**（绿色半透明）
   - 显示机器人的实际外观
   - Alpha值设为0.8，便于观察内部结构
   - 用于显示和美观

2. **Collision Model**（红色/紫色线框）
   - 显示碰撞检测使用的简化模型
   - 线框模式，Alpha值设为0.5
   - 用于MoveIt的碰撞检测计算

3. **PlanningScene**（可选）
   - 显示MoveIt的实时碰撞检测状态
   - 碰撞部位会以红色高亮显示

## 参数说明

### main_demo.launch 参数
- `mode`: 运行模式，fake（仿真）或 real（实机）
- `gripper`: 是否带夹爪，true/false
- `show_collision`: 是否显示碰撞模型，true/false
- `use_rviz`: 是否启动RViz，true/false
- `debug`: 调试模式，true/false
- `pipeline`: 规划管线，默认ompl

## 注意事项

1. 碰撞模型应该略大于视觉模型，提供安全缓冲
2. 所有机械臂关节都使用了专门的碰撞STL文件
3. 底盘、升降杆等使用简单几何体（box/cylinder）
4. 使用Bullet碰撞检测引擎，提供更精确的检测

## 调整技巧

在RViz中可以：
1. 调整Visual Robot Model的Alpha值改变透明度
2. 开关Collision Model显示/隐藏碰撞模型
3. 使用joint_state_publisher_gui手动控制关节
4. 观察不同姿态下的碰撞模型覆盖情况

## 故障排除

如果碰撞模型不显示：
1. 检查robot_description参数是否正确加载
2. 确认URDF文件中collision标签正确定义
3. 验证collision STL文件路径正确
4. 检查RViz配置文件是否正确加载