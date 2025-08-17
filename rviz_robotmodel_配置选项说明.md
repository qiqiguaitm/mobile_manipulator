# RViz RobotModel 显示配置选项说明

## 基础配置项

### 1. **主要显示控制**
- **Enabled**: 是否启用RobotModel显示
- **Robot Description**: 机器人描述参数名（默认为 "robot_desc"）
- **TF Prefix**: TF前缀（用于多机器人场景）
- **Update Interval**: 更新间隔（秒）

### 2. **视觉显示选项**
- **Visual Enabled**: 显示机器人的视觉模型（默认为true）
- **Collision Enabled**: 显示机器人的碰撞模型（默认为false）
- **Alpha**: 透明度设置（0-1，1为完全不透明）

### 3. **链接(Links)配置**
- **All Links Enabled**: 启用所有链接显示
- **Expand Joint Details**: 展开关节详细信息
- **Expand Link Details**: 展开链接详细信息
- **Expand Tree**: 展开树形结构
- **Link Tree Style**: 链接树样式（如 "Links in Alphabetic Order"）

### 4. **单个链接配置**
每个链接都可以单独配置：
- **Alpha**: 单个链接的透明度
- **Show Axes**: 显示坐标轴
- **Show Trail**: 显示轨迹
- **Value**: 是否显示该链接

## MoveIt特有的RobotModel配置

在MoveIt的MotionPlanning插件中，有额外的机器人显示选项：

### 1. **Scene Robot（场景机器人）**
- **Robot Alpha**: 场景中机器人的透明度
- **Show Scene Robot**: 是否显示场景机器人
- **Show Robot Visual**: 显示视觉模型
- **Show Robot Collision**: 显示碰撞模型
- **Attached Body Color**: 附加物体颜色

### 2. **Planned Path（规划路径）**
- **Robot Alpha**: 规划路径中机器人的透明度
- **Show Robot Visual**: 显示规划路径的视觉模型
- **Show Robot Collision**: 显示规划路径的碰撞模型
- **Show Trail**: 显示路径轨迹
- **Loop Animation**: 循环播放动画
- **State Display Time**: 状态显示时间

### 3. **Planning Request（规划请求）**
- **Start State Alpha**: 起始状态透明度
- **Start State Color**: 起始状态颜色
- **Goal State Alpha**: 目标状态透明度
- **Goal State Color**: 目标状态颜色
- **Colliding Link Color**: 碰撞链接颜色
- **Joint Violation Color**: 关节违规颜色

## 示例配置

### 基础RobotModel配置示例：
```yaml
- Class: rviz/RobotModel
  Enabled: true
  Name: RobotModel
  Robot Description: robot_desc
  Alpha: 1
  Collision Enabled: false
  Visual Enabled: true
  Links:
    All Links Enabled: true
    Link Tree Style: Links in Alphabetic Order
    base_link:
      Alpha: 1
      Show Axes: false
      Show Trail: false
      Value: true
```

### MoveIt场景配置示例：
```yaml
Scene Robot:
  Attached Body Color: 150; 50; 150
  Links:
    All Links Enabled: true
  Robot Alpha: 0.5
  Show Robot Collision: true
  Show Robot Visual: false
```

## 常用调整技巧

1. **调整透明度**：通过Alpha值可以让机器人半透明，便于查看内部结构
2. **显示碰撞模型**：开启Collision Enabled可以查看碰撞检测用的简化模型
3. **显示坐标轴**：为每个链接开启Show Axes可以帮助理解关节方向
4. **选择性显示**：可以单独关闭某些链接的显示，专注于特定部分

## 项目中的配置文件位置

- 基础机器人显示：`/src/robot_desc/mobile_manipulator2_description/rviz/display.rviz`
- MoveIt配置：`/src/arm_planner/moveit_config/piper_with_gripper/launch/moveit.rviz`
- 碰撞显示配置：`/src/arm_planner/moveit_config/piper_with_gripper/config/collision_display.rviz`
- SLAM配置：`/src/slam/rviz/mapping.rviz`