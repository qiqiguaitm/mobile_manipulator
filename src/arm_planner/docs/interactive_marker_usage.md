# RViz Interactive Marker 使用指南

## 启用Interactive Marker

MoveIt的MotionPlanning插件已经内置了Interactive Marker功能，无需额外配置。

### 使用步骤

1. **启动机器人和MoveIt**
   ```bash
   roslaunch arm_planner demo.launch
   ```

2. **在RViz中启用Interactive Marker**
   - 在左侧的`Displays`面板中找到`MotionPlanning`
   - 展开`Planning Request`
   - 确保以下选项已勾选：
     - ✓ `Query Goal State` - 允许拖动目标位置
     - ✓ `Query Start State` - 允许设置起始位置（可选）
   - 设置`Interactive Marker Size`（默认0.1，可根据需要调整）

3. **使用Interactive Marker**
   - 在3D视图中，你会看到机械臂末端有一个彩色的交互标记
   - **平移**：拖动箭头移动末端位置
   - **旋转**：拖动圆环旋转末端姿态
   - **规划**：在MotionPlanning面板中点击`Plan`按钮
   - **执行**：点击`Execute`按钮执行规划的路径

### 快捷操作

1. **切换控制模式**
   - 在`Planning Group`下拉菜单中选择：
     - `arm` - 控制整个机械臂（含夹爪）
     - `piper` - 仅控制6轴机械臂
     - `gripper` - 仅控制夹爪

2. **使用预设位置**
   - 在`Goal State`下拉菜单中选择：
     - `zero` - 所有关节归零
     - `home` - 垂直姿态
     - `ready` - 准备姿态

### 注意事项

- Interactive Marker的颜色表示状态：
  - **橙色**：目标位置
  - **绿色**：当前/起始位置
  - **红色**：碰撞或不可达

- 如果看不到Interactive Marker：
  1. 检查`Query Goal State`是否勾选
  2. 检查`Planning Group`是否正确选择
  3. 尝试调整`Interactive Marker Size`

### 高级功能

- **笛卡尔路径规划**：勾选`Use Cartesian Path`可以让末端沿直线运动
- **避障规划**：场景中的障碍物会自动被考虑
- **约束规划**：可以添加位置或姿态约束