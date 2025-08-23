# RViz中标红显示碰撞要素的配置指南

## 快速设置步骤

### 1. 在RViz中显示碰撞网格

在已经打开的RViz窗口中：

1. **找到左侧的Displays面板**
2. **展开 MotionPlanning → Scene Robot**
3. **勾选以下选项**：
   - ✅ Show Robot Collision（显示碰撞网格）
   - ❌ Show Robot Visual（取消显示视觉网格）
   - 设置 Robot Alpha = 0.8（轻微透明）

### 2. 高亮显示需要避免的碰撞物体

在 **Scene Robot → Links** 下，找到以下链接并修改颜色：

| 链接名称 | 设置方法 | 颜色 | 说明 |
|---------|---------|------|------|
| **lifting_link** | 右键 → Set Color | 红色 (255,0,0) | 升降立柱 - 最危险 |
| **lidar_link** | 右键 → Set Color | 红色 (255,0,0) | 激光雷达 - 易碰撞 |
| **box_link** | 右键 → Set Color | 深红 (200,0,0) | 电控箱 - 中等风险 |
| **base_link** | 右键 → Set Color | 暗红 (150,0,0) | 底盘 - 低风险 |

### 3. 设置碰撞时的显示颜色

1. **展开 MotionPlanning → Planning Request**
2. **设置 Colliding link Color = 红色 (255,0,0)**
3. 这样当机械臂接近碰撞时，相关部件会自动变红

### 4. 添加碰撞区域标记（可选）

如果想要更明显的碰撞区域显示：

1. **点击RViz左下角 "Add" 按钮**
2. **选择 "MarkerArray"**
3. **设置 Topic = /collision_visualization**
4. **运行碰撞标记脚本**：
   ```bash
   python ~/MobileManipulator/scripts/_cc_simple_collision_marker.py
   ```

## 效果说明

配置完成后，您将看到：

- 🔴 **红色物体**：需要避免碰撞的关键部件
- 🟡 **黄色/橙色**：机械臂接近碰撞时的警告色
- 🟢 **绿色**：安全的运动状态

## 测试碰撞检测

1. 在Motion Planning面板中，拖动机械臂的**橙色球体**（交互标记）
2. 当机械臂接近红色物体时：
   - 相关链接会变成红色
   - 规划器会自动避开这些区域
   - 如果强制移动到碰撞位置，规划会失败

## 保存配置

配置完成后，保存RViz配置：
- **File → Save Config As...**
- 保存为 `collision_display.rviz`

## 注意事项

- lifting_link（立柱）是最容易发生碰撞的部件
- 机械臂向前伸展时要特别注意避开立柱
- 向下运动时注意避开底座和激光雷达