# 在RViz中标红显示需要避免碰撞的要素

## 方法1：使用Planning Scene显示（推荐）

在RViz的Motion Planning插件中：

1. **展开Scene Robot部分**
   - 勾选 "Show Robot Collision" 
   - 将 "Robot Alpha" 设置为 0.5（半透明）
   
2. **在Scene Robot > Links中**，找到以下链接并设置颜色：
   - **lifting_Link**: 
     - 右键点击 → Set Color → 选择红色 (255, 0, 0)
   - **lidar_Link**: 
     - 右键点击 → Set Color → 选择红色 (255, 0, 0)
   - **box_Link**: 
     - 右键点击 → Set Color → 选择深红色 (200, 0, 0)
   - **base_link**: 
     - 右键点击 → Set Color → 选择暗红色 (150, 0, 0)

## 方法2：使用碰撞网格显示

1. **在Displays面板中**
   - 展开 MotionPlanning → Planning Request
   - 设置 "Colliding Link Color" 为红色 (255, 0, 0)
   
2. **启用碰撞检测显示**
   - 在 Scene Robot 中勾选 "Show Robot Collision"
   - 取消勾选 "Show Robot Visual"（只显示碰撞网格）

## 方法3：使用MarkerArray（已配置）

1. **添加MarkerArray显示**
   - 点击RViz左下角的 "Add" 按钮
   - 选择 "MarkerArray"
   - 设置Topic为: `/collision_objects_markers`
   
2. **运行可视化脚本**
   ```bash
   rosrun arm_planner collision_visualizer.py
   ```

## 关键碰撞物体说明

### 🔴 需要避免的主要碰撞物体：

1. **lifting_Link（升降立柱）** - 最高优先级
   - 位置：机器人前方
   - 碰撞风险：机械臂向前运动时
   
2. **lidar_Link（激光雷达）** - 高优先级  
   - 位置：底座前方
   - 碰撞风险：机械臂向下和向后运动时
   
3. **box_Link（电控箱）** - 中等优先级
   - 位置：机械臂基座下方
   - 碰撞风险：机械臂大幅度向下运动时
   
4. **base_link（底盘）** - 低优先级
   - 位置：最底部
   - 碰撞风险：极端姿态时

## 测试碰撞检测

在Motion Planning中：
1. 拖动机械臂的交互标记
2. 当接近碰撞物体时，相关链接会自动变红
3. 规划路径时会自动避开这些红色区域