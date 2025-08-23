# RGBD深度投影功能使用说明

## 功能概述

此功能将三个相机(hand,chassis,top)的color图像和深度图关联后，投影到机器人坐标系下3D空间，通过RViz可视化彩色点云。

## 核心特性

- **统一处理**: 一套代码处理所有相机，消除特殊情况
- **RGBD关联**: 深度图像与颜色图像精确同步
- **坐标变换**: 自动投影到机器人base_link坐标系
- **灵活配置**: 支持单相机或多相机模式
- **实时可视化**: RViz中显示彩色点云

## 快速开始

### 1. 测试单个相机(推荐)

```bash
# 测试手部相机RGBD投影
./scripts/test_depth_projection.sh hand color

# 测试顶部相机纯深度投影  
./scripts/test_depth_projection.sh top depth

# 测试底盘相机RGBD投影
./scripts/test_depth_projection.sh chassis color
```

### 2. 测试全部相机同步

```bash
# 三相机同步RGBD投影
./scripts/test_depth_projection.sh all color

# 三相机同步深度投影
./scripts/test_depth_projection.sh all depth
```

### 3. 手动启动

```bash
# 1. 启动相机驱动
roslaunch camera_driver camera_driver.launch enable_hand_camera:=true

# 2. 启动RGBD投影
roslaunch perception depth_projection.launch enable_color:=true enable_hand_camera:=true

# 3. 查看结果
rviz -d src/perception/config/depth_projection_visualization.rviz
```

## 输出话题

### 点云话题
- `/projected_cloud/chassis_camera` - 底盘相机点云
- `/projected_cloud/top_camera` - 顶部相机点云
- `/projected_cloud/hand_camera` - 手部相机点云
- `/projected_cloud/combined` - 合并点云 (多相机模式)

### 输入话题
- 深度: `/camera/{camera}/depth/image_rect_raw`
- 颜色: `/camera/{camera}/color/image_rect_color`

## 参数配置

### Launch文件参数

```xml
<!-- 相机选择 -->
<arg name="enable_chassis_camera" default="true"/>
<arg name="enable_top_camera" default="true"/>  
<arg name="enable_hand_camera" default="false"/>

<!-- 功能开关 -->
<arg name="enable_color" default="true"/>
<arg name="publish_combined" default="true"/>

<!-- 可视化 -->
<arg name="use_rviz" default="true"/>
<arg name="load_robot_description" default="true"/>
```

### 深度缩放参数
```xml
<arg name="chassis_depth_scale" default="1000.0"/>
<arg name="top_depth_scale" default="1000.0"/>
<arg name="hand_depth_scale" default="1000.0"/>
```

## 故障排查

### 1. 无点云输出
```bash
# 检查相机话题
rostopic list | grep camera

# 检查深度数据
rostopic hz /camera/hand/depth/image_rect_raw

# 检查TF连接
rosrun tf tf_echo base_link camera/hand_depth_optical_frame
```

### 2. 点云位置错误
```bash
# 查看TF树
rosrun tf view_frames
firefox frames.pdf

# 检查相机标定
rostopic echo /camera/hand/depth/camera_info
```

### 3. 颜色显示异常
```bash
# 检查颜色数据
rostopic hz /camera/hand/color/image_rect_color

# 查看同步状态
rostopic echo /projected_cloud/hand_camera --noarr | head -20
```

## 技术细节

### 数据流程
```
相机RGB图像 + 深度图像 
    ↓ (message_filters同步)
相机坐标系RGBD点云
    ↓ (TF变换到base_link)
机器人坐标系彩色点云
    ↓ (RViz可视化)
3D彩色点云显示
```

### 坐标系链
```
base_link → camera/{camera}_link → camera/{camera}_depth_optical_frame
```

### 点云格式
- **位置**: (x, y, z) 机器人坐标系坐标(米)
- **颜色**: RGB打包为uint32格式
- **帧ID**: base_link

## 性能优化

### 1. 降低计算负载
```bash
# 只启用需要的相机
roslaunch perception depth_projection.launch enable_hand_camera:=true enable_top_camera:=false

# 禁用颜色处理
roslaunch perception depth_projection.launch enable_color:=false
```

### 2. 调整同步参数
```python
# 在depth_projection_node.py中调整
sync = message_filters.ApproximateTimeSynchronizer(
    [depth_sub, color_sub], queue_size=3, slop=0.05)  # 减小队列和时间容差
```

## 开发说明

### 核心文件
- `src/perception/scripts/depth_projector_core.py` - 投影核心逻辑
- `src/perception/scripts/depth_projection_node.py` - ROS节点
- `src/perception/launch/depth_projection.launch` - 启动文件
- `src/perception/config/depth_projection_visualization.rviz` - 可视化配置

### 设计原则
1. **消除特殊情况** - 统一处理所有相机
2. **数据结构优先** - 基于现有优雅设计扩展  
3. **向后兼容** - 不破坏现有depth-only功能
4. **实用主义** - 解决真实感知问题

基于Linus Torvalds的"好品味"编程哲学设计。