# MoveIt配置更改记录

## 更改日期：2025-08-16

### 1. 移除chassis和sensors组

**原因**：move_group启动时出现以下警告信息：
```
[INFO] [1755307347.115222875]: No active joints or end effectors found for group 'chassis'. Make sure that kinematics.yaml is loaded in this node's namespace.
[INFO] [1755307347.117029469]: No active joints or end effectors found for group 'sensors'. Make sure that kinematics.yaml is loaded in this node's namespace.
```

**修改内容**：
- 文件：`/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/config/mobile_manipulator2_description.srdf`
- 注释掉了chassis组（第23-27行）
- 注释掉了sensors组（第30-34行）

**说明**：
- chassis组只包含固定链接（base_link, box_Link, lifting_Link），不需要运动学求解器
- sensors组只包含传感器链接（lidar_Link, top_camera_Link, under_camera_Link），也不需要运动学求解器
- 这些组主要用于碰撞检测，即使注释掉也不影响碰撞检测功能

### 2. 修复碰撞检测问题

**原因**：机械臂不会避开lifting_Link、lidar_Link等物体

**修改内容**：
- 同一文件中，注释掉了过度的碰撞禁用规则：
  - lifting_Link与link4-link8的碰撞禁用
  - lidar_Link与link3-link8的碰撞禁用
  - box_Link与link4-link6的碰撞禁用

**效果**：
- 机械臂现在会正确检测并避开这些物体
- 在RViz中可以看到碰撞检测正常工作

### 3. 使更改生效

需要重启MoveIt才能加载新的配置：
```bash
# 1. 停止当前的MoveIt进程（Ctrl+C）
# 2. 重新启动
roslaunch arm_planner arm_planner_demo.launch
```

### 4. 验证工具

创建了以下脚本来验证更改：
- `/scripts/_cc_verify_groups_removed.py` - 验证chassis和sensors组是否已移除
- `/scripts/_cc_verify_collision_fix.py` - 验证碰撞检测是否正常工作
- `/scripts/_cc_diagnose_collision.py` - 诊断碰撞检测系统

### 5. 碰撞可视化

运行以下脚本可在RViz中显示红色碰撞区域：
```bash
python /home/agilex/MobileManipulator/scripts/_cc_simple_collision_marker.py
```

在RViz中添加MarkerArray显示，话题设置为`/collision_visualization`