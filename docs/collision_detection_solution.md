# 碰撞检测问题解决方案

## 问题描述
在RViz中设置机械臂目标到lifting_Link后面时，机械臂仍然能够规划成功，没有正确避撞。

## 问题分析

### 1. 坐标系问题
- lifting_Link在world_link坐标系中的位置是 x=-0.243
- 这实际上表示lifting_Link在机器人**前方**（不是后方）
- 机器人的正前方是+X方向

### 2. 已确认的状态
- ✓ SRDF文件已正确修改（注释掉了碰撞禁用规则）
- ✓ ACM（允许碰撞矩阵）显示碰撞检测已启用
- ✓ 编程测试显示碰撞检测工作正常
- ✗ 但在RViz交互式规划时仍有问题

## 解决方案

### 方案1：重启整个系统（推荐）
```bash
# 1. 完全停止所有ROS进程
rosnode kill -a
pkill -9 roscore
pkill -9 rosmaster

# 2. 重新启动
roscore &
sleep 3
roslaunch arm_planner arm_planner_demo.launch
```

### 方案2：手动添加碰撞对象
如果重启后问题仍存在，运行以下脚本添加额外的碰撞对象：
```bash
python /home/agilex/MobileManipulator/scripts/_cc_add_collision_objects.py
```

### 方案3：检查碰撞几何体尺寸
lifting_Link的碰撞几何体可能太小（6cm x 40cm x 6cm）。可以：
1. 修改URDF中的碰撞网格
2. 或使用更大的简化碰撞盒

## 验证步骤

### 1. 运行测试脚本
```bash
python /home/agilex/MobileManipulator/scripts/_cc_test_lifting_collision.py
```
应该看到"规划失败 - 碰撞检测正常!"

### 2. 在RViz中测试
1. 打开Motion Planning插件
2. 在Scene Robot中勾选"Show Robot Collision"
3. 拖动橙色交互标记到lifting_Link位置
4. 应该看到：
   - 相关链接变红（表示碰撞）
   - 无法规划路径

### 3. 查看碰撞可视化
```bash
# 运行碰撞标记脚本
python /home/agilex/MobileManipulator/scripts/_cc_simple_collision_marker.py
```
在RViz中添加MarkerArray显示，话题：/collision_visualization

## 常见问题

### Q: 为什么编程测试正常但RViz交互不正常？
A: 可能原因：
1. RViz的Motion Planning插件可能使用了不同的规划参数
2. 交互式标记可能绕过了某些碰撞检查
3. 可能需要调整碰撞检测的容差参数

### Q: lifting_Link到底在前面还是后面？
A: lifting_Link在机器人坐标系的**前方**：
- world_link中x=-0.243表示在世界坐标系的负X方向
- 但相对于机器人base_link，它在正前方
- 机器人面向+X方向

### Q: 如何确保碰撞检测一定工作？
A: 最可靠的方法：
1. 使用更大的碰撞几何体
2. 添加额外的碰撞对象作为安全边界
3. 在代码中进行二次碰撞检查

## 最终建议

1. **先重启整个系统**，确保所有配置重新加载
2. **验证碰撞检测**使用提供的测试脚本
3. 如果问题持续，考虑**增大碰撞几何体尺寸**
4. 作为最后手段，可以**手动添加碰撞对象**到场景中