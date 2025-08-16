# TF World坐标系问题修复

## 问题描述
```
[WARN] [1755312978.622363333]: Unable to transform object from frame 'world' to planning frame 'world_link' 
(Could not find a connection between 'world_link' and 'world' because they are not part of the same tree.
Tf has two or more unconnected trees.)
```

## 问题原因
`collision_scene_manager.py`使用了`world`坐标系，而机器人URDF定义的根坐标系是`world_link`，导致TF树断开。

## 解决方案

### 已完成的修复
1. 修改了`/home/agilex/MobileManipulator/src/arm_planner/scripts/collision_scene_manager.py`
   - 将`frame_id = "world"`改为`frame_id = "world_link"`
   - 影响了地面(ground)和墙壁(wall)的碰撞对象

2. 重启了相关节点
   ```bash
   rosnode kill /ground_collision_manager
   ```

### 验证修复
```bash
# 检查TF树是否连通
rosrun tf tf_echo world_link base_link

# 应该能看到正常的变换输出，而不是错误信息
```

## 临时解决方案（如果还有其他节点使用world）

可以运行TF桥接脚本：
```bash
python /home/agilex/MobileManipulator/scripts/_cc_fix_world_tf.py
```

这将发布`world`到`world_link`的恒等变换，连接两个坐标系。

## 建议

1. **统一坐标系命名**
   - 整个项目应该统一使用`world_link`作为固定坐标系
   - 或者统一使用`world`（需要修改URDF）

2. **检查其他可能的问题源**
   - perception包
   - slam包
   - 其他第三方包

3. **在启动文件中添加静态变换**（可选）
   ```xml
   <node pkg="tf2_ros" type="static_transform_publisher" 
         name="world_to_world_link" 
         args="0 0 0 0 0 0 world world_link"/>
   ```

## 总结
TF警告已通过修改collision_scene_manager.py解决。如果还有其他节点使用"world"坐标系，可以使用提供的临时解决方案。