# World坐标系统一化

## 修改总结

### 之前的问题
- URDF使用`world_link`作为根坐标系
- SRDF定义了`world`到`world_link`的虚拟关节
- 需要TF桥接来连接两个坐标系
- 导致TF树断开的警告

### 现在的解决方案
统一使用`world`作为根坐标系：

1. **URDF修改**
   ```xml
   <!-- 之前 -->
   <link name="world_link"/>
   <parent link="world_link"/>
   
   <!-- 现在 -->
   <link name="world"/>
   <parent link="world"/>
   ```

2. **SRDF修改**
   ```xml
   <!-- 之前 -->
   <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="world_link"/>
   
   <!-- 现在 -->
   <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
   ```

3. **删除了不必要的文件**
   - `world_tf_bridge.launch` - 不再需要
   - 相关的TF桥接代码

## 优势

1. **简化系统**：不需要额外的TF桥接
2. **符合标准**：MoveIt和大多数ROS包默认使用`world`
3. **消除警告**：不会再出现TF树断开的警告
4. **提高性能**：减少了一个不必要的TF查找

## TF树结构

现在的TF树结构更简洁：
```
world (固定坐标系)
  └── base_link (通过fixed joint连接)
      ├── box_Link
      │   ├── lifting_Link
      │   └── arm_base
      │       └── link1
      │           └── link2
      │               └── ... (机械臂链)
      ├── lidar_Link
      └── under_camera_Link
```

## 注意事项

如果有其他包或脚本硬编码使用`world_link`，需要更新为`world`。

## 验证

重启系统后，运行以下命令验证：
```bash
# 查看TF树
rosrun tf2_tools view_frames.py

# 检查world到base_link的变换
rosrun tf tf_echo world base_link
```

应该能看到正常的变换，没有任何警告。