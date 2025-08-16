# World TF问题完整解释

## 问题根源

### SRDF中的Virtual Joint定义
在`mobile_manipulator2_description.srdf`第119行：
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="world_link"/>
```

这个定义告诉MoveIt：
- 机器人通过一个虚拟关节连接到世界
- 父坐标系是"world"
- 子链接是"world_link"

### URDF中的定义
在URDF中，机器人的根链接是`world_link`：
```xml
<link name="world_link"/>
<joint name="world_to_base" type="fixed">
    <parent link="world_link"/>
    <child link="base_link"/>
</joint>
```

### 冲突
- MoveIt期望有一个"world"坐标系（因为SRDF中的virtual_joint）
- 但URDF只定义了"world_link"
- 导致TF树断开

## 解决方案

### 方案1：添加TF桥接（已实施）
在`arm_planner_demo.launch`中添加了TF桥接：
```xml
<!-- TF桥接：SRDF定义了world到world_link的virtual_joint -->
<include file="$(find arm_planner)/launch/world_tf_bridge.launch"/>
```

这会发布一个静态变换：`world_link -> world`

### 方案2：修改SRDF（备选）
可以修改SRDF中的virtual_joint：
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world_link" child_link="world_link"/>
```

但这不符合MoveIt的标准做法。

## 为什么需要Virtual Joint？

Virtual Joint在MoveIt中用于：
1. 定义机器人相对于世界的位置
2. 允许规划器考虑机器人的全局位置
3. 对于固定基座机器人，通常是fixed类型
4. 对于移动机器人，可能是floating或planar类型

## 标准做法

MoveIt的标准做法是：
- 使用"world"作为全局固定坐标系
- 机器人的根链接通过virtual_joint连接到"world"
- 这样可以保持与MoveIt生态系统的兼容性

## 总结

当前的解决方案通过TF桥接保持了兼容性：
- SRDF使用标准的"world"坐标系
- URDF保持使用"world_link"
- TF桥接连接两者

这样既不破坏现有配置，又解决了TF树断开的问题。