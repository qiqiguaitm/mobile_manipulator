# 机械臂关节参数更新总结

## 更新日期
2025-08-15

## 更新内容

根据机械臂的实际物理参数，更新了所有配置文件中的关节运动范围和最大速度限制。

### 1. 关节运动范围更新

| 关节 | 原始范围 | 新范围（角度） | 新范围（弧度） |
|------|----------|----------------|----------------|
| J1 | -150°~124° | ±154° | ±2.687 rad |
| J2 | 0°~180° | 0°~195° | 0~3.403 rad |
| J3 | -170°~0° | -175°~0° | -3.054~0 rad |
| J4 | -100°~100° | -102°~102° | -1.780~1.780 rad |
| J5 | -70°~70° | -75°~75° | -1.309~1.309 rad |
| J6 | ±120° | ±120° | ±2.094 rad |

### 2. 关节最大速度更新

| 关节 | 原始速度 | 新速度（角度/秒） | 新速度（弧度/秒） |
|------|----------|-------------------|-------------------|
| J1 | 5 rad/s | 180°/s | 3.142 rad/s |
| J2 | 5 rad/s | 195°/s | 3.403 rad/s |
| J3 | 5 rad/s | 180°/s | 3.142 rad/s |
| J4 | 5 rad/s | 225°/s | 3.927 rad/s |
| J5 | 5 rad/s | 225°/s | 3.927 rad/s |
| J6 | 3 rad/s | 225°/s | 3.927 rad/s |

### 3. 更新的文件列表

#### URDF文件
- `/src/robot_description/piper_description/urdf/piper_description.urdf`
- `/src/robot_description/piper_description/urdf/piper_no_gripper_description.urdf`

#### Xacro文件
- `/src/robot_description/piper_description/urdf/piper_description.xacro`
- `/src/robot_description/piper_description/urdf/piper_no_gripper_description.xacro`

#### MoveIt配置文件
- `/src/arm_planner/moveit_config/piper_with_gripper/config/joint_limits.yaml`
- `/src/arm_planner/moveit_config/piper_no_gripper/config/joint_limits.yaml`

### 4. 验证方法

创建了验证脚本 `_cc_verify_joint_limits.py`，可以通过以下步骤验证配置：

```bash
# 1. 启动机械臂（加载URDF到参数服务器）
roslaunch arm_controller start_single_piper.launch

# 2. 在新终端运行验证脚本
rosrun arm_controller _cc_verify_joint_limits.py
```

### 5. 注意事项

1. **一致性**: 所有配置文件（URDF、Xacro、MoveIt）中的关节限制参数已保持一致。

2. **安全边界**: 配置的限制值与实际硬件规格完全匹配，确保机械臂在安全范围内运动。

3. **速度缩放**: MoveIt配置中保留了默认的速度缩放因子：
   - 带夹爪版本: 0.5（50%速度）
   - 无夹爪版本: 0.1（10%速度）
   
   这些缩放因子可以在实际应用中根据需要调整。

4. **重新编译**: 修改配置文件后，需要重新编译工作空间：
   ```bash
   catkin build
   ```

### 6. 影响范围

这些更新将影响：
- 运动规划的可达空间
- 轨迹规划的速度约束
- 碰撞检测的关节限制
- 仿真环境中的机械臂行为

建议在实际使用前，在仿真环境中充分测试新的配置参数。