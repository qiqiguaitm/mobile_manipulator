# Piper机器人增强安全系统使用指南

## 概述

本文档介绍如何使用集成到 `main_demo.launch` 中的增强安全系统。该系统提供多层次的碰撞保护和运动控制优化。

## 快速开始

### 1. 使用快速启动脚本（推荐）

```bash
# 默认启动（实机模式 + 增强安全）
rosrun arm_planner launch_with_safety.sh

# 仿真模式 + 增强安全
rosrun arm_planner launch_with_safety.sh --fake

# 基础安全模式
rosrun arm_planner launch_with_safety.sh --basic-safety

# 查看所有选项
rosrun arm_planner launch_with_safety.sh --help
```

### 2. 使用roslaunch直接启动

```bash
# 增强安全系统（默认）
roslaunch arm_planner main_demo.launch

# 自定义安全配置
roslaunch arm_planner main_demo.launch \
    safety_system:=enhanced \
    enable_collision_monitoring:=true \
    enable_motion_smoothing:=true \
    safety_check_rate:=50

# 禁用安全系统
roslaunch arm_planner main_demo.launch safety_system:=none
```

## 安全系统级别

### 1. **增强安全 (enhanced)** - 默认推荐
- ✅ 连续碰撞检测 (50Hz)
- ✅ 加速度/加加速度限制
- ✅ 分级安全响应
- ✅ 智能速度调节
- ✅ 完整的安全日志

### 2. **基础安全 (basic)**
- ✅ 紧急停止功能
- ✅ 基本碰撞检测
- ❌ 无加速度限制
- ❌ 无分级响应

### 3. **无安全 (none)**
- ⚠️ 仅用于调试
- ❌ 所有安全功能禁用

## 安全功能详解

### 分级安全响应

| 安全等级 | 触发距离 | 速度限制 | 系统响应 |
|---------|---------|---------|---------|
| 正常 | >50mm | 100% | 正常运行 |
| 注意 | 30-50mm | 70% | 轻微减速 |
| 警告 | 20-30mm | 30% | 显著减速 |
| 危急 | 10-20mm | 10% | 极慢运动 |
| 紧急 | <10mm | 0% | 立即停止 |

### 运动平滑控制
- 最大加速度：默认限制为30%
- 加加速度限制：1.0 rad/s³
- 紧急减速度：2.0 rad/s²

## 监控和调试

### 1. 查看安全状态
```bash
# 监控安全等级
rostopic echo /safety_level

# 查看碰撞距离
rostopic echo /collision_distances

# 查看速度缩放因子
rostopic echo /speed_scale_factor
```

### 2. 可视化工具
```bash
# 启动时启用安全可视化
roslaunch arm_planner main_demo.launch enable_safety_visualization:=true

# 或使用快速脚本
./scripts/_cc_launch_with_safety.sh --visualize-safety
```

### 3. 紧急操作
```bash
# 紧急停止
rosservice call /emergency_stop

# 重置安全系统
rosservice call /reset_safety_system
```

## 配置文件位置

- 统一安全配置：`/src/arm_planner/moveit_config/piper_with_gripper/config/unified_safety_config.yaml`
- 碰撞填充配置：`/src/arm_planner/moveit_config/piper_with_gripper/config/collision_padding.yaml`
- 安全检查配置：`/src/arm_planner/moveit_config/piper_with_gripper/config/collision_safety_check.yaml`

## 性能优化建议

1. **CPU负载高**
   - 降低检测频率：`safety_check_rate:=30`
   - 禁用运动平滑：`enable_motion_smoothing:=false`

2. **响应延迟**
   - 使用基础安全模式：`safety_system:=basic`
   - 减少碰撞对数量

3. **误触发频繁**
   - 调整安全距离阈值（编辑unified_safety_config.yaml）
   - 增加防抖时间

## 故障排除

### 问题1：安全系统未启动
```bash
# 检查节点状态
rosnode list | grep safety

# 查看错误日志
rosnode info /safety_response_manager
```

### 问题2：机器人突然停止
```bash
# 检查当前安全等级
rostopic echo /safety_level -n 1

# 查看最小碰撞距离
rostopic echo /collision_distances -n 1

# 必要时重置系统
rosservice call /reset_safety_system
```

### 问题3：运动不平滑
```bash
# 检查加速度限制是否生效
rostopic echo /smooth_joint_command

# 调整平滑参数
rosparam set /motion_smoother/smoothing_factor 0.9
```

## 注意事项

1. **首次使用**：建议在仿真模式下测试安全系统
2. **参数调整**：修改安全参数前请充分测试
3. **紧急情况**：始终保持物理急停按钮可用
4. **定期检查**：查看安全日志确保系统正常

## 更新日志

- v1.0 (2024-01): 初始版本，集成基础安全功能
- v2.0 (2024-01): 添加增强安全系统，统一配置管理