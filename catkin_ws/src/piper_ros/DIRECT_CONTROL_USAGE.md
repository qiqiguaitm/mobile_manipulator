# Piper机械臂直接MoveIt控制使用说明

## 概述

这个demo展示了如何不通过桥接的方式，直接使用MoveIt完成规划和实机控制。与桥接模式不同，这种方式实现了真正的MoveIt轨迹跟踪控制器，提供更好的轨迹执行精度和反馈。

## 系统架构

```
MoveIt规划器 → FollowJointTrajectory动作服务器 → piper_trajectory_controller → 机械臂硬件
     ↑                                                    ↓
状态反馈 ← joint_state_bridge ← /joint_states_single ← 状态反馈
```

### 关键组件

1. **piper_trajectory_controller.py**: 实现FollowJointTrajectory动作服务器
2. **real_robot_controllers.yaml**: 配置MoveIt控制器
3. **demo_direct.launch**: 直接控制模式的启动文件

## 使用方法

### 1. 启动系统

#### 方法1: 使用启动脚本（推荐）
```bash
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash launch_piper_direct.sh
```

#### 方法2: 手动启动
```bash
# 设置环境
source /opt/ros/noetic/setup.bash
source ~/AgileXDemo/catkin_ws/devel/setup.bash

# 激活CAN设备
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash can_activate.sh can0 1000000

# 启动直接控制模式
roslaunch piper_with_gripper_moveit demo_direct.launch \
    can_port:=can0 \
    auto_enable:=true \
    use_rviz:=false
```

### 2. 带RViz可视化启动
```bash
roslaunch piper_with_gripper_moveit demo_direct.launch \
    can_port:=can0 \
    auto_enable:=true \
    use_rviz:=true
```

### 3. 测试系统
```bash
# 运行完整测试
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
python3 test_direct_moveit.py
```

## 编程接口

### 使用MoveIt Python API

```python
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

# 初始化
moveit_commander.roscpp_initialize([])
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

# 关节运动控制
joint_goal = [0.1, 0.1, -0.1, 0.1, -0.1, 0.1]
arm_group.set_joint_value_target(joint_goal)
arm_group.go(wait=True)

# 笛卡尔运动控制
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.15
pose_goal.position.y = 0.0
pose_goal.position.z = 0.25
quat = quaternion_from_euler(0, 0, 0)
pose_goal.orientation.x = quat[0]
pose_goal.orientation.y = quat[1]
pose_goal.orientation.z = quat[2]
pose_goal.orientation.w = quat[3]

arm_group.set_pose_target(pose_goal)
arm_group.go(wait=True)

# 夹爪控制
gripper_group.set_joint_value_target([0.03])  # 张开
gripper_group.go(wait=True)
```

### 使用C++ API

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "piper_direct_control");
    ros::NodeHandle nh;
    
    // 创建move group
    moveit::planning_interface::MoveGroupInterface arm_group("arm");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    
    // 关节运动
    std::vector<double> joint_values = {0.1, 0.1, -0.1, 0.1, -0.1, 0.1};
    arm_group.setJointValueTarget(joint_values);
    arm_group.move();
    
    // 笛卡尔运动
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.15;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.25;
    target_pose.orientation.w = 1.0;
    
    arm_group.setPoseTarget(target_pose);
    arm_group.move();
    
    return 0;
}
```

## 高级功能

### 1. 轨迹规划参数调整
```python
# 设置规划时间
arm_group.set_planning_time(10)

# 设置速度和加速度缩放
arm_group.set_max_velocity_scaling_factor(0.5)
arm_group.set_max_acceleration_scaling_factor(0.3)

# 设置位置容差
arm_group.set_goal_position_tolerance(0.01)
arm_group.set_goal_orientation_tolerance(0.1)
```

### 2. 笛卡尔路径规划
```python
# 定义航点
waypoints = []

# 当前位姿
wpose = arm_group.get_current_pose().pose
waypoints.append(copy.deepcopy(wpose))

# 第一个航点
wpose.position.z += 0.1
waypoints.append(copy.deepcopy(wpose))

# 第二个航点
wpose.position.y += 0.1
waypoints.append(copy.deepcopy(wpose))

# 计算笛卡尔路径
(plan, fraction) = arm_group.compute_cartesian_path(
    waypoints,   # 航点
    0.01,        # 步长
    0.0)         # 跳跃阈值

# 执行路径
arm_group.execute(plan, wait=True)
```

### 3. 约束规划
```python
from moveit_msgs.msg import OrientationConstraint

# 创建朝向约束
constraints = moveit_msgs.msg.Constraints()
orientation_constraint = OrientationConstraint()
orientation_constraint.header.frame_id = arm_group.get_planning_frame()
orientation_constraint.link_name = arm_group.get_end_effector_link()
orientation_constraint.orientation.w = 1.0
orientation_constraint.absolute_x_axis_tolerance = 0.1
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
orientation_constraint.weight = 1.0

constraints.orientation_constraints.append(orientation_constraint)
arm_group.set_path_constraints(constraints)
```

## 与桥接模式的对比

| 特性 | 桥接模式 | 直接控制模式 |
|------|----------|--------------|
| 实现复杂度 | 简单 | 中等 |
| 轨迹精度 | 一般 | 高 |
| 实时反馈 | 有限 | 完整 |
| 错误处理 | 基础 | 高级 |
| MoveIt集成 | 有限 | 完整 |
| 轨迹平滑 | 无 | 有 |
| 碰撞检测 | 规划时 | 执行时 |

## 故障排除

### 1. 动作服务器连接失败
```bash
# 检查动作服务器状态
rostopic list | grep follow_joint_trajectory
rostopic echo /arm_controller/follow_joint_trajectory/status

# 检查轨迹控制器
rosnode list | grep trajectory_controller
rosnode info /piper_trajectory_controller
```

### 2. 轨迹执行失败
```bash
# 检查关节状态反馈
rostopic echo /joint_states_single -n 1

# 检查控制器配置
rosparam get /controller_list

# 查看详细日志
rosnode list | xargs -I {} rosnode info {}
```

### 3. 性能优化
- 调整轨迹执行超时参数
- 优化关节状态发布频率
- 调整MoveIt规划参数

## 注意事项

⚠️ **安全提醒**:
1. 直接控制模式下轨迹执行更精确，请确保工作空间安全
2. 系统会实时监控轨迹执行，异常时会自动停止
3. 建议在低速度下测试新的轨迹

🔧 **技术提醒**:
1. 确保CAN设备正确激活
2. 检查机械臂使能状态
3. 监控系统资源使用情况

## 扩展功能

此直接控制框架支持进一步扩展：
- 添加力控制接口
- 集成视觉反馈
- 实现自适应轨迹调整
- 添加安全监控机制