# Piper机械臂本机操作使用说明

## 1. 准备工作

### 1.1 硬件连接
1. 确保机械臂已上电
2. 将USB转CAN模块连接到机械臂和计算机
3. 检查绿色端子连接是否牢固

### 1.2 权限设置
运行权限设置脚本（只需运行一次）：
```bash
sudo bash /home/agilex/AgileXDemo/catkin_ws/src/piper_ros/setup_can_permissions.sh
```

然后重新登录或运行：
```bash
newgrp can
```

## 2. 启动方式

### 2.1 使用脚本启动（推荐）
```bash
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash launch_piper_real.sh
```

### 2.2 手动启动
```bash
# 1. 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/AgileXDemo/catkin_ws/devel/setup.bash

# 2. 激活CAN设备
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash can_activate.sh can0 1000000

# 3. 启动MoveIt（真实硬件模式）
roslaunch piper_with_gripper_moveit demo.launch \
    moveit_controller_manager:=fake \
    real_hardware:=true \
    use_rviz:=false \
    can_port:=can0 \
    auto_enable:=true
```

### 2.3 启动带RViz可视化
```bash
roslaunch piper_with_gripper_moveit demo.launch \
    moveit_controller_manager:=fake \
    real_hardware:=true \
    use_rviz:=true \
    can_port:=can0 \
    auto_enable:=true
```

## 3. 参数说明

- `moveit_controller_manager`: 控制器管理器（推荐使用fake）
  - `fake`: 使用fake控制器（适合与真实硬件配合）
  - `simple`: 需要实际的follow_joint_trajectory服务器
- `real_hardware`: 是否连接真实硬件（默认false）
  - `true`: 启动真实机械臂控制节点和桥接器
  - `false`: 纯仿真模式
- `can_port`: CAN设备名称（默认can0）
- `auto_enable`: 是否自动使能机械臂（默认true）
- `use_rviz`: 是否启动RViz可视化（默认true）

## 4. 故障排除

### 4.1 CAN设备问题
```bash
# 检查CAN设备状态
ip link show type can

# 查看CAN设备详细信息
ip -details link show can0

# 如果设备未激活，手动激活
sudo ip link set can0 up type can bitrate 1000000
```

### 4.2 权限问题
```bash
# 检查当前用户是否在can用户组中
groups

# 如果不在can组中，运行
sudo usermod -a -G can $USER
newgrp can
```

### 4.3 机械臂连接问题
- 检查USB转CAN模块是否正确连接
- 检查机械臂是否上电
- 检查绿色端子连接是否牢固
- 尝试重新插拔USB转CAN模块

### 4.4 MoveIt服务不可用问题
如果遇到`Service [/joint_moveit_ctrl_endpose] is not available`错误：

```bash
# 方法1: 重启完整系统
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
bash test_complete_system.sh

# 方法2: 单独启动服务（在demo.launch运行的情况下）
bash start_moveit_services.sh

# 方法3: 检查服务状态
rosservice list | grep joint_moveit
rosnode list | grep joint_moveit
```

### 4.5 关节名称错误问题
如果遇到`Joint 'gripper' not found in model`或`The complete state of the robot is not yet known`错误：

```bash
# 测试关节名称修复
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
python3 test_joint_names_fix.py

# 如果问题持续，重启系统以应用修复
bash test_complete_system.sh
```

### 4.6 机械臂不运动问题
如果MoveIt服务调用成功但机械臂不运动：

```bash
# 测试命令传递流程
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
python3 test_command_flow.py

# 测试直接控制
python3 test_direct_control.py

# 检查关键话题
rostopic list | grep -E "(joint_ctrl|fake_controller)"
rostopic echo /joint_ctrl_commands -n 1
```

### 4.7 IndexError问题
如果遇到`tuple index out of range`或类似的索引错误：

```bash
# 测试IndexError修复
cd /home/agilex/AgileXDemo/catkin_ws/src/piper_ros
python3 test_index_error_fix.py

# 检查日志中是否还有IndexError
rosnode list | xargs -I {} rosnode info {} | grep -E "(Publications|Subscriptions)"
```

## 5. 控制接口

启动后可用的ROS服务：
```bash
# 机械臂使能/失能
rosservice call /enable_srv "enable_request: true"
rosservice call /enable_srv "enable_request: false"

# 机械臂归零
rosservice call /go_zero_srv "is_mit_mode: false"

# 机械臂停止
rosservice call /stop_srv

# MoveIt控制服务
rosservice call /joint_moveit_ctrl_arm
rosservice call /joint_moveit_ctrl_gripper
rosservice call /joint_moveit_ctrl_piper
rosservice call /joint_moveit_ctrl_endpose
```

## 6. 安全注意事项

⚠️ **警告**：
- 启动前确保机械臂工作范围内无障碍物
- 机械臂会在使能后抬起，请保持安全距离
- 停止服务会让机械臂以恒定阻尼落下
- 重置服务会让机械臂立刻掉电落下

## 7. 工作原理

当使用`real_hardware:=true`时，系统工作流程如下：

1. **MoveIt规划器**：使用fake控制器进行路径规划和轨迹执行
2. **硬件控制**：`piper_ctrl_single_node.py`通过CAN总线控制真实机械臂
3. **状态桥接**：`fake_to_real_bridge.py`将真实机械臂状态发布到`/joint_states`
4. **状态反馈**：MoveIt接收真实机械臂状态，用于轨迹验证和执行监控

### 修复的问题：
- ✅ 修复了时间戳同步问题（"Didn't receive robot state with recent timestamp"）
- ✅ 修复了URDF虚拟关节配置警告（"Skipping virtual joint"）
- ✅ 修复了关节名称不匹配问题（"Joint 'gripper' not found in model"）
- ✅ 修复了IndexError问题（"tuple index out of range"）
- ✅ 改进了状态反馈机制，确保MoveIt能获取到实时的机械臂状态
- ✅ 消除了"CONTROL_FAILED"错误
- ✅ 解决了"The complete state of the robot is not yet known"警告
- ✅ 实现了MoveIt命令到真实机械臂的完整传递链路

这种设计避免了复杂的控制器配置，同时保持了MoveIt的完整功能。

## 8. 开发接口

### 8.1 使用MoveIt Python API
```python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

# 初始化MoveIt
moveit_commander.roscpp_initialize([])
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")

# 设置目标位置
target_pose = Pose()
target_pose.position.x = 0.3
target_pose.position.y = 0.0
target_pose.position.z = 0.3
target_pose.orientation.w = 1.0

arm_group.set_pose_target(target_pose)
arm_group.go(wait=True)
```

### 8.2 直接控制关节
```python
import rospy
from sensor_msgs.msg import JointState

# 发布关节角度
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

joint_msg = JointState()
joint_msg.position = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.01]
joint_msg.velocity = [0.0] * 7
joint_msg.effort = [0.0] * 7

joint_pub.publish(joint_msg)
```