# 200Hz控制频率下的安全系统调优指南

## 关键参数调整

### 1. 系统延迟分析（200Hz控制）

```
控制周期: 5ms (200Hz)
检测延迟: 5ms (200Hz检测)
计算延迟: 3ms (优化后)
通信延迟: 5ms (ROS消息)
执行延迟: 5ms (控制器响应)
总延迟: ~23ms (相比50Hz的60ms大幅降低)
```

### 2. 安全距离调整

基于200Hz控制，最坏情况下的运动距离：
- 一个检测周期: 5.6mm (1.11m/s × 5ms)
- 加上系统延迟(~23ms): 25.5mm
- 加上制动距离(~50ms): 55.5mm

因此调整为：
- **紧急停止**: 20mm (原30mm)
- **减速区**: 40mm (原60mm) 
- **警告区**: 60mm (原100mm)
- **规划缓冲**: 100mm (原150mm)

### 3. 控制参数优化

```yaml
# 200Hz控制下的参数
control_parameters:
  # 控制器更新频率
  controller_frequency: 200.0
  
  # 轨迹执行参数
  trajectory_execution:
    execution_duration_monitoring: false  # 关闭以减少延迟
    allowed_execution_duration_scaling: 1.2
    allowed_goal_duration_margin: 0.5
    
  # 控制器增益调整
  position_control:
    p_gain_scale: 1.2  # 略微提高P增益
    d_gain_scale: 0.8  # 降低D增益避免震荡
```

### 4. 性能优化建议

#### CPU负载管理
- 200Hz碰撞检测约占用单核30-40%
- 建议使用多核并行：`num_collision_check_threads: 4`
- 考虑使用实时内核补丁

#### 内存优化
- 预分配碰撞检测缓存
- 使用环形缓冲区存储历史数据
- 避免动态内存分配

### 5. 实施步骤

1. **更新控制器配置**
```bash
# controllers.yaml
arm_controller:
  type: "position_controllers/JointTrajectoryController"
  joints: [joint1, joint2, joint3, joint4, joint5, joint6]
  constraints:
    goal_time: 0.1
    stopped_velocity_tolerance: 0.05
  state_publish_rate: 200.0  # 匹配控制频率
```

2. **调整MoveIt配置**
```bash
# trajectory_execution.launch.xml
<param name="trajectory_execution/execution_duration_monitoring" value="false"/>
<param name="trajectory_execution/controller_connection_timeout" value="0.5"/>
```

3. **启动时参数**
```bash
roslaunch arm_planner main_demo.launch \
  safety_check_rate:=200 \
  enable_motion_smoothing:=true
```

### 6. 验证检查清单

- [ ] 控制器实际运行在200Hz
- [ ] 碰撞检测延迟 < 5ms
- [ ] 紧急停止响应时间 < 30ms
- [ ] CPU使用率 < 60%
- [ ] 无丢失控制周期

### 7. 故障排除

**问题1**: 控制震荡
- 降低D增益
- 增加滤波器带宽

**问题2**: CPU过载
- 降低碰撞检测频率到100Hz
- 减少插值步数

**问题3**: 通信延迟
- 使用实时ROS传输
- 调整QoS设置