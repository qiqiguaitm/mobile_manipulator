# 统一感知系统使用指南

## 🚀 快速启动

### 基本语法
```bash
./start_unified_perception.sh [投影方法] [相机配置] [API配置] [调试模式]
```

### 参数说明

#### 1. 投影方法 (projection_method)
- `urdf`: 使用RealSense内参 + URDF TF链 ✅ **推荐**
- `calibration`: 使用标定内参 + 标定TF链

#### 2. 相机配置 (camera_config)  
- `hand`: 只启用手部相机 ✅ **默认**
- `chassis`: 只启用底盘相机
- `top`: 只启用顶部相机
- `all`: 启用所有相机
- `none`: 无相机模式（仅算法测试）

#### 3. API配置 (api_config)
- `grasp`: 只启用抓取检测 ✅ **默认**
- `refer`: 只启用Referring检测
- `all`: 启用所有检测API

#### 4. 调试模式 (debug_mode)
- `true`: 启用详细日志和调试信息 ✅ **默认**
- `false`: 简化日志输出

## 📋 使用示例

### 常用组合

#### 1. 默认配置（推荐新手）
```bash
# 手部相机 + 抓取检测 + URDF投影
./start_unified_perception.sh
# 等同于: ./start_unified_perception.sh urdf hand grasp true
```

#### 2. 完整功能测试
```bash  
# 所有相机 + 所有API + URDF投影
./start_unified_perception.sh urdf all all true
```

#### 3. 无相机算法测试
```bash
# 无相机模式，仅测试算法逻辑
./start_unified_perception.sh urdf none grasp true
```

#### 4. 高精度抓取（标定方法）
```bash
# 手部相机 + 抓取检测 + 标定投影
./start_unified_perception.sh calibration hand grasp false
```

#### 5. 多相机抓取
```bash
# 手部+底盘相机 + 抓取检测
./start_unified_perception.sh urdf all grasp true
```

#### 6. Referring检测测试
```bash
# 手部相机 + Referring检测
./start_unified_perception.sh urdf hand refer true
```

### 专业用途

#### 性能测试
```bash
# 最简配置，最小延迟
./start_unified_perception.sh urdf hand grasp false
```

#### 开发调试
```bash  
# 全功能 + 详细日志
./start_unified_perception.sh urdf all all true
```

#### 生产部署
```bash
# 稳定的URDF方法 + 简化日志
./start_unified_perception.sh urdf hand grasp false
```

## 🔧 输出话题说明

### 根据配置自动生成的话题

#### 抓取检测话题
```text
手部相机:
- /perception/hand/grasps          # 2D抓取检测
- /perception/hand/grasps_3d       # 3D抓取检测  
- /perception/hand/grasp_points    # 抓取点云
- /perception/hand/vis_image       # 可视化图像

底盘相机:
- /perception/chassis/grasps       # 2D抓取检测
- /perception/chassis/grasps_3d    # 3D抓取检测
- /perception/chassis/grasp_points # 抓取点云

顶部相机:
- /perception/top/grasps           # 2D抓取检测  
- /perception/top/grasps_3d        # 3D抓取检测
- /perception/top/grasp_points     # 抓取点云
```

#### Referring检测话题
```text
手部相机:
- /perception/hand/referring       # 2D Referring检测
- /perception/hand/referring_3d    # 3D Referring检测
- /perception/hand/referring_points # Referring点云

底盘相机:
- /perception/chassis/referring    # 2D Referring检测
- /perception/chassis/referring_3d # 3D Referring检测

顶部相机:  
- /perception/top/referring        # 2D Referring检测
- /perception/top/referring_3d     # 3D Referring检测
```

## 🛠️ 服务接口

### 系统控制服务
```bash
# 重启整个感知系统
rosservice call /unified_perception/restart

# 重新加载配置文件
rosservice call /unified_perception/reload_config
```

### 状态监控话题
```bash
# 查看系统状态
rostopic echo /unified_perception/status
```

## ⚙️ 配置文件

主配置文件位于：`src/perception/config/perception_unified.yaml`

可以手动编辑此文件来：
- 调整检测参数（阈值、频率等）
- 修改过滤器设置（深度范围、高度范围等）
- 启用/禁用特定功能
- 配置话题名称

修改配置后，使用重启服务生效：
```bash
rosservice call /unified_perception/restart
```

## 🔍 故障排查

### 常见问题

#### 1. 相机启动失败
```bash
# 检查RealSense设备
rs-enumerate-devices -s

# 手动修复USB连接
sudo bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/unbind'
sleep 3  
sudo bash -c 'echo "2-3" > /sys/bus/usb/drivers/usb/bind'
```

#### 2. 检测API不可用
```bash
# 检查GraspAnything服务
ls /home/agilex/MobileManipulator/src/perception/scripts/server_grasp.json

# 检查依赖
python3 -c "from percept_dino_api_test import GraspAnythingAPI"
```

#### 3. TF链错误
```bash
# 检查TF树
rosrun tf2_tools view_frames.py

# 验证变换
rosrun tf tf_echo base_link camera/hand_depth_optical_frame
```

#### 4. 查看详细日志
```bash
# 启用调试模式
./start_unified_perception.sh urdf hand grasp true

# 或者查看ROS日志
tail -f ~/.ros/log/latest/rosout.log
```

## 💡 最佳实践

### 1. 新手入门
从最简单的配置开始：
```bash
./start_unified_perception.sh urdf hand grasp true
```

### 2. 性能优化
生产环境建议：
```bash  
./start_unified_perception.sh urdf hand grasp false
```

### 3. 功能测试
测试所有功能：
```bash
./start_unified_perception.sh urdf all all true
```

### 4. 算法调试
无相机模式快速测试：
```bash
./start_unified_perception.sh urdf none grasp true
```

---

## 📞 支持

如遇问题，请检查：
1. ROS环境是否正确设置
2. 相机硬件是否正常连接
3. 配置文件格式是否正确
4. 系统依赖是否完整

更多技术细节请参考源码注释和配置文件说明。