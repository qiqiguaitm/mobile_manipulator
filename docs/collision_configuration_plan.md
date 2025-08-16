# 机器人系统碰撞配置方案

## 系统组件结构

### 1. 移动底盘部分
```
world_link
└── base_link (移动底盘)
    ├── box_Link (电箱/控制箱)
    │   ├── lifting_Link (升降机构/支撑杆)
    │   │   └── top_camera_Link (上方相机)
    │   └── arm_base (机械臂基座) → 连接到机械臂
    ├── lidar_Link (激光雷达)
    └── under_camera_Link (下方相机)
```

### 2. 机械臂部分
```
arm_base
├── link1 → link2 → link3 → link4 → link5 → link6
└── link7, link8 (夹爪)
```

### 3. 碰撞问题分类

#### A. 机械臂自碰撞
- **相邻连杆**: link1-link2, link2-link3等（天然相连）
- **不可能碰撞**: arm_base-link3, link1-link4等（运动学限制）
- **潜在碰撞**: link2-link5, link3-link6等（需要检测）

#### B. 机械臂与底盘碰撞
- **arm_base与box_Link**: 固定连接，忽略
- **link1-3与base_link**: 可能碰撞，需检测
- **link1-3与lifting_Link**: 高风险碰撞区域
- **link4-6与top_camera_Link**: 工作空间重叠

#### C. 机械臂与传感器碰撞
- **与lidar_Link**: link1-2可能碰撞
- **与under_camera_Link**: link1-3可能碰撞
- **与top_camera_Link**: link3-6可能碰撞

#### D. 环境碰撞
- **地面**: 所有连杆需要z>0约束
- **工作空间边界**: 定义安全工作区域

## 碰撞配置策略

### 1. ACM（Allowed Collision Matrix）配置

#### 永远忽略的碰撞对（Adjacent类型）
```xml
<!-- 移动平台内部 -->
<disable_collisions link1="base_link" link2="box_Link" reason="Adjacent"/>
<disable_collisions link1="box_Link" link2="lifting_Link" reason="Adjacent"/>
<disable_collisions link1="box_Link" link2="arm_base" reason="Adjacent"/>
<disable_collisions link1="lifting_Link" link2="top_camera_Link" reason="Adjacent"/>
<disable_collisions link1="base_link" link2="lidar_Link" reason="Adjacent"/>
<disable_collisions link1="base_link" link2="under_camera_Link" reason="Adjacent"/>

<!-- 机械臂相邻连杆 -->
<disable_collisions link1="arm_base" link2="link1" reason="Adjacent"/>
<disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
<disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
<disable_collisions link1="link3" link2="link4" reason="Adjacent"/>
<disable_collisions link1="link4" link2="link5" reason="Adjacent"/>
<disable_collisions link1="link5" link2="link6" reason="Adjacent"/>
<disable_collisions link1="link6" link2="link7" reason="Adjacent"/>
<disable_collisions link1="link6" link2="link8" reason="Adjacent"/>
```

#### 永不碰撞的对（Never类型）
```xml
<!-- 运动学限制导致不可能碰撞 -->
<disable_collisions link1="arm_base" link2="link3" reason="Never"/>
<disable_collisions link1="arm_base" link2="link4" reason="Never"/>
<disable_collisions link1="link1" link2="link4" reason="Never"/>
<disable_collisions link1="link1" link2="link5" reason="Never"/>

<!-- 固定部件之间 -->
<disable_collisions link1="lidar_Link" link2="under_camera_Link" reason="Never"/>
<disable_collisions link1="box_Link" link2="lidar_Link" reason="Never"/>
```

#### 默认接触的对（Default类型）
```xml
<!-- 初始位置就接触或非常接近 -->
<disable_collisions link1="arm_base" link2="box_Link" reason="Default"/>
<disable_collisions link1="lifting_Link" link2="arm_base" reason="Default"/>
```

### 2. 关键碰撞检测对（必须保留）

#### 高风险碰撞对
```yaml
critical_collisions:
  # 机械臂与升降杆
  - [link1, lifting_Link]     # 非常容易碰撞
  - [link2, lifting_Link]     # 主臂摆动时
  - [link3, lifting_Link]     # 前臂展开时
  
  # 机械臂与顶部相机
  - [link3, top_camera_Link]  # 抬臂时
  - [link4, top_camera_Link]  # 腕部运动
  - [link5, top_camera_Link]  # 末端运动
  
  # 机械臂与底盘
  - [link2, base_link]        # 下压时
  - [link3, base_link]        # 完全展开时
  
  # 机械臂与传感器
  - [link1, lidar_Link]       # 旋转时
  - [link2, lidar_Link]       # 向前伸展时
  - [link2, under_camera_Link] # 向前下方运动
```

### 3. 环境约束配置

#### 地面约束
```yaml
ground_constraint:
  type: "plane"
  point: [0, 0, 0]
  normal: [0, 0, 1]
  padding: 0.05  # 5cm安全距离
```

#### 工作空间限制
```yaml
workspace_limits:
  x: [-0.5, 0.8]   # 相对于arm_base
  y: [-0.6, 0.6]
  z: [0.05, 1.2]   # 地面以上5cm
```

### 4. 碰撞几何优化建议

#### 简化策略
1. **底盘组件**: 使用简化的盒子/圆柱体代替复杂网格
2. **传感器**: 使用包围盒，增加padding
3. **升降杆**: 使用圆柱体原语
4. **相机**: 使用简单立方体

#### 碰撞padding设置
```yaml
collision_padding:
  default: 0.01          # 默认1cm
  arm_links: 0.02        # 机械臂2cm
  sensors: 0.03          # 传感器3cm
  lifting_Link: 0.05     # 升降杆5cm（高风险）
```

## 实施步骤

### 第一阶段：基础配置
1. 更新SRDF文件，添加所有Adjacent碰撞对
2. 添加Never类型的碰撞对
3. 测试基础运动功能

### 第二阶段：优化配置  
1. 识别并添加Default类型碰撞对
2. 简化底盘和传感器的碰撞几何
3. 调整collision padding

### 第三阶段：高级功能
1. 添加动态碰撞物体（如地面）
2. 配置工作空间约束
3. 创建碰撞检测可视化工具

### 第四阶段：测试验证
1. 单元测试每个碰撞对
2. 完整运动范围测试
3. 极限位置测试

## 注意事项

### Linus哲学应用
1. **简单优先**: 只配置必要的碰撞对，不过度设计
2. **实用主义**: 基于实际使用场景，不是理论完美
3. **渐进改进**: 先解决最常见问题，逐步优化
4. **保持兼容**: 所有改动可回退，不破坏现有功能

### 性能考虑
- 碰撞对数量直接影响规划速度
- 优先禁用不可能的碰撞
- 复杂几何用简单形状替代
- 合理设置padding避免过度保守

### 安全原则
- 宁可保守不可激进
- 关键碰撞对必须保留
- 充分测试后再部署
- 保留紧急停止能力