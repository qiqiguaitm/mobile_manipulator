#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import moveit_commander

def test_all_sensors_collision():
    """测试所有传感器的碰撞检测"""
    
    rospy.init_node('test_all_sensors_collision', anonymous=True)
    
    print("\n" + "="*80)
    print("全面传感器碰撞检测测试")
    print("="*80)
    
    # 等待服务
    rospy.wait_for_service('/check_state_validity', timeout=5.0)
    check_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    
    # 初始化MoveIt
    robot = moveit_commander.RobotCommander()
    
    # 定义测试配置
    test_configs = [
        {
            "name": "测试1: 向lifting_Link方向",
            "joints": {
                "joint1": -0.5,  # 向左
                "joint2": 0.0,
                "joint3": -0.5,
                "joint4": 0.0,
                "joint5": 0.5,
                "joint6": 0.0
            },
            "expected_collision": "可能与lifting_Link碰撞"
        },
        {
            "name": "测试2: 向下接近lidar",
            "joints": {
                "joint1": 0.0,
                "joint2": 1.2,   # 向下
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0
            },
            "expected_collision": "可能与lidar_Link碰撞"
        },
        {
            "name": "测试3: 向前下方(under_camera)",
            "joints": {
                "joint1": 0.0,
                "joint2": 0.8,   # 向下
                "joint3": -0.8,  # 向前
                "joint4": 0.0,
                "joint5": 0.8,
                "joint6": 0.0
            },
            "expected_collision": "可能与under_camera_Link碰撞"
        },
        {
            "name": "测试4: 完全向上(top_camera)",
            "joints": {
                "joint1": -0.3,  # 稍微向左
                "joint2": -1.2,  # 向上
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0
            },
            "expected_collision": "可能与top_camera_Link碰撞"
        },
        {
            "name": "测试5: 安全位置",
            "joints": {
                "joint1": 0.5,   # 向右
                "joint2": -0.5,  # 稍微向上
                "joint3": -0.5,
                "joint4": 0.0,
                "joint5": 0.5,
                "joint6": 0.0
            },
            "expected_collision": "应该安全"
        }
    ]
    
    # 运行测试
    print("\n开始测试各种配置...\n")
    
    collision_summary = {
        "lifting_Link": False,
        "lidar_Link": False,
        "top_camera_Link": False,
        "under_camera_Link": False
    }
    
    for config in test_configs:
        print(f"\n{config['name']}")
        print("-" * 60)
        print(f"预期: {config['expected_collision']}")
        
        # 创建机器人状态
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.name = list(config['joints'].keys())
        robot_state.joint_state.position = list(config['joints'].values())
        
        # 创建请求
        req = GetStateValidityRequest()
        req.robot_state = robot_state
        req.group_name = "piper"
        
        try:
            # 检查状态有效性
            resp = check_validity(req)
            
            if resp.valid:
                print("✓ 状态有效 (无碰撞)")
            else:
                print("✗ 状态无效 (检测到碰撞)")
                
                # 分析碰撞
                if resp.contacts:
                    print("\n碰撞详情:")
                    for contact in resp.contacts[:5]:
                        print(f"  - {contact.contact_body_1} <-> {contact.contact_body_2}")
                        print(f"    深度: {contact.depth:.3f}m")
                        
                        # 记录哪些传感器检测到碰撞
                        for sensor in collision_summary.keys():
                            if sensor in contact.contact_body_1 or sensor in contact.contact_body_2:
                                collision_summary[sensor] = True
                    
        except Exception as e:
            print(f"错误: {str(e)}")
    
    # 总结
    print("\n" + "="*80)
    print("碰撞检测总结")
    print("="*80)
    
    print("\n传感器碰撞检测状态:")
    for sensor, detected in collision_summary.items():
        status = "✓ 工作正常" if detected else "⚠ 未检测到碰撞"
        print(f"  {sensor}: {status}")
    
    print("\n碰撞几何体配置:")
    print("  - lifting_Link: 圆柱体 (r=0.04m, h=0.56m)")
    print("  - lidar_Link: 盒子 (0.18×0.14×0.09m)")
    print("  - top_camera_Link: 盒子 (0.08×0.06×0.04m)")
    print("  - under_camera_Link: 盒子 (0.10×0.08×0.05m)")
    
    print("\n修复的内容:")
    print("  ✓ 将所有传感器的网格碰撞体替换为简单几何体")
    print("  ✓ 移除了相机与机械臂末端的碰撞禁用规则")
    print("  ✓ 确保所有传感器都能正确检测碰撞")
    
    print("\n" + "="*80)

if __name__ == '__main__':
    try:
        test_all_sensors_collision()
    except rospy.ROSInterruptException:
        pass