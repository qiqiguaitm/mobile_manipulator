#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import moveit_commander

def check_collision_directly():
    """直接检查特定关节配置的碰撞状态"""
    
    rospy.init_node('direct_collision_check', anonymous=True)
    
    print("\n" + "="*80)
    print("直接碰撞状态检查")
    print("="*80)
    
    # 等待服务
    rospy.wait_for_service('/check_state_validity')
    check_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    
    # 初始化MoveIt获取当前状态
    robot = moveit_commander.RobotCommander()
    
    # 定义测试配置
    test_configs = [
        {
            "name": "配置1: 机械臂向左指向lifting_Link",
            "joints": {
                "joint1": -1.57,  # 左转90度
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0
            },
            "expected_collision": True
        },
        {
            "name": "配置2: 机械臂向前伸展接近lifting_Link",
            "joints": {
                "joint1": -0.7,   # 稍微左转
                "joint2": -0.5,   # 向前倾
                "joint3": 0.5,
                "joint4": 0.0,
                "joint5": 0.5,
                "joint6": 0.0
            },
            "expected_collision": True
        },
        {
            "name": "配置3: 安全的零位",
            "joints": {
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0
            },
            "expected_collision": False
        },
        {
            "name": "配置4: 标准home位置",
            "joints": {
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": -1.57,
                "joint4": 0.0,
                "joint5": 1.57,
                "joint6": 0.0
            },
            "expected_collision": False
        }
    ]
    
    # 运行测试
    print("\n开始测试各种关节配置...\n")
    
    for config in test_configs:
        print(f"\n{config['name']}")
        print("-" * 60)
        
        # 创建机器人状态
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.name = list(config['joints'].keys())
        robot_state.joint_state.position = list(config['joints'].values())
        
        # 打印关节配置
        print("关节配置:")
        for joint, value in config['joints'].items():
            print(f"  {joint}: {value:.2f}")
        
        # 创建请求
        req = GetStateValidityRequest()
        req.robot_state = robot_state
        req.group_name = "piper"
        
        try:
            # 检查状态有效性
            resp = check_validity(req)
            
            if resp.valid:
                print("\n✓ 状态有效 (无碰撞)")
                collision_detected = False
            else:
                print("\n✗ 状态无效 (检测到碰撞)")
                collision_detected = True
                
                # 显示碰撞信息
                if resp.contacts:
                    print("\n碰撞接触点:")
                    for i, contact in enumerate(resp.contacts[:5]):  # 最多显示5个
                        print(f"  {i+1}. {contact.contact_body_1} <-> {contact.contact_body_2}")
                        print(f"     深度: {contact.depth:.3f}")
                
                # 显示成本源
                if resp.cost_sources:
                    print("\n碰撞成本源:")
                    for i, cost in enumerate(resp.cost_sources[:3]):  # 最多显示3个
                        if cost.cost > 0:
                            print(f"  - {cost.cost:.3f}")
            
            # 检查是否符合预期
            if collision_detected == config['expected_collision']:
                print(f"\n✅ 测试通过 - 碰撞检测结果符合预期")
            else:
                print(f"\n❌ 测试失败 - 碰撞检测结果不符合预期!")
                if config['expected_collision']:
                    print("   预期有碰撞，但没有检测到")
                else:
                    print("   预期无碰撞，但检测到了碰撞")
                    
        except Exception as e:
            print(f"\n错误: {str(e)}")
    
    # 额外测试：检查ACM状态
    print("\n" + "="*80)
    print("检查允许碰撞矩阵(ACM)中的关键设置")
    print("="*80)
    
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    
    try:
        get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        
        resp = get_scene(req)
        acm = resp.scene.allowed_collision_matrix
        
        # 检查关键碰撞对
        key_pairs = [
            ("lifting_Link", "link1"),
            ("lifting_Link", "link2"),
            ("lifting_Link", "link3"),
            ("lifting_Link", "arm_base"),
            ("lidar_Link", "link2"),
        ]
        
        print("\n关键碰撞对的ACM状态:")
        for link1, link2 in key_pairs:
            if link1 in acm.entry_names and link2 in acm.entry_names:
                idx1 = acm.entry_names.index(link1)
                idx2 = acm.entry_names.index(link2)
                
                if idx1 < len(acm.entry_values) and idx2 < len(acm.entry_values[idx1].enabled):
                    if acm.entry_values[idx1].enabled[idx2]:
                        print(f"  ✗ {link1} <-> {link2}: 碰撞检测被禁用")
                    else:
                        print(f"  ✓ {link1} <-> {link2}: 碰撞检测启用")
            else:
                print(f"  ? {link1} <-> {link2}: 未在ACM中找到")
                
    except Exception as e:
        print(f"无法获取ACM: {e}")
    
    print("\n" + "="*80)
    print("测试完成")
    print("="*80)

if __name__ == '__main__':
    try:
        check_collision_directly()
    except rospy.ROSInterruptException:
        pass