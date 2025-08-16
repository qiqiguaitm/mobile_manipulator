#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from math import pi

def run_collision_tests():
    """运行一系列碰撞检测测试"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('collision_test_cases', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("piper")
    
    print("\n" + "="*80)
    print("碰撞检测测试案例")
    print("="*80)
    
    # 设置规划参数
    group.set_planning_time(5)
    group.set_num_planning_attempts(3)
    group.allow_replanning(True)
    
    # 测试案例列表
    test_cases = [
        {
            "name": "测试1: 向左旋转撞击lifting_Link",
            "joints": [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0],  # joint1左转90度
            "expected": "失败",
            "reason": "机械臂会撞到lifting_Link"
        },
        {
            "name": "测试2: 向前伸展撞击lifting_Link", 
            "joints": [-0.5, -0.5, 0.5, 0.0, 0.5, 0.0],  # 向前伸展姿态
            "expected": "失败",
            "reason": "末端会撞到lifting_Link"
        },
        {
            "name": "测试3: 安全的home位置",
            "joints": [0.0, 0.0, -1.57, 0.0, 1.57, 0.0],  # home姿态
            "expected": "成功",
            "reason": "标准home位置应该安全"
        },
        {
            "name": "测试4: 向右安全区域",
            "joints": [1.57, 0.0, -1.0, 0.0, 1.0, 0.0],  # 向右侧
            "expected": "成功", 
            "reason": "远离lifting_Link"
        },
        {
            "name": "测试5: 低位撞击lidar",
            "joints": [0.0, 1.57, 0.0, 0.0, 0.0, 0.0],  # joint2向下
            "expected": "失败",
            "reason": "可能撞到lidar_Link"
        }
    ]
    
    # 运行测试
    results = []
    
    for i, test in enumerate(test_cases):
        print(f"\n{test['name']}")
        print("-" * 60)
        print(f"目标关节角度: {test['joints']}")
        print(f"预期结果: {test['expected']}")
        print(f"原因: {test['reason']}")
        
        # 设置目标
        group.set_joint_value_target(test['joints'])
        
        # 尝试规划
        print("\n规划中...")
        success, plan, planning_time, error_code = group.plan()
        
        # 记录结果
        if success:
            result = "成功"
            print(f"✓ 规划成功 (耗时: {planning_time:.2f}秒)")
            print(f"  路径包含 {len(plan.joint_trajectory.points)} 个点")
        else:
            result = "失败"
            print(f"✗ 规划失败")
            print(f"  错误代码: {error_code.val}")
            if error_code.val == -3:
                print("  原因: 目标位置有碰撞")
            elif error_code.val == -2:
                print("  原因: 无效的运动计划")
            elif error_code.val == -1:
                print("  原因: 规划超时")
        
        # 检查是否符合预期
        if result == test['expected']:
            print(f"\n✅ 测试通过 - 结果符合预期")
            results.append((test['name'], True))
        else:
            print(f"\n❌ 测试失败 - 结果不符合预期!")
            print(f"   预期: {test['expected']}, 实际: {result}")
            results.append((test['name'], False))
    
    # 总结
    print("\n" + "="*80)
    print("测试总结")
    print("="*80)
    
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    print(f"\n总测试数: {total}")
    print(f"通过: {passed}")
    print(f"失败: {total - passed}")
    print(f"通过率: {passed/total*100:.1f}%")
    
    print("\n详细结果:")
    for name, success in results:
        print(f"  {'✅' if success else '❌'} {name}")
    
    # 诊断建议
    if passed < total:
        print("\n" + "="*80)
        print("诊断建议")
        print("="*80)
        print("\n如果测试失败，可能的原因：")
        print("1. SRDF配置未正确加载 - 需要重新构建工作空间")
        print("2. MoveIt使用了缓存配置 - 需要完全重启ROS")
        print("3. 碰撞几何体定义有问题 - 检查URDF中的collision定义")
        print("\n建议操作：")
        print("1. 运行: catkin clean && catkin build")
        print("2. 完全重启系统或终止所有ROS进程")
        print("3. 运行动态碰撞启用脚本: python _cc_enable_all_collisions.py")
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        run_collision_tests()
    except rospy.ROSInterruptException:
        pass