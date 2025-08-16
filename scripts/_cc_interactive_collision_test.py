#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def interactive_collision_test():
    """交互式测试碰撞检测"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('interactive_collision_test', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("piper")
    
    print("\n" + "="*80)
    print("交互式碰撞检测测试")
    print("="*80)
    print("\n说明：输入关节角度测试碰撞检测，输入'q'退出")
    print("示例：-1.57 0 0 0 0 0 (6个关节角度，空格分隔)")
    
    # 设置规划参数
    group.set_planning_time(3)
    group.set_num_planning_attempts(1)
    
    # 显示当前关节值
    current = group.get_current_joint_values()
    print("\n当前关节角度:")
    for i, val in enumerate(current):
        print(f"  joint{i+1}: {val:.3f}")
    
    # 主循环
    while not rospy.is_shutdown():
        print("\n" + "-"*60)
        user_input = input("\n输入6个关节角度 (或 'q' 退出): ")
        
        if user_input.lower() == 'q':
            break
        
        try:
            # 解析输入
            values = [float(x) for x in user_input.split()]
            
            if len(values) != 6:
                print("错误：需要6个关节角度")
                continue
            
            print("\n目标关节角度:")
            for i, val in enumerate(values):
                print(f"  joint{i+1}: {val:.3f}")
            
            # 设置目标
            group.set_joint_value_target(values)
            
            # 尝试规划
            print("\n规划中...")
            success, plan, planning_time, error_code = group.plan()
            
            if success:
                print(f"\n✓ 规划成功!")
                print(f"  规划时间: {planning_time:.2f}秒")
                print(f"  路径点数: {len(plan.joint_trajectory.points)}")
                
                # 询问是否执行
                execute = input("\n是否执行此路径? (y/n): ")
                if execute.lower() == 'y':
                    print("执行中...")
                    group.execute(plan, wait=True)
                    print("执行完成!")
                else:
                    print("取消执行")
            else:
                print(f"\n✗ 规划失败!")
                print(f"  错误代码: {error_code.val}")
                
                if error_code.val == -3:
                    print("  原因: 检测到碰撞")
                    print("  说明: 机械臂会与环境发生碰撞")
                elif error_code.val == -2:
                    print("  原因: 无效的运动计划")
                elif error_code.val == -7:
                    print("  原因: 目标约束无法满足")
                else:
                    print(f"  原因: 错误代码 {error_code.val}")
            
        except ValueError:
            print("错误：请输入有效的数字")
        except Exception as e:
            print(f"错误: {str(e)}")
    
    # 清理
    print("\n退出...")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        interactive_collision_test()
    except rospy.ROSInterruptException:
        pass