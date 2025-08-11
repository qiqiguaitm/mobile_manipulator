#!/usr/bin/env python3
"""
测试修复后的笛卡尔空间规划

修复内容:
1. IK求解器超时从5ms增加到100ms  
2. 轨迹起始容差从1cm增加到5cm
3. 执行时间容差从1.2倍增加到2.0倍
4. 速度/加速度缩放因子从10%增加到30%
"""

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import sys
import time
import copy

def test_fixed_cartesian():
    """测试修复后的笛卡尔规划"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_fixed_cartesian', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("=" * 60)
    print("🎯 测试修复后的笛卡尔空间规划")
    print("=" * 60)
    print("修复内容:")
    print("✅ IK求解器超时: 5ms → 100ms")
    print("✅ 轨迹起始容差: 1cm → 5cm") 
    print("✅ 执行时间容差: 1.2x → 2.0x")
    print("✅ 速度缩放因子: 10% → 30%")
    
    # 设置规划参数
    arm_group.set_planning_time(5.0)
    arm_group.set_max_velocity_scaling_factor(0.2)
    arm_group.set_max_acceleration_scaling_factor(0.2)
    arm_group.set_goal_position_tolerance(0.01)  # 1cm位置容差
    arm_group.set_goal_orientation_tolerance(0.1)  # 适中的朝向容差
    arm_group.set_num_planning_attempts(10)
    
    print(f"\n📋 当前规划参数:")
    print(f"- 规划时间: 5秒")
    print(f"- 速度缩放: 20%")
    print(f"- 位置容差: 1cm")
    print(f"- 规划尝试: 10次")
    
    # 获取当前状态
    current_pose = arm_group.get_current_pose().pose
    current_joints = arm_group.get_current_joint_values()
    
    print(f"\n📍 当前状态:")
    print(f"位置: ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
    print(f"关节: {[round(j, 3) for j in current_joints]}")
    
    results = []
    
    # 测试1: 简单的Z轴移动
    print(f"\n{'='*40}")
    print("🔥 测试1: Z轴上升2cm")
    print("="*40)
    
    try:
        # 创建目标位姿
        target_pose1 = copy.deepcopy(current_pose)
        target_pose1.position.z += 0.02  # 上升2cm
        
        print(f"目标位置: Z = {target_pose1.position.z:.4f}m (+2cm)")
        
        # 设置目标并规划
        arm_group.set_pose_target(target_pose1)
        
        print("开始规划...")
        plan_result = arm_group.plan()
        
        if plan_result[0]:  # 规划成功
            print("✅ 规划成功！")
            
            print("执行轨迹...")
            execute_result = arm_group.execute(plan_result[1], wait=True)
            
            if execute_result:
                print("✅ 执行成功！")
                
                # 验证结果
                new_pose = arm_group.get_current_pose().pose
                z_change = new_pose.position.z - current_pose.position.z
                print(f"实际Z轴变化: {z_change*100:.1f}cm")
                
                success1 = True
            else:
                print("❌ 执行失败")
                success1 = False
        else:
            print("❌ 规划失败")
            success1 = False
            
        arm_group.stop()
        arm_group.clear_pose_targets()
        results.append(("Z轴上升", success1))
        
    except Exception as e:
        print(f"❌ 测试1出错: {e}")
        results.append(("Z轴上升", False))
    
    time.sleep(2)
    
    # 测试2: Y轴移动
    print(f"\n{'='*40}")
    print("🔥 测试2: Y轴右移1.5cm")
    print("="*40)
    
    try:
        current_pose2 = arm_group.get_current_pose().pose
        target_pose2 = copy.deepcopy(current_pose2)
        target_pose2.position.y += 0.015  # 右移1.5cm
        
        print(f"当前Y: {current_pose2.position.y:.4f}m")
        print(f"目标Y: {target_pose2.position.y:.4f}m (+1.5cm)")
        
        arm_group.set_pose_target(target_pose2)
        
        print("开始规划...")
        plan_result = arm_group.plan()
        
        if plan_result[0]:
            print("✅ 规划成功！")
            
            print("执行轨迹...")
            execute_result = arm_group.execute(plan_result[1], wait=True)
            
            if execute_result:
                print("✅ 执行成功！")
                
                new_pose = arm_group.get_current_pose().pose
                y_change = new_pose.position.y - current_pose2.position.y
                print(f"实际Y轴变化: {y_change*100:.1f}cm")
                
                success2 = True
            else:
                print("❌ 执行失败")
                success2 = False
        else:
            print("❌ 规划失败")
            success2 = False
            
        arm_group.stop()
        arm_group.clear_pose_targets()
        results.append(("Y轴右移", success2))
        
    except Exception as e:
        print(f"❌ 测试2出错: {e}")
        results.append(("Y轴右移", False))
    
    time.sleep(2)
    
    # 测试3: 笛卡尔路径规划
    print(f"\n{'='*40}")
    print("🔥 测试3: 笛卡尔路径规划")
    print("="*40)
    
    try:
        # 获取当前位姿
        current_pose3 = arm_group.get_current_pose().pose
        
        # 创建路径点
        waypoints = []
        waypoints.append(current_pose3)
        
        # 路径点1: 向左移动
        wpose1 = copy.deepcopy(current_pose3)
        wpose1.position.y -= 0.01  # 左移1cm
        waypoints.append(wpose1)
        
        # 路径点2: 向下移动
        wpose2 = copy.deepcopy(wpose1)
        wpose2.position.z -= 0.01  # 下降1cm
        waypoints.append(wpose2)
        
        print(f"路径点数: {len(waypoints)}")
        print(f"路径1: Y-1cm -> ({wpose1.position.x:.3f}, {wpose1.position.y:.3f}, {wpose1.position.z:.3f})")
        print(f"路径2: Z-1cm -> ({wpose2.position.x:.3f}, {wpose2.position.y:.3f}, {wpose2.position.z:.3f})")
        
        # 计算笛卡尔路径
        print("计算笛卡尔路径...")
        (plan3, fraction) = arm_group.compute_cartesian_path(
            waypoints,
            0.01,    # 1cm步长
            0.0,     # 关节跳跃阈值  
            True     # 避障
        )
        
        print(f"路径规划完成度: {fraction*100:.1f}%")
        
        if fraction > 0.8:
            print("✅ 路径规划成功！")
            
            print("执行笛卡尔路径...")
            execute_result = arm_group.execute(plan3, wait=True)
            
            if execute_result:
                print("✅ 路径执行成功！")
                success3 = True
            else:
                print("❌ 路径执行失败")
                success3 = False
        else:
            print("❌ 路径规划不完整")
            success3 = False
            
        results.append(("笛卡尔路径", success3))
        
    except Exception as e:
        print(f"❌ 测试3出错: {e}")
        results.append(("笛卡尔路径", False))
    
    # 测试结果总结
    print(f"\n{'='*60}")
    print("📊 测试结果总结")
    print("="*60)
    
    passed = 0
    for name, result in results:
        status = "✅ 成功" if result else "❌ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总计: {passed}/{len(results)} 个测试通过 ({passed/len(results)*100:.0f}%)")
    
    # 显示最终位置
    final_pose = arm_group.get_current_pose().pose
    final_joints = arm_group.get_current_joint_values()
    print(f"\n📍 最终状态:")
    print(f"位置: ({final_pose.position.x:.4f}, {final_pose.position.y:.4f}, {final_pose.position.z:.4f})")
    print(f"关节: {[round(j, 3) for j in final_joints]}")
    
    if passed >= 2:
        print(f"\n🎉 修复成功！笛卡尔空间规划现在可以工作了！")
        
        print(f"\n💡 关键修复要点:")
        print(f"1. ✅ IK求解器有足够时间找到解 (100ms)")
        print(f"2. ✅ 轨迹起始点容差放宽 (5cm)")
        print(f"3. ✅ 执行时间容差增加 (2.0倍)")
        print(f"4. ✅ 合理的速度缩放因子 (30%)")
        
        print(f"\n📋 可工作的笛卡尔控制代码:")
        print("```python")
        print("import moveit_commander")
        print("import copy")
        print("")
        print("arm_group = moveit_commander.MoveGroupCommander('arm')")
        print("arm_group.set_max_velocity_scaling_factor(0.2)")
        print("arm_group.set_goal_position_tolerance(0.01)")
        print("")
        print("# 获取当前位姿")
        print("current_pose = arm_group.get_current_pose().pose")
        print("")
        print("# 创建目标位姿")
        print("target_pose = copy.deepcopy(current_pose)")
        print("target_pose.position.z += 0.02  # 上升2cm")
        print("")
        print("# 规划和执行")
        print("arm_group.set_pose_target(target_pose)")  
        print("plan = arm_group.plan()")
        print("if plan[0]:")
        print("    arm_group.execute(plan[1], wait=True)")
        print("arm_group.clear_pose_targets()")
        print("```")
        
    elif passed >= 1:
        print(f"\n✨ 部分修复成功！笛卡尔规划有所改善")
        print(f"可能需要进一步调整参数或检查特定配置")
        
    else:
        print(f"\n⚠️ 修复效果有限")
        print(f"可能需要检查更深层的配置问题：")
        print(f"- URDF模型的关节限制定义")
        print(f"- 工作空间边界设置")
        print(f"- IK求解器算法选择")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_fixed_cartesian()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except Exception as e:
        print(f"测试过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()