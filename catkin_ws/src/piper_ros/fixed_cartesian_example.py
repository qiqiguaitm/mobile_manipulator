#!/usr/bin/env python3
"""
修复笛卡尔运动的正确示例

解决轨迹起始点与当前状态不匹配的问题
"""

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import time
import copy

def fixed_cartesian_example():
    """修复后的笛卡尔运动示例"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fixed_cartesian_example', anonymous=True)
    
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("=" * 50)
    print("修复笛卡尔运动的正确示例")
    print("=" * 50)
    
    # 1. 关键修复：设置起始状态容差
    print("1. 设置系统参数...")
    arm_group.set_start_state_to_current_state()  # 关键！使用当前状态作为起始状态
    arm_group.set_planning_time(8.0)
    arm_group.set_max_velocity_scaling_factor(0.1)
    arm_group.set_max_acceleration_scaling_factor(0.1)
    arm_group.set_goal_position_tolerance(0.02)
    arm_group.set_goal_orientation_tolerance(0.3)
    
    # 2. 修复：设置轨迹容差参数
    rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.05)  # 增大起始容差
    
    print("✅ 参数设置完成")
    
    # 3. 获取并验证当前状态
    print("\n2. 获取当前机械臂状态...")
    
    # 等待状态同步
    rospy.sleep(1.0)
    
    current_joints = arm_group.get_current_joint_values()
    current_pose = arm_group.get_current_pose().pose
    
    print(f"当前关节角度: {[round(j, 4) for j in current_joints]}")
    print(f"当前位置: ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
    
    # 4. 方法1：同步状态后的笛卡尔运动
    print("\n3. 方法1：状态同步笛卡尔运动")
    print("-" * 30)
    
    try:
        # 重新设置起始状态
        arm_group.set_start_state_to_current_state()
        
        # 创建目标位姿（基于当前位姿的小幅调整）
        target_pose1 = copy.deepcopy(current_pose)
        target_pose1.position.z += 0.01  # 上升1cm
        
        print(f"目标位置: Z = {target_pose1.position.z:.4f} m (+1cm)")
        
        # 设置目标并规划
        arm_group.set_pose_target(target_pose1)
        
        print("开始规划...")
        plan = arm_group.plan()
        
        if plan[0]:  # plan[0] 是成功标志
            print("✅ 规划成功，执行轨迹...")
            success1 = arm_group.execute(plan[1], wait=True)  # plan[1] 是轨迹
            
            if success1:
                print("✅ 方法1执行成功！")
                
                # 验证位置
                new_pose = arm_group.get_current_pose().pose
                z_change = new_pose.position.z - current_pose.position.z
                print(f"实际Z轴变化: {z_change*100:.1f}cm")
                result1 = True
            else:
                print("❌ 执行失败")
                result1 = False
        else:
            print("❌ 规划失败")
            result1 = False
            
        arm_group.stop()
        arm_group.clear_pose_targets()
        
    except Exception as e:
        print(f"❌ 方法1出错: {e}")
        result1 = False
    
    time.sleep(2)
    
    # 5. 方法2：使用compute_cartesian_path
    print("\n4. 方法2：笛卡尔路径规划")  
    print("-" * 30)
    
    try:
        # 重新获取当前状态
        arm_group.set_start_state_to_current_state()
        current_pose_now = arm_group.get_current_pose().pose
        
        # 创建路径点
        waypoints = []
        waypoints.append(current_pose_now)
        
        # 第一个点：向右移动1cm
        wpose1 = copy.deepcopy(current_pose_now)
        wpose1.position.y += 0.01
        waypoints.append(wpose1)
        
        # 第二个点：向上移动1cm
        wpose2 = copy.deepcopy(wpose1)
        wpose2.position.z += 0.01
        waypoints.append(wpose2)
        
        print(f"路径点1: Y+1cm -> ({wpose1.position.x:.3f}, {wpose1.position.y:.3f}, {wpose1.position.z:.3f})")
        print(f"路径点2: Z+1cm -> ({wpose2.position.x:.3f}, {wpose2.position.y:.3f}, {wpose2.position.z:.3f})")
        
        # 计算笛卡尔路径
        (plan2, fraction) = arm_group.compute_cartesian_path(
            waypoints,
            0.01,    # 1cm步长
            0.0,     # 关节跳跃阈值
            True     # 避障
        )
        
        print(f"路径规划完成度: {fraction*100:.1f}%")
        
        if fraction > 0.8:  # 80%以上完成
            print("执行笛卡尔路径...")
            success2 = arm_group.execute(plan2, wait=True)
            
            if success2:
                print("✅ 方法2执行成功！")
                result2 = True
            else:
                print("❌ 路径执行失败")
                result2 = False
        else:
            print("❌ 路径规划不完整")
            result2 = False
            
    except Exception as e:
        print(f"❌ 方法2出错: {e}")
        result2 = False
    
    # 6. 方法3：关节-笛卡尔混合控制
    print("\n5. 方法3：混合控制法")
    print("-" * 30)
    
    try:
        # 步骤1：关节运动到一个验证过的位置
        print("步骤1：关节运动到安全位置...")
        
        # 使用很小的关节调整，确保可达
        current_joints_now = arm_group.get_current_joint_values()
        safe_joints = current_joints_now[:]
        safe_joints[0] += 0.05  # joint1 小幅调整
        safe_joints[5] += 0.05  # joint6 小幅调整
        
        print(f"目标关节: {[round(j, 4) for j in safe_joints]}")
        
        arm_group.set_joint_value_target(safe_joints)
        success3a = arm_group.go(wait=True)
        arm_group.stop()
        
        if success3a:
            print("✅ 关节运动成功")
            
            # 步骤2：在新位置做笛卡尔调整
            print("步骤2：笛卡尔微调...")
            rospy.sleep(1)  # 等待状态稳定
            
            arm_group.set_start_state_to_current_state()
            new_pose = arm_group.get_current_pose().pose
            
            target_pose3 = copy.deepcopy(new_pose)
            target_pose3.position.y -= 0.01  # 向左1cm
            
            arm_group.set_pose_target(target_pose3)
            
            plan3 = arm_group.plan()
            if plan3[0]:
                success3b = arm_group.execute(plan3[1], wait=True)
                if success3b:
                    print("✅ 方法3执行成功！")
                    result3 = True
                else:
                    print("❌ 笛卡尔调整失败")
                    result3 = False
            else:
                print("❌ 笛卡尔规划失败")
                result3 = False
                
            arm_group.stop()
            arm_group.clear_pose_targets()
        else:
            print("❌ 关节运动失败")
            result3 = False
            
    except Exception as e:
        print(f"❌ 方法3出错: {e}")
        result3 = False
    
    # 总结
    print("\n" + "=" * 50)
    print("测试结果总结")
    print("=" * 50)
    
    results = [
        ("状态同步笛卡尔运动", result1),
        ("笛卡尔路径规划", result2),
        ("混合控制法", result3)
    ]
    
    passed = 0
    for name, result in results:
        status = "✅ 成功" if result else "❌ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总计: {passed}/3 个方法成功")
    
    if passed > 0:
        print("\n🎉 笛卡尔运动修复成功！")
        
        print("\n🔧 关键修复要点:")
        print("1. ✅ 使用 set_start_state_to_current_state()")
        print("2. ✅ 增大轨迹起始容差到 0.05")
        print("3. ✅ 使用 plan() + execute() 而不是 go()")
        print("4. ✅ 在规划前等待状态同步")
        print("5. ✅ 使用小幅度调整 (1-2cm)")
        
        print("\n💡 可工作的笛卡尔代码模板:")
        print("```python")
        print("# 关键修复步骤")
        print("arm_group.set_start_state_to_current_state()")
        print("rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.05)")
        print("")
        print("# 获取当前位姿")
        print("current_pose = arm_group.get_current_pose().pose")
        print("")
        print("# 创建目标位姿")
        print("target_pose = copy.deepcopy(current_pose)")
        print("target_pose.position.z += 0.01  # 上升1cm")
        print("")
        print("# 规划和执行")
        print("arm_group.set_pose_target(target_pose)")
        print("plan = arm_group.plan()")
        print("if plan[0]:")
        print("    success = arm_group.execute(plan[1], wait=True)")
        print("arm_group.clear_pose_targets()")
        print("```")
        
    else:
        print("\n⚠️ 笛卡尔运动仍有问题")
        print("可能需要进一步检查MoveIt配置")
    
    print(f"\n📍 最终位置: {arm_group.get_current_pose().pose.position}")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        fixed_cartesian_example()
    except rospy.ROSInterruptException:
        print("示例被中断")
    except Exception as e:
        print(f"示例过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()