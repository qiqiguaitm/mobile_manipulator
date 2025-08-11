#!/usr/bin/env python3
"""
最简单的Piper笛卡尔空间运动示例 - 确保可工作

基于当前机械臂位置，进行最小幅度的笛卡尔调整
"""

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import time
import copy

def simple_working_cartesian():
    """最简单可工作的笛卡尔运动"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_working_cartesian', anonymous=True)
    
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("=" * 50)
    print("最简单的笛卡尔空间运动示例")
    print("=" * 50)
    
    # 设置非常保守的参数
    arm_group.set_planning_time(5.0)
    arm_group.set_max_velocity_scaling_factor(0.05)  # 5%速度
    arm_group.set_max_acceleration_scaling_factor(0.05)  # 5%加速度
    arm_group.set_goal_position_tolerance(0.03)  # 3cm容差
    arm_group.set_goal_orientation_tolerance(1.0)  # 很大的朝向容差
    arm_group.set_num_planning_attempts(5)
    
    print("系统参数:")
    print(f"- 规划时间: 5秒")
    print(f"- 运动速度: 5%")
    print(f"- 位置容差: 3cm")
    print(f"- 朝向容差: 1.0 弧度")
    
    # 获取当前位姿
    print("\n获取当前机械臂状态...")
    current_pose = arm_group.get_current_pose().pose
    
    print(f"当前位置:")
    print(f"  X: {current_pose.position.x:.6f} m")
    print(f"  Y: {current_pose.position.y:.6f} m") 
    print(f"  Z: {current_pose.position.z:.6f} m")
    print(f"当前朝向:")
    print(f"  x: {current_pose.orientation.x:.6f}")
    print(f"  y: {current_pose.orientation.y:.6f}")
    print(f"  z: {current_pose.orientation.z:.6f}")
    print(f"  w: {current_pose.orientation.w:.6f}")
    
    # 测试1：最小的Z轴调整
    print("\n" + "="*30)
    print("测试1: Z轴微调 (+5mm)")
    print("="*30)
    
    target_pose1 = copy.deepcopy(current_pose)
    target_pose1.position.z += 0.005  # 上升5mm
    
    print(f"目标位置: Z = {target_pose1.position.z:.6f} m (变化: +5mm)")
    
    try:
        arm_group.set_pose_target(target_pose1)
        print("开始规划...")
        success1 = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success1:
            print("✅ 测试1成功！")
            
            # 验证位置
            new_pose = arm_group.get_current_pose().pose
            z_change = new_pose.position.z - current_pose.position.z
            print(f"实际Z轴变化: {z_change*1000:.2f}mm")
            
            time.sleep(2)
            result1 = True
        else:
            print("❌ 测试1失败")
            result1 = False
            
    except Exception as e:
        print(f"❌ 测试1出错: {e}")
        result1 = False
    
    # 测试2：回到原位置
    print("\n" + "="*30)
    print("测试2: 回到原位置")
    print("="*30)
    
    try:
        arm_group.set_pose_target(current_pose)
        print("规划回到原位置...")
        success2 = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success2:
            print("✅ 测试2成功！")
            result2 = True
        else:
            print("❌ 测试2失败")
            result2 = False
            
    except Exception as e:
        print(f"❌ 测试2出错: {e}")
        result2 = False
    
    time.sleep(1)
    
    # 测试3：Y轴微调
    print("\n" + "="*30)
    print("测试3: Y轴微调 (+3mm)")
    print("="*30)
    
    current_pose_now = arm_group.get_current_pose().pose
    target_pose3 = copy.deepcopy(current_pose_now)
    target_pose3.position.y += 0.003  # 右移3mm
    
    print(f"当前Y: {current_pose_now.position.y:.6f} m")
    print(f"目标Y: {target_pose3.position.y:.6f} m (变化: +3mm)")
    
    try:
        arm_group.set_pose_target(target_pose3)
        print("开始规划...")
        success3 = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success3:
            print("✅ 测试3成功！")
            
            # 验证位置
            new_pose = arm_group.get_current_pose().pose
            y_change = new_pose.position.y - current_pose_now.position.y
            print(f"实际Y轴变化: {y_change*1000:.2f}mm")
            
            result3 = True
        else:
            print("❌ 测试3失败")
            result3 = False
            
    except Exception as e:
        print(f"❌ 测试3出错: {e}")
        result3 = False
    
    # 总结
    print("\n" + "="*50)
    print("测试结果总结")
    print("="*50)
    
    results = [
        ("Z轴微调 (+5mm)", result1),
        ("回到原位置", result2), 
        ("Y轴微调 (+3mm)", result3)
    ]
    
    passed = 0
    for name, result in results:
        status = "✅ 成功" if result else "❌ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总计: {passed}/3 个测试通过")
    
    if passed > 0:
        print("\n🎉 笛卡尔空间控制可以工作！")
        print("\n✨ 成功的笛卡尔运动示例代码:")
        print("```python")
        print("import moveit_commander")
        print("import copy")
        print("")
        print("# 初始化")
        print("arm_group = moveit_commander.MoveGroupCommander('arm')")
        print("")
        print("# 设置保守参数")
        print("arm_group.set_max_velocity_scaling_factor(0.05)")
        print("arm_group.set_goal_position_tolerance(0.03)")
        print("")
        print("# 获取当前位姿")
        print("current_pose = arm_group.get_current_pose().pose")
        print("")
        print("# 创建目标位姿 - 微小调整")
        print("target_pose = copy.deepcopy(current_pose)")
        print("target_pose.position.z += 0.005  # 上升5mm")
        print("")
        print("# 执行笛卡尔运动")
        print("arm_group.set_pose_target(target_pose)")
        print("success = arm_group.go(wait=True)")
        print("arm_group.stop()")
        print("arm_group.clear_pose_targets()")
        print("```")
        
        print("\n💡 实用建议:")
        print("- 笛卡尔调整幅度保持在1-5mm范围内")
        print("- 使用较大的位置容差(2-3cm)")
        print("- 运动速度设置为5-10%") 
        print("- 每次只调整一个轴向")
        print("- 保持当前朝向不变")
        
    else:
        print("\n⚠️ 所有笛卡尔测试都失败了")
        print("可能的原因:")
        print("- 当前位置接近工作空间边界")
        print("- 逆运动学求解器配置问题")
        print("- 关节限制过于严格")
        print("\n建议:")
        print("- 先用关节运动到工作空间中心")
        print("- 检查MoveIt配置文件")
        print("- 使用纯关节空间控制")
    
    # 显示最终位置
    final_pose = arm_group.get_current_pose().pose
    print(f"\n📍 最终位置:")
    print(f"  X: {final_pose.position.x:.6f} m")
    print(f"  Y: {final_pose.position.y:.6f} m")
    print(f"  Z: {final_pose.position.z:.6f} m")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        simple_working_cartesian()
    except rospy.ROSInterruptException:
        print("示例被中断")
    except Exception as e:
        print(f"示例过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()