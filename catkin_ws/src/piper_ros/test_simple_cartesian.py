#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import sys
import time
import copy

def test_simple_cartesian():
    """测试简单的笛卡尔空间运动"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_simple_cartesian', anonymous=True)
    
    # 创建move group
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("========== 简单笛卡尔空间运动测试 ==========")
    
    # 设置宽松的规划参数
    arm_group.set_planning_time(15)
    arm_group.set_max_velocity_scaling_factor(0.2)
    arm_group.set_max_acceleration_scaling_factor(0.2)
    arm_group.set_goal_position_tolerance(0.02)
    arm_group.set_goal_orientation_tolerance(0.2)
    arm_group.set_num_planning_attempts(10)
    
    # 获取当前位姿
    current_pose = arm_group.get_current_pose().pose
    print(f"当前位姿: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
    
    # 方法1: 保持当前朝向，只做微小位置调整
    print("\n--- 方法1: 微小位置调整 ---")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = current_pose.position.x
    pose_goal.position.y = current_pose.position.y  
    pose_goal.position.z = current_pose.position.z - 0.01  # 下降1cm
    
    # 保持完全相同的朝向
    pose_goal.orientation = current_pose.orientation
    
    print(f"目标位姿: x={pose_goal.position.x:.4f}, y={pose_goal.position.y:.4f}, z={pose_goal.position.z:.4f}")
    
    arm_group.set_pose_target(pose_goal)
    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    if success:
        print("✅ 方法1执行成功")
        time.sleep(2)
        
        # 方法2: 回到原位置
        print("\n--- 方法2: 回到原位置 ---")
        arm_group.set_pose_target(current_pose)
        success2 = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success2:
            print("✅ 方法2执行成功")
        else:
            print("❌ 方法2执行失败")
    else:
        print("❌ 方法1执行失败")
        success2 = False
    
    # 方法3: 尝试关节空间的简单笛卡尔规划
    print("\n--- 方法3: 笛卡尔路径(关节空间) ---")
    try:
        # 先移动到一个已知好的关节配置
        arm_group.set_named_target("zero")  # 使用预定义的姿态
        success3a = arm_group.go(wait=True)
        arm_group.stop()
        
        if success3a:
            print("✅ 移动到zero姿态成功")
            time.sleep(2)
            
            # 从zero姿态做一个简单的笛卡尔移动
            zero_pose = arm_group.get_current_pose().pose
            print(f"Zero姿态位置: x={zero_pose.position.x:.4f}, y={zero_pose.position.y:.4f}, z={zero_pose.position.z:.4f}")
            
            # 在zero基础上做微调
            target_pose = copy.deepcopy(zero_pose)
            target_pose.position.z += 0.02  # 上升2cm
            
            arm_group.set_pose_target(target_pose)
            success3b = arm_group.go(wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()
            
            if success3b:
                print("✅ 方法3执行成功")
            else:
                print("❌ 方法3执行失败")
        else:
            print("❌ 无法移动到zero姿态")
            success3b = False
    except Exception as e:
        print(f"❌ 方法3出错: {e}")
        success3b = False
    
    # 方法4: 使用compute_cartesian_path
    print("\n--- 方法4: 笛卡尔路径计算 ---")
    try:
        waypoints = []
        
        # 获取当前位姿
        wpose = arm_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        
        # 添加一个很小的移动
        wpose.position.z -= 0.01  # 下降1cm
        waypoints.append(copy.deepcopy(wpose))
        
        # 计算路径
        (plan, fraction) = arm_group.compute_cartesian_path(
            waypoints,
            0.005,  # 0.5cm步长
            0.0     # 无跳跃检测
        )
        
        print(f"路径计算完成度: {fraction*100:.1f}%")
        
        if fraction > 0.5:
            success4 = arm_group.execute(plan, wait=True)
            if success4:
                print("✅ 方法4执行成功")
            else:
                print("❌ 方法4执行失败")
        else:
            print("❌ 路径计算不完整")
            success4 = False
    except Exception as e:
        print(f"❌ 方法4出错: {e}")
        success4 = False
    
    # 总结结果
    print("\n========== 测试结果总结 ==========")
    results = [success, success2, success3b, success4]
    passed = sum(results)
    
    print(f"成功执行: {passed}/4 个测试")
    
    if passed > 0:
        print("🎉 至少有一些笛卡尔运动可以工作！")
        print("\n💡 建议:")
        print("- 使用微小的位置调整(1-2cm)")
        print("- 保持当前朝向不变")
        print("- 设置较大的位置和朝向容差")
        print("- 使用较低的速度和加速度")
    else:
        print("⚠️ 所有笛卡尔运动都失败了")
        print("可能原因: 工作空间限制、关节限制或逆运动学求解困难")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_simple_cartesian()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except Exception as e:
        print(f"测试过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()