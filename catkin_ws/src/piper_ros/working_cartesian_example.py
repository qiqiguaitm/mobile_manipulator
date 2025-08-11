#!/usr/bin/env python3
"""
Piper机械臂笛卡尔空间运动的工作示例

这个示例展示了如何正确使用MoveIt进行笛卡尔空间控制
"""

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import sys
import time

def working_cartesian_example():
    """工作的笛卡尔空间运动示例"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('working_cartesian_example', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("=" * 60)
    print("Piper机械臂笛卡尔空间运动工作示例")
    print("=" * 60)
    
    # 设置保守的规划参数
    arm_group.set_planning_time(10.0)
    arm_group.set_max_velocity_scaling_factor(0.1)  # 10%速度
    arm_group.set_max_acceleration_scaling_factor(0.1)  # 10%加速度
    arm_group.set_goal_position_tolerance(0.05)  # 5cm位置容差
    arm_group.set_goal_orientation_tolerance(0.5)  # 较大朝向容差
    arm_group.set_num_planning_attempts(20)  # 多次尝试
    
    # 显示基本信息
    print(f"参考坐标系: {arm_group.get_planning_frame()}")
    print(f"末端执行器: {arm_group.get_end_effector_link()}")
    
    # 示例1: 使用预定义姿态进行笛卡尔控制
    print("\n--- 示例1: 预定义姿态笛卡尔控制 ---")
    try:
        # 先移动到一个安全的关节配置
        print("移动到安全的关节配置...")
        joint_goal = [0.0, 0.3, -0.5, 0.2, 0.0, 0.0]  # 安全的关节角度
        arm_group.set_joint_value_target(joint_goal)
        success1a = arm_group.go(wait=True)
        arm_group.stop()
        
        if success1a:
            print("✅ 关节运动成功")
            time.sleep(1)
            
            # 获取这个配置下的末端位姿
            current_pose = arm_group.get_current_pose().pose
            print(f"当前末端位姿:")
            print(f"  位置: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
            print(f"  朝向: x={current_pose.orientation.x:.4f}, y={current_pose.orientation.y:.4f}, z={current_pose.orientation.z:.4f}, w={current_pose.orientation.w:.4f}")
            
            # 现在基于这个位姿做笛卡尔调整
            print("\n基于当前位姿做小幅笛卡尔调整...")
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y + 0.02  # 向Y方向移动2cm
            target_pose.position.z = current_pose.position.z
            target_pose.orientation = current_pose.orientation  # 保持朝向
            
            print(f"目标位姿: x={target_pose.position.x:.4f}, y={target_pose.position.y:.4f}, z={target_pose.position.z:.4f}")
            
            arm_group.set_pose_target(target_pose)
            success1b = arm_group.go(wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()
            
            if success1b:
                print("✅ 笛卡尔调整成功！")
                result1 = True
            else:
                print("❌ 笛卡尔调整失败")
                result1 = False
        else:
            print("❌ 初始关节运动失败")
            result1 = False
    except Exception as e:
        print(f"❌ 示例1出错: {e}")
        result1 = False
    
    time.sleep(2)
    
    # 示例2: 使用更大的容差进行笛卡尔运动
    print("\n--- 示例2: 大容差笛卡尔运动 ---")
    try:
        # 设置更宽松的容差
        arm_group.set_goal_position_tolerance(0.08)  # 8cm
        arm_group.set_goal_orientation_tolerance(1.0)  # 很大的朝向容差
        
        # 移动到一个相对安全的笛卡尔位置
        target_pose2 = geometry_msgs.msg.Pose()
        target_pose2.position.x = 0.12   # 12cm前方
        target_pose2.position.y = 0.0    # 居中
        target_pose2.position.z = 0.15   # 15cm高度
        
        # 简单的朝向 - 向下
        quat = quaternion_from_euler(0, 1.0, 0)  # 向下倾斜
        target_pose2.orientation.x = quat[0]
        target_pose2.orientation.y = quat[1]
        target_pose2.orientation.z = quat[2]
        target_pose2.orientation.w = quat[3]
        
        print(f"目标位置: x={target_pose2.position.x:.4f}, y={target_pose2.position.y:.4f}, z={target_pose2.position.z:.4f}")
        
        arm_group.set_pose_target(target_pose2)
        success2 = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success2:
            print("✅ 大容差笛卡尔运动成功！")
            result2 = True
        else:
            print("❌ 大容差笛卡尔运动失败")
            result2 = False
    except Exception as e:
        print(f"❌ 示例2出错: {e}")
        result2 = False
    
    time.sleep(2)
    
    # 示例3: 关节空间到笛卡尔空间的组合运动
    print("\n--- 示例3: 关节+笛卡尔组合运动 ---")
    try:
        # 第一步：关节运动到一个好的配置
        print("第一步：关节运动...")
        joint_goal3 = [0.2, 0.2, -0.3, 0.1, 0.0, 0.1]
        arm_group.set_joint_value_target(joint_goal3)
        success3a = arm_group.go(wait=True)
        arm_group.stop()
        
        if success3a:
            print("✅ 关节运动完成")
            time.sleep(1)
            
            # 第二步：在这个基础上做笛卡尔微调
            print("第二步：笛卡尔微调...")
            current_pose = arm_group.get_current_pose().pose
            
            # 只做Z方向的调整
            target_pose3 = geometry_msgs.msg.Pose()
            target_pose3.position.x = current_pose.position.x
            target_pose3.position.y = current_pose.position.y
            target_pose3.position.z = current_pose.position.z + 0.03  # 上升3cm
            target_pose3.orientation = current_pose.orientation
            
            arm_group.set_pose_target(target_pose3)
            success3b = arm_group.go(wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()
            
            if success3b:
                print("✅ 组合运动成功！")
                result3 = True
            else:
                print("❌ 笛卡尔微调失败")
                result3 = False
        else:
            print("❌ 关节运动失败")
            result3 = False
    except Exception as e:
        print(f"❌ 示例3出错: {e}")
        result3 = False
    
    # 总结
    print("\n" + "=" * 60)
    print("测试结果总结")
    print("=" * 60)
    
    results = [
        ("预定义姿态笛卡尔控制", result1),
        ("大容差笛卡尔运动", result2),
        ("关节+笛卡尔组合运动", result3)
    ]
    
    passed = 0
    for name, result in results:
        status = "✅ 成功" if result else "❌ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总计: {passed}/{len(results)} 个示例成功")
    
    if passed > 0:
        print("\n🎉 笛卡尔空间控制可以工作！")
        print("\n💡 成功要点:")
        print("1. 先用关节运动到一个已知安全的配置")
        print("2. 在这个基础上做小幅的笛卡尔调整(1-3cm)")
        print("3. 使用较大的位置和朝向容差")
        print("4. 降低运动速度和加速度")
        print("5. 保持当前朝向不变，只调整位置")
        
        print("\n📋 推荐的笛卡尔运动流程:")
        print("```python")
        print("# 1. 关节运动到安全配置")
        print("joint_goal = [0.0, 0.3, -0.5, 0.2, 0.0, 0.0]")
        print("arm_group.set_joint_value_target(joint_goal)")
        print("arm_group.go(wait=True)")
        print("")
        print("# 2. 获取当前位姿")
        print("current_pose = arm_group.get_current_pose().pose")
        print("")
        print("# 3. 基于当前位姿做小幅调整")
        print("target_pose = copy.deepcopy(current_pose)")
        print("target_pose.position.y += 0.02  # 小幅移动")
        print("")
        print("# 4. 执行笛卡尔运动")
        print("arm_group.set_pose_target(target_pose)")
        print("arm_group.go(wait=True)")
        print("```")
    else:
        print("\n⚠️ 所有笛卡尔运动都失败了")
        print("可能需要检查:")
        print("- 机械臂URDF模型中的关节限制")
        print("- MoveIt配置中的工作空间设置")
        print("- 逆运动学求解器配置")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        working_cartesian_example()
    except rospy.ROSInterruptException:
        print("示例被中断")
    except Exception as e:
        print(f"示例过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()