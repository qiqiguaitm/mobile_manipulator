#!/usr/bin/env python3
"""
Piper机械臂实用笛卡尔空间运动示例

基于已验证可工作的关节运动，提供实际可用的笛卡尔控制方案
"""

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
import time
import copy

def practical_cartesian_example():
    """实用的笛卡尔空间运动示例"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('practical_cartesian_example', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("=" * 60)
    print("Piper机械臂实用笛卡尔空间运动示例")
    print("=" * 60)
    print("基于已验证的关节运动，展示笛卡尔空间的实用控制方法")
    
    # 保守的规划参数
    arm_group.set_planning_time(8.0)
    arm_group.set_max_velocity_scaling_factor(0.15)
    arm_group.set_max_acceleration_scaling_factor(0.15)
    arm_group.set_num_planning_attempts(15)
    
    print("\n方法：先关节运动建立位姿，再分析笛卡尔可达性")
    
    # 已知可工作的关节配置（从之前的测试得出）
    working_joint_configs = [
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "零位"),
        ([0.2, 0.1, -0.1, 0.1, -0.1, 0.2], "测试位置A"),
        ([0.1, 0.2, -0.2, 0.15, 0.0, 0.1], "测试位置B"),
        ([0.0, 0.1, -0.1, 0.0, 0.0, 0.0], "测试位置C")
    ]
    
    successful_poses = []
    
    print("\n第一阶段：建立关节-笛卡尔位姿映射表")
    print("-" * 40)
    
    for i, (joints, name) in enumerate(working_joint_configs):
        try:
            print(f"\n{i+1}. 测试 {name}: {joints}")
            
            # 执行关节运动
            arm_group.set_joint_value_target(joints)
            success = arm_group.go(wait=True)
            arm_group.stop()
            
            if success:
                # 获取该关节配置对应的笛卡尔位姿
                pose = arm_group.get_current_pose().pose
                
                # 转换四元数到欧拉角便于理解
                orientation_list = [pose.orientation.x, pose.orientation.y, 
                                  pose.orientation.z, pose.orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                
                pose_info = {
                    'name': name,
                    'joints': joints,
                    'pose': pose,
                    'position': (pose.position.x, pose.position.y, pose.position.z),
                    'orientation': (roll, pitch, yaw)
                }
                successful_poses.append(pose_info)
                
                print(f"  ✅ 成功 - 位置: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})")
                print(f"         方向: (roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f})")
                
                time.sleep(1)
            else:
                print(f"  ❌ 关节运动失败")
                
        except Exception as e:
            print(f"  ❌ 错误: {e}")
    
    if len(successful_poses) < 2:
        print("\n⚠️ 可用的位姿不足，无法进行笛卡尔测试")
        moveit_commander.roscpp_shutdown()
        return
    
    print(f"\n✅ 建立了 {len(successful_poses)} 个有效位姿")
    
    # 第二阶段：基于已知位姿进行笛卡尔运动
    print("\n第二阶段：基于已知位姿的笛卡尔运动")
    print("-" * 40)
    
    success_count = 0
    
    # 示例1：在两个已知位姿之间进行笛卡尔运动
    if len(successful_poses) >= 2:
        print("\n示例1：已知位姿间的笛卡尔切换")
        try:
            # 从第一个位姿到第二个位姿
            pose1 = successful_poses[0]
            pose2 = successful_poses[1]
            
            print(f"从 {pose1['name']} 到 {pose2['name']}")
            
            # 先到第一个位姿
            arm_group.set_joint_value_target(pose1['joints'])
            arm_group.go(wait=True)
            arm_group.stop()
            
            time.sleep(1)
            
            # 尝试用笛卡尔方式到第二个位姿
            arm_group.set_pose_target(pose2['pose'])
            success = arm_group.go(wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()
            
            if success:
                print("  ✅ 笛卡尔运动成功！")
                success_count += 1
                
                # 验证到达位置
                current = arm_group.get_current_pose().pose
                error_x = abs(current.position.x - pose2['pose'].position.x)
                error_y = abs(current.position.y - pose2['pose'].position.y)
                error_z = abs(current.position.z - pose2['pose'].position.z)
                print(f"  位置误差: x={error_x:.4f}m, y={error_y:.4f}m, z={error_z:.4f}m")
            else:
                print("  ❌ 笛卡尔运动失败，回退到关节运动")
                # 用关节运动作为备选
                arm_group.set_joint_value_target(pose2['joints'])
                arm_group.go(wait=True)
                arm_group.stop()
        except Exception as e:
            print(f"  ❌ 示例1错误: {e}")
    
    # 示例2：基于当前位姿的微调
    print("\n示例2：基于当前位姿的微小调整")
    try:
        current_pose = arm_group.get_current_pose().pose
        print(f"当前位置: ({current_pose.position.x:.4f}, {current_pose.position.y:.4f}, {current_pose.position.z:.4f})")
        
        # 创建微调目标（只改变一个轴）
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z += 0.01  # 上升1cm
        
        print(f"目标位置: ({target_pose.position.x:.4f}, {target_pose.position.y:.4f}, {target_pose.position.z:.4f})")
        
        # 设置更宽松的容差
        arm_group.set_goal_position_tolerance(0.02)
        arm_group.set_goal_orientation_tolerance(0.5)
        
        arm_group.set_pose_target(target_pose)
        success = arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        
        if success:
            print("  ✅ 微调成功！")
            success_count += 1
        else:
            print("  ❌ 微调失败")
    except Exception as e:
        print(f"  ❌ 示例2错误: {e}")
    
    # 示例3：笛卡尔路径规划（使用compute_cartesian_path的正确方式）
    print("\n示例3：笛卡尔路径规划")
    try:
        waypoints = []
        
        # 当前位姿
        wpose = arm_group.get_current_pose().pose
        waypoints.append(wpose)
        
        # 向右移动
        wpose2 = copy.deepcopy(wpose)
        wpose2.position.y += 0.02
        waypoints.append(wpose2)
        
        # 向前移动  
        wpose3 = copy.deepcopy(wpose2)
        wpose3.position.x += 0.01
        waypoints.append(wpose3)
        
        # 调用compute_cartesian_path（修正参数）
        (plan, fraction) = arm_group.compute_cartesian_path(
            waypoints,   # 路径点
            0.01,        # eef_step: 末端执行器步长
            0.0,         # jump_threshold: 关节空间跳跃阈值
            True         # avoid_collisions: 避障
        )
        
        print(f"  路径规划完成度: {fraction*100:.1f}%")
        
        if fraction > 0.8:
            print("  执行笛卡尔路径...")
            success = arm_group.execute(plan, wait=True)
            if success:
                print("  ✅ 路径执行成功！")
                success_count += 1
            else:
                print("  ❌ 路径执行失败")
        else:
            print("  ❌ 路径规划不完整，跳过执行")
            
    except Exception as e:
        print(f"  ❌ 示例3错误: {e}")
    
    # 总结和建议
    print("\n" + "=" * 60)
    print("测试结果总结")
    print("=" * 60)
    
    print(f"成功的笛卡尔运动: {success_count}/3")
    
    if success_count > 0:
        print("\n🎉 笛卡尔空间控制部分工作正常！")
        
        print("\n📋 实用的笛卡尔控制策略:")
        print("1. **混合控制法**: 用关节运动到达主要位置，用笛卡尔做精细调整")
        print("2. **已知位姿切换**: 在预先验证的位姿之间进行笛卡尔运动")
        print("3. **微小调整**: 基于当前位姿做1-2cm的小幅调整")
        print("4. **路径规划**: 对于复杂轨迹使用compute_cartesian_path")
        
        print("\n💡 推荐的实际应用代码:")
        print("```python")
        print("# 混合控制示例")
        print("# 1. 关节运动到目标附近")
        print("arm_group.set_joint_value_target([0.2, 0.1, -0.1, 0.1, -0.1, 0.2])")
        print("arm_group.go(wait=True)")
        print("")
        print("# 2. 笛卡尔精细调整")
        print("current_pose = arm_group.get_current_pose().pose")
        print("target_pose = copy.deepcopy(current_pose)")
        print("target_pose.position.z += 0.01  # 上升1cm")
        print("arm_group.set_pose_target(target_pose)")
        print("arm_group.go(wait=True)")
        print("```")
        
    else:
        print("\n⚠️ 笛卡尔运动完全不可用")
        print("建议使用纯关节空间控制，或检查:")
        print("- MoveIt配置中的IK求解器设置")  
        print("- URDF模型的关节限制")
        print("- 工作空间边界设置")
        
    print("\n📊 位姿映射表（用于应用开发）:")
    for pose in successful_poses:
        print(f"{pose['name']}: 关节{pose['joints']} -> 位置{pose['position']}")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        practical_cartesian_example()
    except rospy.ROSInterruptException:
        print("示例被中断")
    except Exception as e:
        print(f"示例过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()