#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import sys
import time

def test_safe_cartesian_movement():
    """测试安全的笛卡尔空间运动"""
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_safe_cartesian', anonymous=True)
    
    # 创建move group
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("========== 安全笛卡尔空间运动测试 ==========")
    
    # 设置规划参数
    arm_group.set_planning_time(10)
    arm_group.set_max_velocity_scaling_factor(0.3)  # 降低速度
    arm_group.set_max_acceleration_scaling_factor(0.3)  # 降低加速度
    arm_group.set_goal_position_tolerance(0.01)
    arm_group.set_goal_orientation_tolerance(0.1)
    
    # 获取当前位姿
    current_pose = arm_group.get_current_pose().pose
    print(f"当前位姿: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
    print(f"当前朝向: x={current_pose.orientation.x:.4f}, y={current_pose.orientation.y:.4f}, z={current_pose.orientation.z:.4f}, w={current_pose.orientation.w:.4f}")
    
    # 示例1: 小幅度移动 - 在当前位置基础上做微调
    print("\n--- 示例1: 小幅度位置调整 ---")
    pose_goal_1 = geometry_msgs.msg.Pose()
    pose_goal_1.position.x = current_pose.position.x + 0.02  # 前进2cm
    pose_goal_1.position.y = current_pose.position.y
    pose_goal_1.position.z = current_pose.position.z + 0.01  # 上升1cm
    
    # 保持当前朝向
    pose_goal_1.orientation = current_pose.orientation
    
    print(f"目标位姿1: x={pose_goal_1.position.x:.4f}, y={pose_goal_1.position.y:.4f}, z={pose_goal_1.position.z:.4f}")
    
    arm_group.set_pose_target(pose_goal_1)
    success_1 = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    if success_1:
        print("✅ 示例1执行成功")
        time.sleep(2)
    else:
        print("❌ 示例1执行失败")
    
    # 示例2: 回到安全的中心位置
    print("\n--- 示例2: 移动到安全中心位置 ---")
    pose_goal_2 = geometry_msgs.msg.Pose()
    pose_goal_2.position.x = 0.08   # 保守的前向距离
    pose_goal_2.position.y = 0.0    # 居中
    pose_goal_2.position.z = 0.20   # 适中高度
    
    # 设置垂直向下的朝向
    quat = quaternion_from_euler(0, 1.57, 0)  # pitch=90度，垂直向下
    pose_goal_2.orientation.x = quat[0]
    pose_goal_2.orientation.y = quat[1]
    pose_goal_2.orientation.z = quat[2]
    pose_goal_2.orientation.w = quat[3]
    
    print(f"目标位姿2: x={pose_goal_2.position.x:.4f}, y={pose_goal_2.position.y:.4f}, z={pose_goal_2.position.z:.4f}")
    
    arm_group.set_pose_target(pose_goal_2)
    success_2 = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    if success_2:
        print("✅ 示例2执行成功")
        time.sleep(2)
    else:
        print("❌ 示例2执行失败")
    
    # 示例3: 左右移动
    print("\n--- 示例3: 左右移动 ---")
    pose_goal_3 = geometry_msgs.msg.Pose()
    pose_goal_3.position.x = 0.08
    pose_goal_3.position.y = 0.05   # 向右移动5cm
    pose_goal_3.position.z = 0.20
    
    # 保持垂直朝向
    pose_goal_3.orientation = pose_goal_2.orientation
    
    print(f"目标位姿3: x={pose_goal_3.position.x:.4f}, y={pose_goal_3.position.y:.4f}, z={pose_goal_3.position.z:.4f}")
    
    arm_group.set_pose_target(pose_goal_3)
    success_3 = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    
    if success_3:
        print("✅ 示例3执行成功")
        time.sleep(2)
    else:
        print("❌ 示例3执行失败")
    
    # 示例4: 笛卡尔路径规划
    print("\n--- 示例4: 笛卡尔路径规划 ---")
    waypoints = []
    
    # 当前位姿作为起点
    current_pose = arm_group.get_current_pose().pose
    waypoints.append(current_pose)
    
    # 第一个航点：向左移动
    wpose = current_pose
    wpose.position.y -= 0.03  # 向左3cm
    waypoints.append(wpose.__deepcopy__({}))
    
    # 第二个航点：向上移动
    wpose.position.z += 0.02  # 向上2cm
    waypoints.append(wpose.__deepcopy__({}))
    
    # 第三个航点：回到中心
    wpose.position.y = 0.0
    waypoints.append(wpose.__deepcopy__({}))
    
    print("计算笛卡尔路径...")
    (plan, fraction) = arm_group.compute_cartesian_path(
        waypoints,   # 航点
        0.01,        # 步长1cm
        0.0)         # 跳跃阈值
    
    print(f"路径规划完成度: {fraction*100:.1f}%")
    
    if fraction > 0.8:  # 如果80%以上的路径可规划
        print("执行笛卡尔路径...")
        success_4 = arm_group.execute(plan, wait=True)
        if success_4:
            print("✅ 示例4执行成功")
        else:
            print("❌ 示例4执行失败")
    else:
        print("❌ 路径规划不完整，跳过执行")
        success_4 = False
    
    # 总结
    print("\n========== 测试结果总结 ==========")
    results = [
        ("小幅度位置调整", success_1),
        ("安全中心位置", success_2), 
        ("左右移动", success_3),
        ("笛卡尔路径规划", success_4)
    ]
    
    passed = 0
    for name, result in results:
        status = "✅ 成功" if result else "❌ 失败"
        print(f"{name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总计: {passed}/{len(results)} 个测试通过")
    
    if passed >= 2:
        print("🎉 笛卡尔空间控制基本功能正常！")
    else:
        print("⚠️ 需要检查机械臂工作空间配置")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_safe_cartesian_movement()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except Exception as e:
        print(f"测试过程中发生错误: {str(e)}")