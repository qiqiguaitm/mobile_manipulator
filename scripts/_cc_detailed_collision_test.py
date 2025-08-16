#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import tf2_ros
import numpy as np

def detailed_collision_test():
    """详细的碰撞检测测试"""
    
    rospy.init_node('detailed_collision_test', anonymous=True)
    
    print("\n" + "="*80)
    print("详细碰撞检测测试")
    print("="*80)
    
    # 初始化
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("piper")
    
    # TF监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.sleep(2.0)
    
    # 1. 检查lifting_Link的位置
    print("\n1. 检查lifting_Link的TF位置:")
    try:
        # 尝试不同的参考坐标系
        for ref_frame in ["world", "base_link", "arm_base"]:
            try:
                trans = tf_buffer.lookup_transform(ref_frame, "lifting_Link", rospy.Time(0))
                print(f"  {ref_frame} -> lifting_Link:")
                print(f"    位置: x={trans.transform.translation.x:.3f}, "
                      f"y={trans.transform.translation.y:.3f}, "
                      f"z={trans.transform.translation.z:.3f}")
            except:
                print(f"  {ref_frame} -> lifting_Link: 无法获取")
    except Exception as e:
        print(f"  错误: {e}")
    
    # 2. 检查碰撞几何体
    print("\n2. 检查碰撞几何体信息:")
    
    # 获取规划场景
    get_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    req = PlanningSceneComponents()
    req.components = (
        PlanningSceneComponents.TRANSFORMS |
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
        PlanningSceneComponents.LINK_PADDING_AND_SCALING
    )
    
    try:
        resp = get_scene_srv(req)
        scene_msg = resp.scene
        
        # 检查变换
        print("\n  场景中的变换数量:", len(scene_msg.robot_state.multi_dof_joint_state.transforms))
        
        # 检查链接填充
        print("\n  链接填充信息:")
        for pad in scene_msg.link_padding:
            if "lifting" in pad.link_name or "link" in pad.link_name:
                print(f"    {pad.link_name}: padding={pad.padding}")
                
    except Exception as e:
        print(f"  获取场景失败: {e}")
    
    # 3. 测试具体的碰撞情况
    print("\n3. 测试特定位置的碰撞:")
    
    # 获取当前位置
    current_pose = group.get_current_pose().pose
    print(f"\n  当前末端位置:")
    print(f"    x={current_pose.position.x:.3f}")
    print(f"    y={current_pose.position.y:.3f}")
    print(f"    z={current_pose.position.z:.3f}")
    
    # 测试向左90度
    print("\n  测试: joint1=-90度")
    test_joints = [-1.57, 0, 0, 0, 0, 0]
    group.set_joint_value_target(test_joints)
    
    # 计算这个姿态下的末端位置
    group.set_joint_value_target(test_joints)
    plan = group.plan()
    
    if plan[0]:  # 如果规划成功
        # 获取规划的最后一个点
        if plan[1].joint_trajectory.points:
            last_point = plan[1].joint_trajectory.points[-1]
            print("  ✗ 能够规划到该位置（不应该成功）")
            
            # 计算末端位置
            robot_state = robot.get_current_state()
            robot_state.joint_state.position = list(last_point.positions)
            group.set_start_state(robot_state)
            end_pose = group.get_current_pose().pose
            print(f"  目标末端位置:")
            print(f"    x={end_pose.position.x:.3f}")
            print(f"    y={end_pose.position.y:.3f}")
            print(f"    z={end_pose.position.z:.3f}")
    else:
        print("  ✓ 无法规划到该位置（正确）")
    
    # 4. 检查机器人链接
    print("\n4. 机器人链接列表:")
    links = robot.get_link_names()
    critical_links = ["lifting_Link", "lidar_Link", "arm_base", "link1", "link2", "link3"]
    for link in critical_links:
        if link in links:
            print(f"  ✓ {link}")
        else:
            print(f"  ✗ {link} (缺失)")
    
    # 5. 直接检查碰撞
    print("\n5. 使用场景接口检查碰撞对象:")
    known_objects = scene.get_known_object_names()
    if known_objects:
        print(f"  场景对象: {known_objects}")
    else:
        print("  场景中没有额外对象")
    
    # 获取附着对象
    attached_objects = scene.get_attached_objects()
    if attached_objects:
        print(f"  附着对象: {list(attached_objects.keys())}")
    else:
        print("  没有附着对象")
    
    print("\n" + "="*80)
    print("诊断完成")
    print("="*80)

if __name__ == '__main__':
    try:
        detailed_collision_test()
    except rospy.ROSInterruptException:
        pass