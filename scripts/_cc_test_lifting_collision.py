#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import sys

def test_lifting_collision():
    """测试机械臂与lifting_Link的碰撞检测"""
    rospy.init_node('test_lifting_collision', anonymous=True)
    
    print("\n" + "="*60)
    print("测试lifting_Link碰撞检测")
    print("="*60)
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("piper")
    
    # 获取状态有效性检查服务
    rospy.wait_for_service('/check_state_validity')
    check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    
    # 获取lifting_Link的位置
    print("\n1. 检查lifting_Link的TF位置...")
    import tf2_ros
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    
    try:
        lifting_tf = tf_buffer.lookup_transform("world_link", "lifting_Link", rospy.Time(0))
        print("   lifting_Link位置: x=%.3f, y=%.3f, z=%.3f" % 
              (lifting_tf.transform.translation.x,
               lifting_tf.transform.translation.y,
               lifting_tf.transform.translation.z))
    except:
        print("   ✗ 无法获取lifting_Link的TF")
    
    # 测试1: 创建一个会碰撞的关节配置
    print("\n2. 测试关节配置的碰撞...")
    
    # 设置一个伸向前方的姿态（应该与lifting_Link碰撞）
    test_joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]  # 向前伸展
    
    # 创建机器人状态
    robot_state = RobotState()
    robot_state.joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    robot_state.joint_state.position = test_joints
    
    # 检查状态有效性
    req = GetStateValidityRequest()
    req.robot_state = robot_state
    req.group_name = "piper"
    
    try:
        resp = check_state_validity(req)
        if resp.valid:
            print("   ⚠ 状态被判定为有效 - 碰撞检测可能未工作!")
            print("   碰撞成本: %.3f" % resp.cost_sources.cost_density[0].cost if resp.cost_sources.cost_density else 0)
        else:
            print("   ✓ 状态无效 - 检测到碰撞!")
            if resp.contacts:
                print("   碰撞接触:")
                for contact in resp.contacts[:3]:  # 只显示前3个
                    print("     - %s <-> %s" % (contact.contact_body_1, contact.contact_body_2))
    except Exception as e:
        print("   ✗ 检查失败: %s" % str(e))
    
    # 测试2: 尝试规划到lifting_Link后面
    print("\n3. 测试规划到lifting_Link后面...")
    
    target_pose = Pose()
    target_pose.position.x = 0.35  # lifting_Link后面
    target_pose.position.y = 0.0
    target_pose.position.z = 0.4
    target_pose.orientation.w = 1.0
    
    print("   目标位置: x=%.2f, y=%.2f, z=%.2f" % 
          (target_pose.position.x, target_pose.position.y, target_pose.position.z))
    
    move_group.set_pose_target(target_pose)
    
    # 规划
    success, plan, planning_time, error_code = move_group.plan()
    
    if success:
        print("   ⚠ 规划成功 - 这不应该发生!")
        print("   规划时间: %.3f秒" % planning_time)
        print("   路径点数: %d" % len(plan.joint_trajectory.points))
    else:
        print("   ✓ 规划失败 - 碰撞检测正常!")
        print("   错误代码: %s" % error_code)
    
    # 获取ACM状态
    print("\n4. 检查允许碰撞矩阵(ACM)...")
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    req = PlanningSceneComponents()
    req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    
    try:
        resp = get_planning_scene(req)
        acm = resp.scene.allowed_collision_matrix
        
        # 检查lifting_Link与机械臂链接的碰撞设置
        if "lifting_Link" in acm.entry_names:
            lifting_idx = acm.entry_names.index("lifting_Link")
            
            critical_links = ["link4", "link5", "link6", "gripper_base"]
            for link in critical_links:
                if link in acm.entry_names:
                    link_idx = acm.entry_names.index(link)
                    if link_idx < len(acm.entry_values[lifting_idx].enabled):
                        allowed = acm.entry_values[lifting_idx].enabled[link_idx]
                        if allowed:
                            print("   ✗ lifting_Link <-> %s: 允许碰撞!" % link)
                        else:
                            print("   ✓ lifting_Link <-> %s: 碰撞检测启用" % link)
    except:
        print("   ✗ 无法获取ACM")
    
    print("\n" + "="*60)
    print("诊断结果:")
    print("如果机械臂仍能规划到lifting_Link后面，可能的原因：")
    print("1. MoveIt使用了缓存的SRDF配置")
    print("2. 碰撞几何体尺寸太小")
    print("3. 需要完全重启ROS系统")
    print("="*60 + "\n")
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_lifting_collision()
    except rospy.ROSInterruptException:
        pass