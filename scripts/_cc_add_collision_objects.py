#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

def add_collision_objects():
    """手动添加碰撞对象到MoveIt规划场景"""
    rospy.init_node('add_collision_objects', anonymous=True)
    
    print("\n" + "="*60)
    print("添加额外的碰撞对象")
    print("="*60)
    
    # 初始化
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # 等待场景接口初始化
    
    # 清除可能存在的旧对象
    scene.remove_world_object("lifting_collision_box")
    scene.remove_world_object("lidar_collision_cylinder")
    rospy.sleep(0.5)
    
    print("\n添加碰撞对象到规划场景...")
    
    # 1. 在lifting_Link位置添加一个更大的碰撞盒
    # lifting_Link实际上在机器人前方
    lifting_pose = PoseStamped()
    lifting_pose.header.frame_id = "base_link"
    lifting_pose.pose.position.x = 0.25  # 机器人前方25cm
    lifting_pose.pose.position.y = 0.0
    lifting_pose.pose.position.z = 0.3   # 高度30cm
    lifting_pose.pose.orientation.w = 1.0
    
    # 创建一个较大的碰撞盒
    scene.add_box("lifting_collision_box", 
                  lifting_pose, 
                  size=(0.1, 0.5, 0.8))  # 10cm x 50cm x 80cm
    
    print("  ✓ 添加了lifting区域碰撞盒")
    print("    位置: x=%.2f, y=%.2f, z=%.2f" % 
          (lifting_pose.pose.position.x, lifting_pose.pose.position.y, lifting_pose.pose.position.z))
    print("    尺寸: 10cm x 50cm x 80cm")
    
    # 2. 在lidar位置添加碰撞圆柱
    lidar_pose = PoseStamped()
    lidar_pose.header.frame_id = "base_link"
    lidar_pose.pose.position.x = 0.15   # 机器人前方15cm
    lidar_pose.pose.position.y = 0.0
    lidar_pose.pose.position.z = 0.05   # 地面上方5cm
    lidar_pose.pose.orientation.w = 1.0
    
    scene.add_cylinder("lidar_collision_cylinder",
                      lidar_pose,
                      height=0.1,
                      radius=0.1)
    
    print("  ✓ 添加了lidar区域碰撞圆柱")
    print("    位置: x=%.2f, y=%.2f, z=%.2f" % 
          (lidar_pose.pose.position.x, lidar_pose.pose.position.y, lidar_pose.pose.position.z))
    print("    尺寸: 半径10cm, 高10cm")
    
    # 等待对象被添加
    rospy.sleep(1.0)
    
    # 验证对象是否添加成功
    known_objects = scene.get_known_object_names()
    print("\n当前场景中的碰撞对象:")
    for obj in known_objects:
        print("  - %s" % obj)
    
    print("\n" + "="*60)
    print("碰撞对象添加完成!")
    print("现在在RViz中测试：")
    print("1. 尝试将机械臂目标设置到机器人前方")
    print("2. 应该无法规划路径（碰撞）")
    print("3. 碰撞对象会在Scene Objects中显示")
    print("="*60 + "\n")
    
    # 保持节点运行
    print("节点保持运行中... (Ctrl+C退出)")
    rospy.spin()

if __name__ == '__main__':
    try:
        add_collision_objects()
    except rospy.ROSInterruptException:
        # 退出时清理
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_world_object("lifting_collision_box")
        scene.remove_world_object("lidar_collision_cylinder")
        print("\n已清理碰撞对象")