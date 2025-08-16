#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import moveit_commander
import sys

def display_collision_objects():
    """直接从MoveIt规划场景中获取并显示碰撞信息"""
    rospy.init_node('direct_collision_display', anonymous=True)
    
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    
    # 创建标记发布器
    marker_pub = rospy.Publisher('/moveit_collision_display', MarkerArray, queue_size=10)
    
    # 等待规划场景服务
    rospy.wait_for_service('/get_planning_scene', timeout=5.0)
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    
    print("\n=== MoveIt碰撞显示工具 ===")
    
    # 获取完整的规划场景
    try:
        req = PlanningSceneComponents()
        req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | \
                        PlanningSceneComponents.TRANSFORMS | \
                        PlanningSceneComponents.ROBOT_STATE | \
                        PlanningSceneComponents.WORLD_OBJECT_NAMES | \
                        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        
        resp = get_planning_scene(req)
        planning_scene = resp.scene
        
        print("\n当前规划场景信息:")
        print("- 机器人名称: %s" % planning_scene.name)
        print("- 世界对象数量: %d" % len(planning_scene.world.collision_objects))
        
        # 分析允许碰撞矩阵
        acm = planning_scene.allowed_collision_matrix
        print("\n允许碰撞矩阵条目数: %d" % len(acm.entry_names))
        
        # 查找与lifting_Link, lidar_Link等相关的碰撞设置
        important_links = ["lifting_Link", "lidar_Link", "box_Link", "base_link"]
        
        print("\n重要链接的碰撞设置:")
        for link in important_links:
            if link in acm.entry_names:
                idx = acm.entry_names.index(link)
                print("\n%s 的碰撞设置:" % link)
                allowed_collisions = []
                for i, other_link in enumerate(acm.entry_names):
                    if i < len(acm.entry_values[idx].enabled):
                        if acm.entry_values[idx].enabled[i]:
                            allowed_collisions.append(other_link)
                
                if allowed_collisions:
                    print("  允许与以下链接碰撞:")
                    for allowed in allowed_collisions[:5]:  # 只显示前5个
                        print("    - %s" % allowed)
                    if len(allowed_collisions) > 5:
                        print("    ... 和其他 %d 个链接" % (len(allowed_collisions) - 5))
            else:
                print("\n%s: 未在碰撞矩阵中找到" % link)
        
    except Exception as e:
        print("获取规划场景失败: %s" % str(e))
    
    # 获取机器人链接
    link_names = robot.get_link_names()
    print("\n机器人总链接数: %d" % len(link_names))
    
    # 发布碰撞标记
    rate = rospy.Rate(2)  # 2Hz
    marker_id = 0
    
    print("\n开始发布碰撞可视化标记...")
    print("在RViz中添加 MarkerArray，话题设置为: /moveit_collision_display")
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # 为重要的碰撞链接创建标记
        collision_links = [
            ("lifting_Link", Marker.CYLINDER, (0.06, 0.06, 0.4), (1.0, 0.0, 0.0, 0.8)),  # 红色圆柱
            ("lidar_Link", Marker.CYLINDER, (0.08, 0.08, 0.06), (1.0, 0.0, 0.0, 0.7)),  # 红色圆柱
            ("box_Link", Marker.CUBE, (0.3, 0.15, 0.25), (0.8, 0.0, 0.0, 0.6)),  # 深红色立方体
            ("base_link", Marker.CUBE, (0.5, 0.08, 0.35), (0.6, 0.0, 0.0, 0.4))  # 暗红色立方体
        ]
        
        for link_name, marker_type, scale, color in collision_links:
            if link_name in link_names:
                marker = Marker()
                marker.header.frame_id = link_name
                marker.header.stamp = rospy.Time.now()
                marker.ns = "collision_highlights"
                marker.id = marker_id
                marker.type = marker_type
                marker.action = Marker.ADD
                
                marker.pose.orientation.w = 1.0
                marker.scale.x = scale[0]
                marker.scale.y = scale[1]
                marker.scale.z = scale[2]
                
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = color[3]
                
                marker.lifetime = rospy.Duration(0.5)
                marker_array.markers.append(marker)
                marker_id += 1
        
        # 添加文本标记
        text_marker = Marker()
        text_marker.header.frame_id = "world_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "collision_info"
        text_marker.id = marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "红色区域: 需要避免碰撞"
        text_marker.lifetime = rospy.Duration(0.5)
        marker_array.markers.append(text_marker)
        
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        display_collision_objects()
    except rospy.ROSInterruptException:
        pass