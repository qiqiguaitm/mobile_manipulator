#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

def publish_collision_markers():
    """发布碰撞物体的可视化标记"""
    rospy.init_node('collision_marker_publisher', anonymous=True)
    
    # 创建发布器
    marker_pub = rospy.Publisher('/collision_visualization', MarkerArray, queue_size=10)
    
    rospy.loginfo("开始发布碰撞物体标记...")
    print("\n在RViz中添加MarkerArray显示:")
    print("1. 点击Add → MarkerArray")
    print("2. Topic设置为: /collision_visualization")
    print("3. 红色物体表示需要避免碰撞的部件\n")
    
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # lifting_Link - 红色立方体
        lifting_marker = Marker()
        lifting_marker.header.frame_id = "lifting_Link"
        lifting_marker.header.stamp = rospy.Time.now()
        lifting_marker.ns = "collision_objects"
        lifting_marker.id = 0
        lifting_marker.type = Marker.CUBE
        lifting_marker.action = Marker.ADD
        lifting_marker.pose.orientation.w = 1.0
        lifting_marker.scale.x = 0.06
        lifting_marker.scale.y = 0.4
        lifting_marker.scale.z = 0.06
        lifting_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.7)  # 红色，70%不透明
        lifting_marker.lifetime = rospy.Duration(0)  # 永久显示
        marker_array.markers.append(lifting_marker)
        
        # lidar_Link - 红色圆柱体
        lidar_marker = Marker()
        lidar_marker.header.frame_id = "lidar_Link"
        lidar_marker.header.stamp = rospy.Time.now()
        lidar_marker.ns = "collision_objects"
        lidar_marker.id = 1
        lidar_marker.type = Marker.CYLINDER
        lidar_marker.action = Marker.ADD
        lidar_marker.pose.orientation.w = 1.0
        lidar_marker.scale.x = 0.08  # 直径
        lidar_marker.scale.y = 0.08  # 直径
        lidar_marker.scale.z = 0.06  # 高度
        lidar_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.7)  # 红色
        lidar_marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(lidar_marker)
        
        # box_Link - 深红色立方体
        box_marker = Marker()
        box_marker.header.frame_id = "box_Link"
        box_marker.header.stamp = rospy.Time.now()
        box_marker.ns = "collision_objects"
        box_marker.id = 2
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        box_marker.pose.orientation.w = 1.0
        box_marker.scale.x = 0.3
        box_marker.scale.y = 0.15
        box_marker.scale.z = 0.25
        box_marker.color = ColorRGBA(0.8, 0.0, 0.0, 0.5)  # 深红色，50%不透明
        box_marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(box_marker)
        
        # base_link - 暗红色立方体
        base_marker = Marker()
        base_marker.header.frame_id = "base_link"
        base_marker.header.stamp = rospy.Time.now()
        base_marker.ns = "collision_objects"
        base_marker.id = 3
        base_marker.type = Marker.CUBE
        base_marker.action = Marker.ADD
        base_marker.pose.orientation.w = 1.0
        base_marker.scale.x = 0.5
        base_marker.scale.y = 0.08
        base_marker.scale.z = 0.35
        base_marker.color = ColorRGBA(0.6, 0.0, 0.0, 0.3)  # 暗红色，30%不透明
        base_marker.lifetime = rospy.Duration(0)
        marker_array.markers.append(base_marker)
        
        # 发布标记
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_collision_markers()
    except rospy.ROSInterruptException:
        pass