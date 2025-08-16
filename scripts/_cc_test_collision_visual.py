#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf2_geometry_msgs

def create_collision_visualization():
    """创建碰撞元素的可视化标记"""
    rospy.init_node('collision_visualization', anonymous=True)
    
    # 发布器
    marker_pub = rospy.Publisher('/collision_visualization', MarkerArray, queue_size=10)
    
    # TF监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10)
    
    print("发布碰撞可视化标记...")
    print("请在RViz中添加MarkerArray显示，topic: /collision_visualization")
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # 1. lifting_Link的碰撞体 (红色圆柱)
        lifting_marker = Marker()
        lifting_marker.header.frame_id = "lifting_Link"
        lifting_marker.header.stamp = rospy.Time.now()
        lifting_marker.ns = "collision"
        lifting_marker.id = 1
        lifting_marker.type = Marker.CYLINDER
        lifting_marker.action = Marker.ADD
        
        # 设置位置（根据URDF中的碰撞体定义）
        lifting_marker.pose.position.x = 0
        lifting_marker.pose.position.y = 0
        lifting_marker.pose.position.z = 0.28  # 高度的一半
        lifting_marker.pose.orientation.w = 1.0
        
        # 设置尺寸
        lifting_marker.scale.x = 0.06  # 直径
        lifting_marker.scale.y = 0.06
        lifting_marker.scale.z = 0.56  # 高度
        
        # 设置颜色（红色，半透明）
        lifting_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        
        marker_array.markers.append(lifting_marker)
        
        # 2. lidar_Link的碰撞体 (黄色方块)
        lidar_marker = Marker()
        lidar_marker.header.frame_id = "lidar_Link"
        lidar_marker.header.stamp = rospy.Time.now()
        lidar_marker.ns = "collision"
        lidar_marker.id = 2
        lidar_marker.type = Marker.CUBE
        lidar_marker.action = Marker.ADD
        
        lidar_marker.pose.position.x = 0
        lidar_marker.pose.position.y = 0
        lidar_marker.pose.position.z = 0.045
        lidar_marker.pose.orientation.w = 1.0
        
        lidar_marker.scale.x = 0.18
        lidar_marker.scale.y = 0.14
        lidar_marker.scale.z = 0.09
        
        lidar_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.5)
        
        marker_array.markers.append(lidar_marker)
        
        # 3. box_Link的碰撞体 (蓝色方块)
        box_marker = Marker()
        box_marker.header.frame_id = "box_Link"
        box_marker.header.stamp = rospy.Time.now()
        box_marker.ns = "collision"
        box_marker.id = 3
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        
        box_marker.pose.position.x = 0
        box_marker.pose.position.y = 0
        box_marker.pose.position.z = 0.03
        box_marker.pose.orientation.w = 1.0
        
        box_marker.scale.x = 0.4
        box_marker.scale.y = 0.24
        box_marker.scale.z = 0.06
        
        box_marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        
        marker_array.markers.append(box_marker)
        
        # 4. 添加文本标签
        for i, (frame, text) in enumerate([
            ("lifting_Link", "LIFTING"),
            ("lidar_Link", "LIDAR"),
            ("box_Link", "BOX")
        ]):
            text_marker = Marker()
            text_marker.header.frame_id = frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "labels"
            text_marker.id = i + 10
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.z = 0.1
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.05
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text_marker.text = text
            
            marker_array.markers.append(text_marker)
        
        # 发布标记
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        create_collision_visualization()
    except rospy.ROSInterruptException:
        pass