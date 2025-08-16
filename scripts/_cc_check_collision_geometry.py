#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def check_collision_geometry():
    """检查并可视化lifting_Link的实际位置和碰撞几何体"""
    rospy.init_node('check_collision_geometry', anonymous=True)
    
    print("\n" + "="*60)
    print("检查碰撞几何体")
    print("="*60)
    
    # TF监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 创建标记发布器
    marker_pub = rospy.Publisher('/collision_geometry_check', MarkerArray, queue_size=10)
    
    rospy.sleep(1.0)  # 等待TF
    
    # 获取lifting_Link的位置
    try:
        lifting_tf = tf_buffer.lookup_transform("world_link", "lifting_Link", rospy.Time(0))
        x = lifting_tf.transform.translation.x
        y = lifting_tf.transform.translation.y
        z = lifting_tf.transform.translation.z
        
        print("\nlifting_Link TF位置:")
        print("  X: %.3f m" % x)
        print("  Y: %.3f m" % y)
        print("  Z: %.3f m" % z)
        
        # 注意：lifting_Link的X坐标是-0.243，表示在世界坐标系的后方
        # 但在机器人本体坐标系中，它实际上在机器人前方
        
        print("\n分析:")
        print("  lifting_Link在world_link坐标系中的X=-0.243")
        print("  这意味着它在机器人前方（机器人坐标系）")
        
    except Exception as e:
        print("  ✗ 无法获取lifting_Link TF: %s" % str(e))
        return
    
    # 获取机器人基座位置
    try:
        base_tf = tf_buffer.lookup_transform("world_link", "base_link", rospy.Time(0))
        base_x = base_tf.transform.translation.x
        base_y = base_tf.transform.translation.y
        base_z = base_tf.transform.translation.z
        
        print("\nbase_link位置:")
        print("  X: %.3f m" % base_x)
        print("  Y: %.3f m" % base_y)
        print("  Z: %.3f m" % base_z)
        
    except:
        base_x = base_y = base_z = 0
    
    # 创建可视化标记
    rate = rospy.Rate(2)
    
    print("\n开始发布可视化标记...")
    print("在RViz中添加MarkerArray，话题: /collision_geometry_check")
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        
        # 1. lifting_Link的实际位置（红色立方体）
        lifting_marker = Marker()
        lifting_marker.header.frame_id = "lifting_Link"
        lifting_marker.header.stamp = rospy.Time.now()
        lifting_marker.ns = "actual_position"
        lifting_marker.id = 0
        lifting_marker.type = Marker.CUBE
        lifting_marker.action = Marker.ADD
        lifting_marker.pose.orientation.w = 1.0
        lifting_marker.scale.x = 0.06  # 碰撞几何体尺寸
        lifting_marker.scale.y = 0.4
        lifting_marker.scale.z = 0.06
        lifting_marker.color.r = 1.0
        lifting_marker.color.a = 0.8
        marker_array.markers.append(lifting_marker)
        
        # 2. 机器人前方区域（黄色透明立方体）
        front_area = Marker()
        front_area.header.frame_id = "base_link"
        front_area.header.stamp = rospy.Time.now()
        front_area.ns = "front_area"
        front_area.id = 1
        front_area.type = Marker.CUBE
        front_area.action = Marker.ADD
        front_area.pose.position.x = 0.3  # 机器人前方30cm
        front_area.pose.position.y = 0.0
        front_area.pose.position.z = 0.3
        front_area.pose.orientation.w = 1.0
        front_area.scale.x = 0.4
        front_area.scale.y = 0.6
        front_area.scale.z = 0.6
        front_area.color.r = 1.0
        front_area.color.g = 1.0
        front_area.color.b = 0.0
        front_area.color.a = 0.2
        marker_array.markers.append(front_area)
        
        # 3. 文字说明
        text_marker = Marker()
        text_marker.header.frame_id = "world_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "info"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "红色=lifting_Link\n黄色=机器人前方危险区域"
        marker_array.markers.append(text_marker)
        
        # 4. 坐标轴方向指示
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "base_link"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "direction"
        arrow_marker.id = 3
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.points = [Point(0, 0, 0.5), Point(0.5, 0, 0.5)]
        arrow_marker.scale.x = 0.05  # 箭头粗细
        arrow_marker.scale.y = 0.1   # 箭头宽度
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.8
        marker_array.markers.append(arrow_marker)
        
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        check_collision_geometry()
    except rospy.ROSInterruptException:
        pass