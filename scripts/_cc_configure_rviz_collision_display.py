#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import tf

class CollisionObjectVisualizer:
    """在RViz中高亮显示需要避免碰撞的要素"""
    
    def __init__(self):
        rospy.init_node('collision_object_visualizer', anonymous=True)
        
        # 发布碰撞物体标记
        self.marker_pub = rospy.Publisher('/collision_objects_markers', MarkerArray, queue_size=10)
        self.tf_listener = tf.TransformListener()
        
        # 需要避免碰撞的关键部件
        self.collision_objects = [
            {
                'name': 'lifting_Link',
                'type': 'box',
                'size': [0.06, 0.4, 0.06],  # 使用简化碰撞网格的尺寸
                'frame': 'lifting_Link',
                'color': [1.0, 0.0, 0.0, 0.7]  # 红色，70%透明度
            },
            {
                'name': 'lidar_Link',
                'type': 'cylinder',
                'size': [0.04, 0.06],  # 半径和高度
                'frame': 'lidar_Link',
                'color': [1.0, 0.0, 0.0, 0.7]  # 红色
            },
            {
                'name': 'box_Link',
                'type': 'box',
                'size': [0.3, 0.15, 0.25],
                'frame': 'box_Link',
                'color': [0.8, 0.0, 0.0, 0.5]  # 深红色，50%透明度
            },
            {
                'name': 'base_link',
                'type': 'box',
                'size': [0.5, 0.08, 0.35],
                'frame': 'base_link',
                'color': [0.6, 0.0, 0.0, 0.3]  # 暗红色，30%透明度
            }
        ]
        
        rospy.loginfo("碰撞物体可视化器已启动")
        rospy.loginfo("在RViz中添加MarkerArray显示，topic: /collision_objects_markers")
        
    def create_box_marker(self, obj, marker_id):
        """创建立方体标记"""
        marker = Marker()
        marker.header.frame_id = obj['frame']
        marker.header.stamp = rospy.Time.now()
        marker.ns = "collision_objects"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 设置位置（相对于frame原点）
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸
        marker.scale.x = obj['size'][0]
        marker.scale.y = obj['size'][1]
        marker.scale.z = obj['size'][2]
        
        # 设置颜色
        marker.color.r = obj['color'][0]
        marker.color.g = obj['color'][1]
        marker.color.b = obj['color'][2]
        marker.color.a = obj['color'][3]
        
        marker.lifetime = rospy.Duration(0.5)  # 0.5秒后消失，需要持续发布
        
        return marker
    
    def create_cylinder_marker(self, obj, marker_id):
        """创建圆柱体标记"""
        marker = Marker()
        marker.header.frame_id = obj['frame']
        marker.header.stamp = rospy.Time.now()
        marker.ns = "collision_objects"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        # 设置尺寸 (直径和高度)
        marker.scale.x = obj['size'][0] * 2  # 直径
        marker.scale.y = obj['size'][0] * 2  # 直径
        marker.scale.z = obj['size'][1]      # 高度
        
        # 设置颜色
        marker.color.r = obj['color'][0]
        marker.color.g = obj['color'][1]
        marker.color.b = obj['color'][2]
        marker.color.a = obj['color'][3]
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    def publish_markers(self):
        """发布所有碰撞物体标记"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.collision_objects):
            try:
                # 检查frame是否存在
                self.tf_listener.waitForTransform('world_link', obj['frame'], 
                                                rospy.Time(0), rospy.Duration(0.1))
                
                # 创建标记
                if obj['type'] == 'box':
                    marker = self.create_box_marker(obj, i)
                elif obj['type'] == 'cylinder':
                    marker = self.create_cylinder_marker(obj, i)
                
                marker_array.markers.append(marker)
                
            except tf.Exception:
                pass  # Frame不存在，跳过
        
        # 发布标记数组
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        print("\n=== 碰撞物体可视化配置 ===")
        print("\n在RViz中配置显示：")
        print("1. 点击'Add'按钮")
        print("2. 选择'MarkerArray'")
        print("3. 设置Topic为: /collision_objects_markers")
        print("4. 点击OK")
        print("\n红色透明物体表示需要避免碰撞的部件：")
        print("- lifting_Link (立柱) - 红色")
        print("- lidar_Link (激光雷达) - 红色")
        print("- box_Link (底座盒子) - 深红色")
        print("- base_link (底盘) - 暗红色")
        print("\n透明度表示碰撞风险等级")
        
        while not rospy.is_shutdown():
            self.publish_markers()
            rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = CollisionObjectVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass