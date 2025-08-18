#!/usr/bin/env python3
"""
安全可视化发布器 - 在RViz中显示安全状态
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point
import colorsys

class SafetyVisualization:
    def __init__(self):
        rospy.init_node('safety_visualization')
        
        # 状态
        self.current_safety_level = "NORMAL"
        self.min_distance = 1.0
        
        # 发布器
        self.marker_pub = rospy.Publisher('/safety_status_markers', MarkerArray, queue_size=1)
        
        # 订阅器
        self.safety_level_sub = rospy.Subscriber(
            '/safety_level', String, self.safety_level_callback
        )
        self.distance_sub = rospy.Subscriber(
            '/collision_distances', Float32MultiArray, self.distance_callback
        )
        
        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_markers)
        
        rospy.loginfo("Safety visualization started")
    
    def safety_level_callback(self, msg):
        """更新安全等级"""
        # 解析消息格式 "LEVEL:distance"
        parts = msg.data.split(':')
        if parts:
            self.current_safety_level = parts[0]
    
    def distance_callback(self, msg):
        """更新最小距离"""
        if len(msg.data) > 0:
            self.min_distance = msg.data[0]
    
    def get_color_for_level(self, level):
        """根据安全等级返回颜色"""
        colors = {
            "NORMAL": (0.0, 1.0, 0.0),      # 绿色
            "CAUTION": (1.0, 1.0, 0.0),     # 黄色
            "WARNING": (1.0, 0.5, 0.0),     # 橙色
            "CRITICAL": (1.0, 0.2, 0.0),    # 深橙
            "EMERGENCY": (1.0, 0.0, 0.0),   # 红色
        }
        return colors.get(level, (0.5, 0.5, 0.5))
    
    def publish_markers(self, event):
        """发布可视化标记"""
        marker_array = MarkerArray()
        
        # 1. 安全状态文本
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "safety_status"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0
        text_marker.pose.position.y = 0
        text_marker.pose.position.z = 1.5
        text_marker.scale.z = 0.1
        text_marker.text = f"Safety: {self.current_safety_level}\nMin Dist: {self.min_distance:.3f}m"
        
        r, g, b = self.get_color_for_level(self.current_safety_level)
        text_marker.color.r = r
        text_marker.color.g = g
        text_marker.color.b = b
        text_marker.color.a = 1.0
        
        marker_array.markers.append(text_marker)
        
        # 2. 安全边界球体
        if self.min_distance < 0.1:  # 只在接近时显示
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "base_link"
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "safety_boundary"
            sphere_marker.id = 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = 0
            sphere_marker.pose.position.y = 0
            sphere_marker.pose.position.z = 0.5
            sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = self.min_distance * 2
            
            # 半透明
            sphere_marker.color.r = r
            sphere_marker.color.g = g
            sphere_marker.color.b = b
            sphere_marker.color.a = 0.3
            
            marker_array.markers.append(sphere_marker)
        
        # 3. 状态指示器（类似交通灯）
        indicator_marker = Marker()
        indicator_marker.header.frame_id = "base_link"
        indicator_marker.header.stamp = rospy.Time.now()
        indicator_marker.ns = "safety_indicator"
        indicator_marker.id = 2
        indicator_marker.type = Marker.CYLINDER
        indicator_marker.action = Marker.ADD
        indicator_marker.pose.position.x = 0.5
        indicator_marker.pose.position.y = 0.5
        indicator_marker.pose.position.z = 1.0
        indicator_marker.scale.x = indicator_marker.scale.y = 0.1
        indicator_marker.scale.z = 0.05
        
        indicator_marker.color.r = r
        indicator_marker.color.g = g
        indicator_marker.color.b = b
        indicator_marker.color.a = 1.0
        
        marker_array.markers.append(indicator_marker)
        
        self.marker_pub.publish(marker_array)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        viz = SafetyVisualization()
        viz.run()
    except rospy.ROSInterruptException:
        pass