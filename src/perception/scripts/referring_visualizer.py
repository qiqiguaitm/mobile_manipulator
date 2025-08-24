#!/usr/bin/python3
"""
Referring检测结果可视化器
将Referring DINO检测结果转换为RViz可视化标记
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from perception.msg import GraspDetectionArray, GraspDetectionArray3D
from std_msgs.msg import ColorRGBA
import numpy as np


class ReferringVisualizer:
    """Referring检测结果可视化器"""
    
    def __init__(self):
        rospy.init_node('referring_visualizer', anonymous=True)
        
        # 参数
        self.camera_name = rospy.get_param('~camera_name', 'hand')
        self.show_bbox = rospy.get_param('~show_bbox', True)
        self.show_labels = rospy.get_param('~show_labels', True)
        self.show_points = rospy.get_param('~show_points', True)
        
        # 订阅器
        self.sub_3d = rospy.Subscriber(
            f'/perception/{self.camera_name}/referring_3d',
            GraspDetectionArray3D,
            self.callback_3d,
            queue_size=1
        )
        
        # 发布器
        self.marker_pub = rospy.Publisher(
            f'/perception/{self.camera_name}/referring_markers',
            MarkerArray,
            queue_size=1
        )
        
        rospy.loginfo(f"🎯 Referring Visualizer启动: camera={self.camera_name}")
    
    def callback_3d(self, msg):
        """处理3D检测结果"""
        try:
            marker_array = MarkerArray()
            
            # 清除旧标记
            clear_marker = Marker()
            clear_marker.header = msg.header
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            
            for idx, detection in enumerate(msg.detections):
                # 3D边界框
                if self.show_bbox and hasattr(detection, 'bbox_3d') and hasattr(detection.bbox_3d, 'corners') and len(detection.bbox_3d.corners) > 0:
                    bbox_marker = self.create_bbox_marker(
                        detection.bbox_3d,
                        msg.header,
                        idx,
                        namespace="refer_bbox"
                    )
                    marker_array.markers.append(bbox_marker)
                
                # 文字标签
                if self.show_labels:
                    label_marker = self.create_label_marker(
                        detection,
                        msg.header,
                        idx,
                        namespace="refer_labels"
                    )
                    marker_array.markers.append(label_marker)
                
                # 点云标记
                if self.show_points and len(detection.points_3d) > 0:
                    points_marker = self.create_points_marker(
                        detection.points_3d,
                        msg.header,
                        idx,
                        namespace="refer_points"
                    )
                    marker_array.markers.append(points_marker)
            
            # 发布标记
            self.marker_pub.publish(marker_array)
            
            if len(msg.detections) > 0:
                rospy.loginfo(f"📍 发布{len(msg.detections)}个Referring检测标记")
                
        except Exception as e:
            rospy.logerr(f"处理3D检测失败: {e}")
    
    def create_bbox_marker(self, bbox_3d, header, idx, namespace="refer_bbox"):
        """创建3D边界框标记"""
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = idx
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005  # 线宽
        
        # 黄色边界框（区别于绿色的抓取框）
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.8)
        
        # 添加边界框的12条边
        corners = bbox_3d.corners
        if len(corners) == 8:
            # 底面4条边
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                # 顶面4条边
                (4, 5), (5, 6), (6, 7), (7, 4),
                # 垂直4条边
                (0, 4), (1, 5), (2, 6), (3, 7)
            ]
            
            for edge in edges:
                p1 = Point(corners[edge[0]].x, corners[edge[0]].y, corners[edge[0]].z)
                p2 = Point(corners[edge[1]].x, corners[edge[1]].y, corners[edge[1]].z)
                marker.points.append(p1)
                marker.points.append(p2)
        
        marker.lifetime = rospy.Duration(1.0)
        return marker
    
    def create_label_marker(self, detection, header, idx, namespace="refer_labels"):
        """创建文字标签标记"""
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 位置在边界框中心或第一个3D点
        if hasattr(detection, 'bbox_3d') and hasattr(detection.bbox_3d, 'corners') and len(detection.bbox_3d.corners) > 0:
            # 计算边界框中心
            center_x = sum(c.x for c in detection.bbox_3d.corners) / len(detection.bbox_3d.corners)
            center_y = sum(c.y for c in detection.bbox_3d.corners) / len(detection.bbox_3d.corners)
            center_z = max(c.z for c in detection.bbox_3d.corners) + 0.1  # 在顶部显示
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = center_z
        elif len(detection.points_3d) > 0:
            marker.pose.position = detection.points_3d[0]
            marker.pose.position.z += 0.1
        
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05  # 文字大小
        
        # 黄色文字
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        
        # 显示类别和置信度
        marker.text = f"Ref: {detection.class_name} ({detection.score:.2f})"
        
        marker.lifetime = rospy.Duration(1.0)
        return marker
    
    def create_points_marker(self, points_3d, header, idx, namespace="refer_points"):
        """创建点云标记"""
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = idx
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # 点大小
        marker.scale.y = 0.01
        
        # 黄色点云
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.5)
        
        # 添加所有点
        marker.points = points_3d
        
        marker.lifetime = rospy.Duration(1.0)
        return marker
    
    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        visualizer = ReferringVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass