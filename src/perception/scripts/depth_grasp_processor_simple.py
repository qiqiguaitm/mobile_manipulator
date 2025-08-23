#!/usr/bin/python3
"""
简化的3D处理器 - 不使用cv_bridge
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from perception.msg import (GraspDetectionArray, 
                           GraspPose3D, GraspDetection3D, GraspDetectionArray3D,
                           TouchingPoint3D)
from geometry_msgs.msg import Point
import struct

class SimpleDepthProcessor:
    def __init__(self):
        rospy.init_node('simple_depth_processor')
        
        # Camera intrinsics
        self.fx = 615.0  # Default value
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0
        self.depth_scale = 0.001  # mm to meters
        
        self.latest_depth = None
        self.latest_grasps = None
        
        # Publisher
        self.pub = rospy.Publisher('/perception/hand/grasps_3d', 
                                  GraspDetectionArray3D, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/camera/hand/depth/camera_info', CameraInfo, self.camera_cb)
        rospy.Subscriber('/camera/hand/aligned_depth_to_color/image_raw', Image, self.depth_cb)
        rospy.Subscriber('/perception/hand/grasps', GraspDetectionArray, self.grasp_cb)
        
        rospy.loginfo("Simple depth processor started")
    
    def camera_cb(self, msg):
        """更新相机内参"""
        if msg.K[0] > 0:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            rospy.loginfo_once(f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}")
    
    def depth_cb(self, msg):
        """保存最新深度图"""
        self.latest_depth = msg
    
    def grasp_cb(self, msg):
        """处理抓取检测"""
        self.latest_grasps = msg
        
        if self.latest_depth is None:
            rospy.logwarn_once("No depth image yet")
            return
        
        # 解析深度图像（不用cv_bridge）
        depth_array = self.parse_depth_image(self.latest_depth)
        if depth_array is None:
            return
        
        # 创建3D消息
        msg3d = GraspDetectionArray3D()
        msg3d.header = msg.header
        msg3d.header.frame_id = "camera/hand_depth_optical_frame"
        
        # 转换每个检测
        for det in msg.detections:
            det3d = GraspDetection3D()
            
            # 边界框中心
            if len(det.bbox) >= 4:
                cx = (det.bbox[0] + det.bbox[2]) / 2
                cy = (det.bbox[1] + det.bbox[3]) / 2
                
                # 获取深度
                ix = int(cx)
                iy = int(cy)
                if 0 <= ix < depth_array.shape[1] and 0 <= iy < depth_array.shape[0]:
                    depth = depth_array[iy, ix] * self.depth_scale
                    
                    if depth > 0.1 and depth < 5.0:  # 有效深度范围
                        # 计算3D坐标
                        x = (cx - self.cx) * depth / self.fx
                        y = (cy - self.cy) * depth / self.fy
                        
                        det3d.center_3d = Point(x=x, y=y, z=depth)
                        
                        # 估算3D尺寸
                        width_px = det.bbox[2] - det.bbox[0]
                        height_px = det.bbox[3] - det.bbox[1]
                        det3d.dimensions = Point(
                            x=width_px * depth / self.fx,
                            y=height_px * depth / self.fy,
                            z=0.1  # 默认深度
                        )
            
            det3d.score = det.score
            det3d.bbox_2d = det.bbox
            
            # 转换抓取姿态
            for grasp in det.grasp_poses:
                g3d = GraspPose3D()
                
                # 获取抓取中心深度
                ix = int(grasp.x)
                iy = int(grasp.y)
                if 0 <= ix < depth_array.shape[1] and 0 <= iy < depth_array.shape[0]:
                    depth = depth_array[iy, ix] * self.depth_scale
                    if depth > 0.1 and depth < 5.0:
                        g3d.position = Point(
                            x=(grasp.x - self.cx) * depth / self.fx,
                            y=(grasp.y - self.cy) * depth / self.fy,
                            z=depth
                        )
                        g3d.width = grasp.width * depth / self.fx
                        # orientation 可以从theta角度计算，这里简化处理
                        # g3d.orientation = ... 
                        g3d.confidence = grasp.confidence
                        det3d.grasp_poses_3d.append(g3d)
            
            msg3d.detections.append(det3d)
        
        # 发布
        if len(msg3d.detections) > 0:
            self.pub.publish(msg3d)
            rospy.loginfo(f"Published {len(msg3d.detections)} 3D detections")
    
    def parse_depth_image(self, msg):
        """手动解析深度图像数据"""
        try:
            if msg.encoding == '16UC1':
                # 16位无符号整数
                depth_array = np.frombuffer(msg.data, dtype=np.uint16)
                depth_array = depth_array.reshape((msg.height, msg.width))
                return depth_array
            else:
                rospy.logwarn(f"Unsupported depth encoding: {msg.encoding}")
                return None
        except Exception as e:
            rospy.logerr(f"Failed to parse depth: {e}")
            return None

if __name__ == '__main__':
    node = SimpleDepthProcessor()
    rospy.spin()