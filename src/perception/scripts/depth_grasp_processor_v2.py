#!/usr/bin/env python3
"""
改进的3D抓取处理器 - 使用depth_projector_core统一双TF链投影
"""

import rospy
import numpy as np
import sys
import os
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from perception.msg import (GraspDetectionArray, 
                           GraspPose3D, GraspDetection3D, GraspDetectionArray3D,
                           TouchingPoint3D)
from geometry_msgs.msg import Point
import struct

# 添加depth_projector_core路径
sys.path.append(os.path.dirname(__file__))
from depth_projector_core import CameraDepthProjector

class DualProjectionGraspProcessor:
    def __init__(self):
        rospy.init_node('depth_grasp_processor_v2', anonymous=True)
        
        # 参数获取
        self.projection_method = rospy.get_param('~projection_method', 'calibration')
        self.config_path = rospy.get_param('~config_path', '/home/agilex/MobileManipulator/src/robot_drivers/camera_driver/config')
        self.enable_color = rospy.get_param('~enable_color', True)
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/hand/depth/image_rect_raw')
        self.color_topic = rospy.get_param('~color_topic', '/camera/hand/color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/hand/depth/camera_info')
        
        # 过滤参数
        self.min_depth = rospy.get_param('~min_depth', 200)      # 最小深度mm
        self.max_depth = rospy.get_param('~max_depth', 2000)     # 最大深度mm
        self.min_z_height = rospy.get_param('~min_z_height', -0.3) # 最小Z高度m (base_link在底盘上表面)
        self.max_z_height = rospy.get_param('~max_z_height', 1.5)  # 最大Z高度m
        self.max_xy_dist = rospy.get_param('~max_xy_dist', 1.5)  # 最大XY距离m
        self.min_score = rospy.get_param('~min_score', 0.3)      # 最小置信度
        
        rospy.loginfo(f"🚀 启动3D抓取处理器 v2")
        rospy.loginfo(f"  投影方法: {self.projection_method}")
        rospy.loginfo(f"  彩色模式: {self.enable_color}")
        rospy.loginfo(f"  深度范围: {self.min_depth}-{self.max_depth}mm")
        rospy.loginfo(f"  Z高度范围: {self.min_z_height}-{self.max_z_height}m")
        rospy.loginfo(f"  最小置信度: {self.min_score}")
        
        # 初始化深度投影器
        try:
            self.depth_projector = CameraDepthProjector(
                'hand_camera', 
                self.projection_method, 
                self.config_path
            )
            rospy.loginfo("✅ 深度投影器初始化成功")
            rospy.loginfo(f"  相机内参: fx={self.depth_projector.K[0,0]:.1f}, fy={self.depth_projector.K[1,1]:.1f}")
            rospy.loginfo(f"  相机坐标系: {self.depth_projector.camera_frame}")
        except Exception as e:
            rospy.logerr(f"❌ 深度投影器初始化失败: {e}")
            return
        
        # 数据存储
        self.latest_depth = None
        self.latest_color = None
        self.latest_grasps_2d = None
        self.depth_scale = 1000.0  # 深度值缩放
        
        # 发布器
        self.pub_grasps_3d = rospy.Publisher('/perception/hand/grasps_3d', 
                                            GraspDetectionArray3D, queue_size=1)
        self.pub_pointcloud = rospy.Publisher('/perception/hand/grasp_points', 
                                            PointCloud2, queue_size=1)
        
        # 订阅器
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        if self.enable_color:
            rospy.Subscriber(self.color_topic, Image, self.color_callback)
        rospy.Subscriber('/perception/hand/grasps', GraspDetectionArray, self.grasp_2d_callback)
        
        rospy.loginfo("🎯 准备处理3D抓取检测...")
        
    def depth_callback(self, msg):
        """深度图回调"""
        self.latest_depth = self._image_msg_to_array(msg)
        
    def color_callback(self, msg):
        """彩色图回调"""
        if self.enable_color:
            self.latest_color = self._image_msg_to_array(msg, is_color=True)
    
    def grasp_2d_callback(self, msg):
        """2D抓取检测回调"""
        self.latest_grasps_2d = msg
        self._process_3d_grasps()
    
    def _image_msg_to_array(self, msg, is_color=False):
        """ROS Image消息转numpy数组"""
        try:
            if is_color:
                # RGB8格式
                data = np.frombuffer(msg.data, dtype=np.uint8)
                image = data.reshape(msg.height, msg.width, 3)
                return image
            else:
                # 深度图 16UC1格式
                data = np.frombuffer(msg.data, dtype=np.uint16)
                image = data.reshape(msg.height, msg.width)
                return image.astype(np.float32)
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
            return None
    
    def _process_3d_grasps(self):
        """处理3D抓取检测"""
        if self.latest_depth is None or self.latest_grasps_2d is None:
            return
            
        if len(self.latest_grasps_2d.detections) == 0:
            return
        
        try:
            # 提取2D抓取点
            grasp_points_2d = []
            colors = []
            valid_detection_indices = []
            
            for det_idx, detection in enumerate(self.latest_grasps_2d.detections):
                # 过滤低置信度检测
                if detection.score < self.min_score:
                    rospy.logdebug(f"跳过检测{det_idx}: 置信度{detection.score:.2f} < {self.min_score}")
                    continue
                detection_has_valid = False
                for grasp in detection.grasp_poses:
                    # 抓取中心点
                    u = int(grasp.x)
                    v = int(grasp.y)
                    
                    if 0 <= u < self.latest_depth.shape[1] and 0 <= v < self.latest_depth.shape[0]:
                        depth_value = self.latest_depth[v, u]
                        
                        # 如果深度为0，尝试使用邻域深度
                        if depth_value == 0:
                            # 搜索邻域有效深度
                            for radius in [5, 10, 15, 20]:
                                u_min = max(0, u-radius)
                                u_max = min(self.latest_depth.shape[1], u+radius+1)
                                v_min = max(0, v-radius)
                                v_max = min(self.latest_depth.shape[0], v+radius+1)
                                
                                neighborhood = self.latest_depth[v_min:v_max, u_min:u_max]
                                valid_neighbor = neighborhood[neighborhood > 0]
                                
                                if len(valid_neighbor) > 0:
                                    # 使用中位数作为估计深度
                                    depth_value = np.median(valid_neighbor)
                                    rospy.logdebug(f"使用邻域深度(半径{radius}): {depth_value:.0f}mm")
                                    break
                        
                        # 深度范围过滤
                        if self.min_depth <= depth_value <= self.max_depth:
                            grasp_points_2d.append([u, v, depth_value])
                            detection_has_valid = True
                            rospy.loginfo(f"有效抓取点: u={u}, v={v}, depth={depth_value:.0f}mm")
                            
                            # 获取颜色信息
                            if self.enable_color and self.latest_color is not None:
                                color = self.latest_color[v, u]
                                colors.append(color)
                            else:
                                colors.append([255, 255, 255])  # 默认白色
                
                if detection_has_valid:
                    valid_detection_indices.append(det_idx)
            
            if not grasp_points_2d:
                rospy.logwarn("没有找到有效的3D抓取点")
                return
            
            # 使用depth_projector_core进行3D投影
            grasp_points_2d = np.array(grasp_points_2d)
            colors = np.array(colors) if colors else None
            
            # 投影到3D空间
            points_3d = self.depth_projector.project_depth_to_3d(
                grasp_points_2d[:, :2],  # u, v坐标
                grasp_points_2d[:, 2] / self.depth_scale  # 深度值(米)
            )
            
            if points_3d is None or len(points_3d) == 0:
                rospy.logwarn("3D投影失败")
                return
            
            # 进一步过滤3D点
            filtered_points = []
            filtered_colors = []
            for i, point_3d in enumerate(points_3d):
                x, y, z = point_3d
                
                # Z高度过滤
                if z < self.min_z_height or z > self.max_z_height:
                    rospy.logwarn(f"过滤点: Z={z:.3f}m 超出范围[{self.min_z_height}, {self.max_z_height}]")
                    continue
                
                # XY距离过滤
                xy_dist = np.sqrt(x**2 + y**2)
                if xy_dist > self.max_xy_dist:
                    rospy.logdebug(f"过滤点: XY距离={xy_dist:.3f}m > {self.max_xy_dist}m")
                    continue
                
                filtered_points.append(point_3d)
                if len(colors) > 0:
                    filtered_colors.append(colors[i] if i < len(colors) else [255, 255, 255])
            
            if not filtered_points:
                rospy.logwarn("所有3D点都被过滤")
                return
            
            points_3d = np.array(filtered_points)
            if len(filtered_colors) > 0:
                colors = np.array(filtered_colors)
            else:
                colors = None
            
            # 创建3D抓取检测消息
            grasps_3d_msg = GraspDetectionArray3D()
            grasps_3d_msg.header.stamp = rospy.Time.now()
            grasps_3d_msg.header.frame_id = "base_link"
            
            # 转换2D抓取到3D（只处理有效检测）
            point_idx = 0
            for det_idx in valid_detection_indices:
                detection = self.latest_grasps_2d.detections[det_idx]
                detection_3d = GraspDetection3D()
                detection_3d.bbox_2d = detection.bbox
                detection_3d.score = detection.score
                
                for j, grasp_2d in enumerate(detection.grasp_poses):
                    if point_idx < len(points_3d):
                        grasp_3d = GraspPose3D()
                        
                        # 3D位置
                        point_3d = points_3d[point_idx]
                        point_idx += 1
                        grasp_3d.position.x = point_3d[0]
                        grasp_3d.position.y = point_3d[1] 
                        grasp_3d.position.z = point_3d[2]
                        
                        # 姿态 (简化处理)
                        grasp_3d.orientation.w = 1.0
                        
                        # 抓取参数
                        grasp_3d.width = grasp_2d.width / 1000.0  # 转换为米
                        grasp_3d.confidence = grasp_2d.confidence
                        
                        # 接触点 - touching_points在detection级别，不在grasp级别
                        # 简化处理：不处理touching_points
                        
                        detection_3d.grasp_poses_3d.append(grasp_3d)
                
                grasps_3d_msg.detections.append(detection_3d)
            
            # 发布3D抓取检测
            self.pub_grasps_3d.publish(grasps_3d_msg)
            
            # 发布点云用于可视化
            if self.enable_color and colors is not None:
                pointcloud_msg = self.depth_projector.create_colored_pointcloud2_msg(
                    points_3d, colors, "base_link", rospy.Time.now()
                )
            else:
                pointcloud_msg = self.depth_projector.create_pointcloud2_msg(
                    points_3d, "base_link", rospy.Time.now()
                )
            
            self.pub_pointcloud.publish(pointcloud_msg)
            
            rospy.loginfo(f"✅ 处理了 {len(points_3d)} 个3D抓取点 ({self.projection_method}方法)")
            rospy.loginfo(f"  发布了 {len(grasps_3d_msg.detections)} 个有效检测")
            
            # 显示3D位置统计
            if len(points_3d) > 0:
                z_values = [p[2] for p in points_3d]
                rospy.loginfo(f"  Z高度: min={min(z_values):.3f}m, max={max(z_values):.3f}m, avg={sum(z_values)/len(z_values):.3f}m")
            
        except Exception as e:
            rospy.logerr(f"❌ 3D抓取处理失败: {e}")
            import traceback
            traceback.print_exc()

def main():
    try:
        processor = DualProjectionGraspProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("3D抓取处理器已停止")
    except Exception as e:
        rospy.logerr(f"3D抓取处理器异常: {e}")

if __name__ == '__main__':
    main()