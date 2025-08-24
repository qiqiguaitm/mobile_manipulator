#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 修复libffi版本冲突问题

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import message_filters

from depth_projector_core import DepthProjectionManager, CameraDepthProjector


class DepthProjectionNode:
    """
    深度投影ROS节点 - 订阅深度图像，发布投影后的点云
    """
    
    def __init__(self):
        rospy.init_node('depth_projection_node', anonymous=True)
        
        # 参数配置
        self.enable_chassis = rospy.get_param('~enable_chassis_camera', True)
        self.enable_top = rospy.get_param('~enable_top_camera', True)  
        self.enable_hand = rospy.get_param('~enable_hand_camera', True)
        self.publish_combined = rospy.get_param('~publish_combined_cloud', True)
        
        # 深度缩放参数
        self.depth_scales = {
            'chassis_camera': rospy.get_param('~chassis_depth_scale', 1000.0),
            'top_camera': rospy.get_param('~top_depth_scale', 1000.0), 
            'hand_camera': rospy.get_param('~hand_depth_scale', 1000.0)
        }
        
        # 话题名称配置
        self.depth_topics = {
            'chassis_camera': rospy.get_param('~chassis_depth_topic', '/chassis_camera/depth/image_raw'),
            'top_camera': rospy.get_param('~top_depth_topic', '/top_camera/depth/image_raw'),
            'hand_camera': rospy.get_param('~hand_depth_topic', '/hand_camera/depth/image_raw')
        }
        
        # 颜色图像话题配置
        self.color_topics = {
            'chassis_camera': rospy.get_param('~chassis_color_topic', '/camera/chassis/color/image_rect_color'),
            'top_camera': rospy.get_param('~top_color_topic', '/camera/top/color/image_rect_color'), 
            'hand_camera': rospy.get_param('~hand_color_topic', '/camera/hand/color/image_rect_color')
        }
        
        # 是否启用颜色
        self.enable_color = rospy.get_param('~enable_color', True)
        
        # 投影方法
        self.projection_method = rospy.get_param('~projection_method', 'urdf')
        
        self.bridge = CvBridge()
        
        # 初始化投影管理器
        enabled_cameras = []
        if self.enable_chassis: enabled_cameras.append('chassis_camera')
        if self.enable_top: enabled_cameras.append('top_camera') 
        if self.enable_hand: enabled_cameras.append('hand_camera')
        
        try:
            self.manager = DepthProjectionManager(enabled_cameras, projection_method=self.projection_method)
            rospy.loginfo(f"Initialized depth projection for cameras: {enabled_cameras}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize DepthProjectionManager: {e}")
            return
            
        # 设置发布者
        self.publishers = {}
        for camera_name in enabled_cameras:
            topic_name = f"/projected_cloud/{camera_name}"
            self.publishers[camera_name] = rospy.Publisher(
                topic_name, PointCloud2, queue_size=1)
                
        if self.publish_combined:
            self.combined_publisher = rospy.Publisher(
                "/projected_cloud/combined", PointCloud2, queue_size=1)
        
        # 设置订阅者
        self.subscribers = {}
        self.color_subscribers = {}
        self.latest_images = {}
        self.sync_subscribers = {}
        
        if self.enable_color:
            # 使用message_filters同步深度和颜色图像
            for camera_name in enabled_cameras:
                depth_topic = self.depth_topics[camera_name]
                color_topic = self.color_topics[camera_name]
                
                depth_sub = message_filters.Subscriber(depth_topic, Image)
                color_sub = message_filters.Subscriber(color_topic, Image)
                
                # 时间同步器
                sync = message_filters.ApproximateTimeSynchronizer(
                    [depth_sub, color_sub], queue_size=5, slop=0.1)
                sync.registerCallback(self._rgbd_callback, camera_name)
                
                self.sync_subscribers[camera_name] = sync
                self.latest_images[camera_name] = None
                
                rospy.loginfo(f"Set up RGBD sync for {camera_name}: {depth_topic} + {color_topic}")
        else:
            # 只订阅深度图像
            for camera_name in enabled_cameras:
                topic = self.depth_topics[camera_name]
                self.subscribers[camera_name] = rospy.Subscriber(
                    topic, Image, self._depth_callback, camera_name)
                self.latest_images[camera_name] = None
            
        # 定时器处理 (如果需要同步多相机)
        if len(enabled_cameras) > 1:
            self.sync_timer = rospy.Timer(rospy.Duration(0.1), self._sync_callback)
            
        rospy.loginfo("DepthProjectionNode initialized successfully")
        
    def _rgbd_callback(self, depth_msg, color_msg, camera_name):
        """RGBD同步回调"""
        try:
            # 转换深度图像
            if depth_msg.encoding in ['mono16', '16UC1']:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            elif depth_msg.encoding in ['mono8', '8UC1']:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                depth_image = depth_image.astype(np.uint16) * 256
            else:
                rospy.logwarn(f"Unsupported depth encoding: {depth_msg.encoding}")
                return
            
            # 转换颜色图像
            if color_msg.encoding in ['rgb8', 'bgr8']:
                color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            elif color_msg.encoding in ['mono8']:
                color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='mono8')
            else:
                rospy.logwarn(f"Unsupported color encoding: {color_msg.encoding}")
                color_image = None
            
            # 存储最新图像
            self.latest_images[camera_name] = {
                'depth_image': depth_image,
                'color_image': color_image,
                'timestamp': depth_msg.header.stamp,
                'frame_id': depth_msg.header.frame_id
            }
            
            # 立即处理单个相机 (如果不需要多相机同步)
            if len(self.latest_images) == 1:
                self._process_single_camera_rgbd(camera_name, depth_image, color_image, depth_msg.header)
                
        except Exception as e:
            rospy.logerr(f"Error processing {camera_name} RGBD images: {e}")
        
    def _depth_callback(self, msg, camera_name):
        """深度图像回调"""
        try:
            # 转换ROS图像消息为OpenCV格式
            if msg.encoding in ['mono16', '16UC1']:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg.encoding in ['mono8', '8UC1']:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_image = depth_image.astype(np.uint16) * 256  # 转换为16位
            else:
                rospy.logwarn(f"Unsupported depth encoding: {msg.encoding}")
                return
                
            # 存储最新图像
            self.latest_images[camera_name] = {
                'image': depth_image,
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            
            # 立即处理单个相机 (如果不需要同步)
            if len(self.subscribers) == 1:
                self._process_single_camera(camera_name, depth_image, msg.header)
                
        except Exception as e:
            rospy.logerr(f"Error processing {camera_name} depth image: {e}")
    
    def _process_single_camera_rgbd(self, camera_name, depth_image, color_image, header):
        """处理单个相机的RGBD图像"""
        projector = self.manager.get_projector(camera_name)
        if projector is None:
            return
            
        try:
            # 投影到机器人坐标系
            scale = self.depth_scales.get(camera_name, 1000.0)
            robot_points, colors, valid_mask = projector.project_rgbd_to_robot_coordinates(
                depth_image, color_image, scale)
                
            if robot_points.shape[0] == 0:
                rospy.logdebug(f"No valid points from {camera_name}")
                return
                
            # 创建并发布彩色点云消息
            cloud_msg = projector.create_colored_pointcloud2_msg(
                robot_points, colors, frame_id="base_link", timestamp=header.stamp)
                
            if camera_name in self.publishers:
                self.publishers[camera_name].publish(cloud_msg)
                
            rospy.logdebug(f"Published {robot_points.shape[0]} colored points from {camera_name}")
            
        except Exception as e:
            rospy.logerr(f"Error projecting RGBD {camera_name}: {e}")
            
    def _process_single_camera(self, camera_name, depth_image, header):
        """处理单个相机的深度图像"""
        projector = self.manager.get_projector(camera_name)
        if projector is None:
            return
            
        try:
            # 投影到机器人坐标系
            scale = self.depth_scales.get(camera_name, 1000.0)
            robot_points, valid_mask = projector.project_to_robot_coordinates(
                depth_image, scale)
                
            if robot_points.shape[0] == 0:
                rospy.logdebug(f"No valid points from {camera_name}")
                return
                
            # 创建并发布点云消息
            cloud_msg = projector.create_pointcloud2_msg(
                robot_points, frame_id="base_link", timestamp=header.stamp)
                
            if camera_name in self.publishers:
                self.publishers[camera_name].publish(cloud_msg)
                
            rospy.logdebug(f"Published {robot_points.shape[0]} points from {camera_name}")
            
        except Exception as e:
            rospy.logerr(f"Error projecting {camera_name}: {e}")
            
    def _sync_callback(self, event):
        """同步多相机处理"""
        if not self.publish_combined:
            return
            
        # 检查是否所有相机都有数据
        valid_images = {}
        current_time = rospy.Time.now()
        
        for camera_name, data in self.latest_images.items():
            if data is not None:
                # 检查数据是否过时 (1秒内)
                age = (current_time - data['timestamp']).to_sec()
                if age < 1.0:
                    valid_images[camera_name] = data
                    
        if len(valid_images) == 0:
            return
            
        # 批量投影
        if self.enable_color:
            # RGBD模式
            depth_images = {name: data['depth_image'] for name, data in valid_images.items()}
            color_images = {name: data['color_image'] for name, data in valid_images.items()}
            results = self.manager.project_all_cameras(depth_images, self.depth_scales, color_images)
        else:
            # 纯深度模式
            depth_images = {name: data['image'] for name, data in valid_images.items()}
            results = self.manager.project_all_cameras(depth_images, self.depth_scales)
        
        # 合并点云
        all_points = []
        all_colors = []
        has_colors = self.enable_color
        
        for camera_name, result in results.items():
            if result['points'].shape[0] > 0:
                all_points.append(result['points'])
                if has_colors and result['colors'] is not None:
                    all_colors.append(result['colors'])
                else:
                    has_colors = False  # 如果任何相机没有颜色，则合并点云也不使用颜色
                
                # 发布单独的点云
                if camera_name in self.publishers:
                    self.publishers[camera_name].publish(result['pointcloud_msg'])
                    
        # 发布合并点云
        if len(all_points) > 0 and hasattr(self, 'combined_publisher'):
            combined_points = np.vstack(all_points)
            
            # 使用最新的时间戳
            latest_timestamp = max([data['timestamp'] for data in valid_images.values()])
            
            # 创建合并点云消息
            projector = list(self.manager.projectors.values())[0]  # 使用任一投影器
            
            if has_colors and len(all_colors) > 0:
                combined_colors = np.vstack(all_colors)
                combined_msg = projector.create_colored_pointcloud2_msg(
                    combined_points, combined_colors, frame_id="base_link", timestamp=latest_timestamp)
            else:
                combined_msg = projector.create_pointcloud2_msg(
                    combined_points, frame_id="base_link", timestamp=latest_timestamp)
                
            self.combined_publisher.publish(combined_msg)
            
            rospy.logdebug(f"Published combined cloud with {combined_points.shape[0]} points")


class SingleCameraProjectionNode:
    """
    单相机深度投影节点 - 用于只处理一个相机的情况
    """
    
    def __init__(self, camera_name):
        self.camera_name = camera_name
        rospy.init_node(f'{camera_name}_depth_projection', anonymous=True)
        
        # 参数配置
        self.depth_scale = rospy.get_param('~depth_scale', 1000.0)
        self.depth_topic = rospy.get_param('~depth_topic', f'/{camera_name}/depth/image_raw')
        self.output_topic = rospy.get_param('~output_topic', f'/projected_cloud/{camera_name}')
        self.projection_method = rospy.get_param('~projection_method', 'urdf')
        
        self.bridge = CvBridge()
        
        # 初始化投影器
        try:
            self.projector = CameraDepthProjector(camera_name, projection_method=self.projection_method)
            rospy.loginfo(f"Initialized projector for {camera_name}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize projector: {e}")
            return
            
        # 设置发布者和订阅者
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(self.depth_topic, Image, self._depth_callback)
        
        rospy.loginfo(f"SingleCameraProjectionNode for {camera_name} initialized")
        
    def _depth_callback(self, msg):
        """深度图像回调"""
        try:
            # 转换图像
            if msg.encoding in ['mono16', '16UC1']:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            else:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                depth_image = depth_image.astype(np.uint16) * 256
                
            # 投影
            robot_points, valid_mask = self.projector.project_to_robot_coordinates(
                depth_image, self.depth_scale)
                
            if robot_points.shape[0] > 0:
                # 发布点云
                cloud_msg = self.projector.create_pointcloud2_msg(
                    robot_points, frame_id="base_link", timestamp=msg.header.stamp)
                self.publisher.publish(cloud_msg)
                
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {e}")


if __name__ == "__main__":
    import sys
    
    # 过滤掉ROS参数，寻找实际的相机名称参数
    camera_name = None
    for arg in sys.argv[1:]:
        if not arg.startswith('__') and not arg.startswith('_') and not '.' in arg:
            camera_name = arg
            break
            
    if camera_name:
        # 单相机模式
        node = SingleCameraProjectionNode(camera_name)
    else:
        # 多相机模式
        node = DepthProjectionNode()
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down depth projection node")