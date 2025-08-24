#!/usr/bin/python3
"""
相机管理器 - 统一管理多相机输入
处理图像订阅、缓存、时间同步和内参加载
"""

import rospy
import numpy as np
import threading
import time
from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os


class ImageBuffer:
    """图像缓存，支持时间同步"""
    
    def __init__(self, maxsize=2, sync_tolerance=0.1):
        self.maxsize = maxsize
        self.sync_tolerance = sync_tolerance
        self.color_buffer = deque()
        self.depth_buffer = deque()
        self.lock = threading.Lock()
    
    def add_color(self, msg):
        """添加彩色图像"""
        with self.lock:
            while len(self.color_buffer) >= self.maxsize:
                self.color_buffer.popleft()
            self.color_buffer.append(msg)
    
    def add_depth(self, msg):
        """添加深度图像"""
        with self.lock:
            while len(self.depth_buffer) >= self.maxsize:
                self.depth_buffer.popleft()
            self.depth_buffer.append(msg)
    
    def get_synchronized_pair(self):
        """获取时间同步的彩色+深度图像对"""
        with self.lock:
            if not self.color_buffer or not self.depth_buffer:
                return None, None
            
            # 找最新的彩色图像
            latest_color = self.color_buffer[-1]
            
            # 找时间戳最接近的深度图像
            best_depth = None
            min_time_diff = float('inf')
            
            color_time = latest_color.header.stamp.to_sec()
            
            for depth_msg in self.depth_buffer:
                depth_time = depth_msg.header.stamp.to_sec()
                time_diff = abs(color_time - depth_time)
                
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    best_depth = depth_msg
            
            # 检查时间同步是否在容差范围内
            if min_time_diff <= self.sync_tolerance and best_depth is not None:
                return latest_color, best_depth
            else:
                rospy.logdebug(f"Images not synchronized: time_diff={min_time_diff:.3f}s")
                return None, None


class IntrinsicsLoader:
    """相机内参加载器"""
    
    @staticmethod
    def load(source, camera_name, config_path=""):
        """加载相机内参
        Args:
            source: "realsense" 或 "calibration"
            camera_name: 相机名称 (hand, chassis, top)
            config_path: 标定文件路径
        """
        if source == "realsense":
            return IntrinsicsLoader._load_realsense_intrinsics(camera_name)
        elif source == "calibration":
            return IntrinsicsLoader._load_calibration_intrinsics(camera_name, config_path)
        else:
            rospy.logwarn(f"Unknown intrinsics source: {source}, using realsense defaults")
            return IntrinsicsLoader._load_realsense_intrinsics(camera_name)
    
    @staticmethod
    def _load_realsense_intrinsics(camera_name):
        """加载RealSense默认内参"""
        # 基于现有depth_projector_core.py的默认值
        realsense_params = {
            'hand': {
                'fx': 389.2, 'fy': 389.2,
                'cx': 316.2, 'cy': 242.0,
                'width': 640, 'height': 480
            },
            'chassis': {
                'fx': 389.2, 'fy': 389.2, 
                'cx': 316.2, 'cy': 242.0,
                'width': 640, 'height': 480
            },
            'top': {
                'fx': 389.2, 'fy': 389.2,
                'cx': 316.2, 'cy': 242.0, 
                'width': 640, 'height': 480
            }
        }
        
        if camera_name in realsense_params:
            params = realsense_params[camera_name]
            K = np.array([
                [params['fx'], 0, params['cx']],
                [0, params['fy'], params['cy']],
                [0, 0, 1]
            ])
            return {
                'K': K,
                'width': params['width'],
                'height': params['height'],
                'source': 'realsense'
            }
        else:
            rospy.logwarn(f"Unknown camera {camera_name}, using hand camera defaults")
            return IntrinsicsLoader._load_realsense_intrinsics('hand')
    
    @staticmethod
    def _load_calibration_intrinsics(camera_name, config_path):
        """加载标定内参"""
        try:
            # 根据相机名称选择配置文件
            config_files = {
                'hand': 'intrics_hand_camera.yaml',
                'chassis': 'intrics_chassis_camera.yaml', 
                'top': 'intrics_top_camera.yaml'
            }
            
            if camera_name not in config_files:
                rospy.logwarn(f"No calibration file for camera {camera_name}")
                return IntrinsicsLoader._load_realsense_intrinsics(camera_name)
            
            config_file = os.path.join(config_path, config_files[camera_name])
            
            if not os.path.exists(config_file):
                rospy.logwarn(f"Calibration file not found: {config_file}")
                return IntrinsicsLoader._load_realsense_intrinsics(camera_name)
            
            with open(config_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # 提取内参矩阵
            if 'camera_matrix' in calib_data and 'data' in calib_data['camera_matrix']:
                K_data = calib_data['camera_matrix']['data']
                K = np.array(K_data).reshape(3, 3)
            else:
                rospy.logwarn(f"Invalid calibration format in {config_file}")
                return IntrinsicsLoader._load_realsense_intrinsics(camera_name)
            
            # 图像尺寸
            width = calib_data.get('image_width', 640)
            height = calib_data.get('image_height', 480)
            
            return {
                'K': K,
                'width': width,
                'height': height, 
                'source': 'calibration',
                'file': config_file
            }
            
        except Exception as e:
            rospy.logerr(f"Failed to load calibration for {camera_name}: {e}")
            return IntrinsicsLoader._load_realsense_intrinsics(camera_name)


class CameraInterface:
    """单相机接口"""
    
    def __init__(self, name, config, global_settings):
        self.name = name
        self.config = config
        self.bridge = CvBridge()
        
        # 图像缓存
        buffer_size = global_settings.get('image_buffer_size', 2)
        sync_tolerance = global_settings.get('sync_tolerance', 0.1)
        self.buffer = ImageBuffer(buffer_size, sync_tolerance)
        
        # 加载内参
        config_path = config.get('config_path', '')
        self.intrinsics = IntrinsicsLoader.load(
            config['intrinsics_source'], 
            name,
            config_path
        )
        
        # ROS订阅器
        self._setup_subscribers()
        
        rospy.loginfo(f"📷 相机 {name} 初始化完成")
        rospy.loginfo(f"  内参源: {self.intrinsics['source']}")
        rospy.loginfo(f"  fx={self.intrinsics['K'][0,0]:.1f}, fy={self.intrinsics['K'][1,1]:.1f}")
        rospy.loginfo(f"  cx={self.intrinsics['K'][0,2]:.1f}, cy={self.intrinsics['K'][1,2]:.1f}")
    
    def _setup_subscribers(self):
        """设置ROS订阅器"""
        # 彩色图像订阅
        self.color_sub = rospy.Subscriber(
            self.config['topics']['color'],
            Image,
            self._color_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 深度图像订阅
        self.depth_sub = rospy.Subscriber(
            self.config['topics']['depth'],
            Image,
            self._depth_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 相机信息订阅（可选，用于动态内参）
        if 'info' in self.config['topics']:
            self.info_sub = rospy.Subscriber(
                self.config['topics']['info'],
                CameraInfo,
                self._info_callback,
                queue_size=1
            )
    
    def _color_callback(self, msg):
        """彩色图像回调"""
        self.buffer.add_color(msg)
        rospy.logdebug(f"Camera {self.name} received color image")
    
    def _depth_callback(self, msg):
        """深度图像回调"""
        self.buffer.add_depth(msg)
        rospy.logdebug(f"Camera {self.name} received depth image")
    
    def _info_callback(self, msg):
        """相机信息回调（用于动态更新内参）"""
        # 如果使用动态内参，可以在这里更新
        pass
    
    def get_synchronized_images(self):
        """获取时间同步的彩色+深度图像对"""
        color_msg, depth_msg = self.buffer.get_synchronized_pair()
        
        if color_msg is None or depth_msg is None:
            return None, None, None
        
        try:
            # 转换为OpenCV格式
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            
            # 确保深度图像是float32格式
            if depth_image.dtype != np.float32:
                depth_image = depth_image.astype(np.float32)
            
            return color_image, depth_image, color_msg.header
            
        except Exception as e:
            rospy.logerr(f"Image conversion error for camera {self.name}: {e}")
            return None, None, None
    
    def get_intrinsics(self):
        """获取相机内参"""
        return self.intrinsics.copy()


class CameraManager:
    """相机管理器 - 管理所有相机接口"""
    
    def __init__(self, camera_configs, global_settings):
        self.cameras = {}
        self.global_settings = global_settings
        
        # 为每个启用的相机创建接口
        for name, config in camera_configs.items():
            if config.get('enabled', False):
                try:
                    # 添加配置路径信息
                    if 'config_path' not in config and 'projection_methods' in global_settings:
                        config['config_path'] = global_settings.get('config_path', '')
                    
                    self.cameras[name] = CameraInterface(name, config, global_settings)
                    rospy.loginfo(f"✅ 相机 {name} 启动成功")
                except Exception as e:
                    rospy.logerr(f"❌ 相机 {name} 启动失败: {e}")
            else:
                rospy.loginfo(f"⏸️ 相机 {name} 已禁用")
        
        rospy.loginfo(f"🎥 相机管理器初始化完成，活跃相机: {list(self.cameras.keys())}")
    
    def get_camera(self, camera_name):
        """获取指定相机接口"""
        if camera_name in self.cameras:
            return self.cameras[camera_name]
        else:
            raise ValueError(f"Camera {camera_name} not found or not enabled")
    
    def get_active_cameras(self):
        """获取所有活跃相机名称"""
        return list(self.cameras.keys())
    
    def shutdown(self):
        """关闭所有相机"""
        for camera in self.cameras.values():
            # 取消订阅
            if hasattr(camera, 'color_sub'):
                camera.color_sub.unregister()
            if hasattr(camera, 'depth_sub'):
                camera.depth_sub.unregister()
            if hasattr(camera, 'info_sub'):
                camera.info_sub.unregister()
        
        rospy.loginfo("📷 所有相机已关闭")


if __name__ == '__main__':
    # 测试代码
    rospy.init_node('camera_manager_test')
    
    # 测试配置
    test_config = {
        'hand': {
            'topics': {
                'color': '/camera/hand/color/image_raw',
                'depth': '/camera/hand/depth/image_rect_raw',
                'info': '/camera/hand/depth/camera_info'
            },
            'intrinsics_source': 'realsense',
            'enabled': True
        }
    }
    
    test_global = {
        'image_buffer_size': 2,
        'sync_tolerance': 0.1
    }
    
    try:
        manager = CameraManager(test_config, test_global)
        
        # 测试获取图像
        hand_camera = manager.get_camera('hand')
        
        rate = rospy.Rate(1)  # 1Hz测试
        while not rospy.is_shutdown():
            color, depth, header = hand_camera.get_synchronized_images()
            if color is not None and depth is not None:
                rospy.loginfo(f"Got synchronized images: color={color.shape}, depth={depth.shape}")
                rospy.loginfo(f"Time: {header.stamp.to_sec():.3f}")
            else:
                rospy.loginfo("Waiting for synchronized images...")
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        manager.shutdown()