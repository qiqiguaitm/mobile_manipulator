#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 修复libffi版本冲突问题

import numpy as np
import yaml
import cv2
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import PointCloud2, PointField
import rospy

class CameraDepthProjector:
    """
    统一的RGBD图像投影器 - 将相机深度+彩色图像投影到机器人坐标系
    
    核心思想：消除特殊情况，统一处理三个相机的RGBD投影
    好品味：一套代码同时处理深度和颜色，而不是两套系统
    """
    
    def __init__(self, camera_name, projection_method='urdf', config_path="/home/agilex/MobileManipulator/src/robot_drivers/camera_driver/config"):
        self.camera_name = camera_name
        self.projection_method = projection_method
        self.config_path = config_path
        
        # 根据投影方法选择内参和外参数据源
        self._load_camera_params()
        self._build_transform_chain()
        
    def _load_camera_params(self):
        """根据投影方法选择内参数据源"""
        if self.projection_method == 'urdf':
            # RealSense驱动内参
            self._load_intrinsics_from_camera_info()
        elif self.projection_method == 'calibration':
            # 标定文件内参
            self._load_intrinsics_from_config()
        elif self.projection_method == 'compare':
            # 对比模式：优先使用标定内参，TF使用标定链
            rospy.loginfo("对比模式：使用标定内参，将发布两条TF链用于对比")
            self._load_intrinsics_from_config()
            # 在对比模式下，主要使用标定方法，但会在节点中创建对比发布器
        else:
            raise ValueError(f"Unknown projection method: {self.projection_method}")
            
    def _load_intrinsics_from_camera_info(self):
        """从RealSense驱动获取内参"""
        camera_short_names = {
            'chassis_camera': 'chassis',
            'top_camera': 'top',
            'hand_camera': 'hand'
        }
        camera_short = camera_short_names.get(self.camera_name, self.camera_name)
        topic = f"/camera/{camera_short}/depth/camera_info"
        
        try:
            from sensor_msgs.msg import CameraInfo
            rospy.loginfo(f"Getting RealSense intrinsics from {topic}...")
            msg = rospy.wait_for_message(topic, CameraInfo, timeout=10.0)
            
            self.K = np.array(msg.K).reshape(3, 3)
            self.D = np.array(msg.D) if len(msg.D) > 0 else np.zeros(5)
            self.image_width = msg.width
            self.image_height = msg.height
            
            rospy.loginfo(f"RealSense intrinsics [{self.camera_name}]: {self.image_width}x{self.image_height}")
            rospy.loginfo(f"  fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}")
            
        except Exception as e:
            rospy.logerr(f"Failed to get RealSense camera_info: {e}")
            raise
            
    def _load_intrinsics_from_config(self):
        """从标定文件获取内参"""
        intrinsics_file = f"{self.config_path}/intrics_{self.camera_name}.yaml"
        
        try:
            with open(intrinsics_file, 'r') as f:
                content = f.read()
                
                # 按连续空行分割不同的配置块，跳过YAML头部
                config_blocks = []
                current_block = []
                
                for line in content.split('\n'):
                    # 跳过YAML头部行
                    if line.strip() and not line.startswith('%YAML') and not line.startswith('---'):
                        current_block.append(line)
                    else:
                        if current_block:
                            config_blocks.append('\n'.join(current_block))
                            current_block = []
                
                # 添加最后一个配置块
                if current_block:
                    config_blocks.append('\n'.join(current_block))
                
                # 优先选择640x480配置
                config = None
                preferred_config = None
                
                for block in config_blocks:
                    try:
                        temp_config = yaml.safe_load(block)
                        if temp_config and 'projection_parameters' in temp_config:
                            # 记录第一个有效配置作为备选
                            if config is None:
                                config = temp_config
                            
                            # 优先选择640x480分辨率
                            if (temp_config.get('image_width') == 640 and 
                                temp_config.get('image_height') == 480):
                                preferred_config = temp_config
                                rospy.loginfo(f"Found preferred 640x480 config for {self.camera_name}")
                                break
                    except Exception as e:
                        continue
                
                # 使用优先配置或第一个有效配置
                if preferred_config:
                    config = preferred_config
                    rospy.loginfo(f"Using preferred 640x480 calibration for {self.camera_name}")
                else:
                    if config:
                        rospy.logwarn(f"No 640x480 config found, using {config.get('image_width')}x{config.get('image_height')} for {self.camera_name}")
                    else:
                        raise ValueError("No valid calibration config found")
                    
            # 构建内参矩阵
            params = config['projection_parameters']
            self.K = np.array([
                [params['fx'], 0, params['cx']],
                [0, params['fy'], params['cy']],
                [0, 0, 1]
            ])
            
            # 畸变系数
            dist = config['distortion_parameters']
            self.D = np.array([dist['k1'], dist['k2'], dist['p1'], dist['p2']])
            
            # 图像尺寸
            self.image_width = config['image_width']
            self.image_height = config['image_height']
            
            rospy.loginfo(f"Calibrated intrinsics [{self.camera_name}]: {self.image_width}x{self.image_height}")
            rospy.loginfo(f"  fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}")
            
        except Exception as e:
            rospy.logerr(f"Failed to load calibrated intrinsics: {e}")
            raise
        
    def _build_transform_chain(self):
        """根据投影方法选择外参TF frame"""
        frame_maps = {
            'urdf': {
                'chassis_camera': 'camera/chassis_depth_optical_frame',
                'top_camera': 'camera/top_depth_optical_frame',
                'hand_camera': 'camera/hand_depth_optical_frame'
            },
            'calibration': {
                'chassis_camera': 'calibration/chassis_camera_optical',
                'top_camera': 'calibration/top_camera_optical',
                'hand_camera': 'calibration/hand_camera_optical'
            },
            'compare': {
                # 对比模式默认使用标定链，但节点中会处理双链对比
                'chassis_camera': 'calibration/chassis_camera_optical',
                'top_camera': 'calibration/top_camera_optical',
                'hand_camera': 'calibration/hand_camera_optical'
            }
        }
        
        frame_map = frame_maps[self.projection_method]
        self.camera_frame = frame_map.get(self.camera_name, 
                                         f"{self.projection_method}/{self.camera_name}_optical")
        
        # 初始化TF系统
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo(f"Using {self.projection_method} method, TF frame: {self.camera_frame}")
            
    def _yaml_to_transform_matrix(self, transform_yaml):
        """将YAML格式的变换转换为4x4矩阵"""
        trans = transform_yaml['translation']
        rot = transform_yaml['rotation']
        
        # 四元数转旋转矩阵 - 兼容老版本scipy
        quat = [rot['x'], rot['y'], rot['z'], rot['w']]
        r_obj = R.from_quat(quat)
        if hasattr(r_obj, 'as_matrix'):
            rotation_matrix = r_obj.as_matrix()
        else:
            rotation_matrix = r_obj.as_dcm()  # scipy < 1.4.0
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = [trans['x'], trans['y'], trans['z']]
        
        return T
        
    def _get_transform_to_robot_base(self):
        """获取从相机坐标系到机器人base_link的完整变换"""
        try:
            # 直接从TF树查询变换
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.camera_frame, rospy.Time(0), rospy.Duration(1.0))
                
            # 转换为4x4矩阵
            T_total = self._transform_stamped_to_matrix(transform)
            
            return T_total
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get TF transform {self.camera_frame} -> base_link: {e}")
            # 返回单位矩阵作为fallback
            return np.eye(4)
        
    def _transform_stamped_to_matrix(self, transform_stamped):
        """将TransformStamped转换为4x4矩阵"""
        t = transform_stamped.transform
        trans = [t.translation.x, t.translation.y, t.translation.z]
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        
        r_obj = R.from_quat(quat)
        if hasattr(r_obj, 'as_matrix'):
            rotation_matrix = r_obj.as_matrix()
        else:
            rotation_matrix = r_obj.as_dcm()  # scipy < 1.4.0
        
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = trans
        
        return T
        
    def deproject_depth_image(self, depth_image, depth_scale=1000.0):
        """
        将深度图像反投影到相机坐标系3D点云
        
        Args:
            depth_image: numpy array, 深度图像 (单位：毫米或自定义)
            depth_scale: 深度值到米的转换比例 (默认1000，即毫米转米)
            
        Returns:
            points_3d: Nx3 numpy array, 相机坐标系下的3D点
            valid_mask: 有效点的mask
        """
        h, w = depth_image.shape
        
        # 生成像素网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # 有效深度点
        valid_mask = (depth_image > 0) & (depth_image < 65535)  # 排除无效值
        
        if not np.any(valid_mask):
            rospy.logwarn("No valid depth points found")
            return np.array([]).reshape(0, 3), valid_mask
            
        # 提取有效点
        u_valid = u[valid_mask]
        v_valid = v[valid_mask] 
        depth_valid = depth_image[valid_mask] / depth_scale  # 转换为米
        
        # 去畸变（可选，如果需要高精度）
        # points_2d = np.column_stack([u_valid, v_valid]).astype(np.float32)
        # points_2d_undistorted = cv2.undistortPoints(
        #     points_2d.reshape(-1, 1, 2), self.K, self.D, P=self.K)
        # u_valid = points_2d_undistorted[:, 0, 0]
        # v_valid = points_2d_undistorted[:, 0, 1]
        
        # 反投影到3D空间
        x = (u_valid - self.K[0, 2]) * depth_valid / self.K[0, 0]
        y = (v_valid - self.K[1, 2]) * depth_valid / self.K[1, 1] 
        z = depth_valid
        
        points_3d = np.column_stack([x, y, z])
        
        return points_3d, valid_mask
        
    def project_rgbd_to_robot_coordinates(self, depth_image, color_image=None, depth_scale=1000.0):
        """
        完整的RGBD投影管道：深度+颜色图像 -> 机器人坐标系彩色点云
        
        Args:
            depth_image: 深度图像
            color_image: RGB颜色图像 (可选)
            depth_scale: 深度缩放因子
            
        Returns:
            robot_points: Nx3 numpy array, 机器人坐标系下的3D点
            colors: Nx3 numpy array, RGB颜色值 (0-255) 或 None
            valid_mask: 有效点mask
        """
        # Step 1: 深度图 -> 相机坐标系点云
        camera_points, valid_mask = self.deproject_depth_image(depth_image, depth_scale)
        
        if camera_points.shape[0] == 0:
            return camera_points, None, valid_mask
            
        # Step 2: 提取颜色信息
        colors = None
        if color_image is not None:
            # 从深度图有效点位置提取颜色
            h, w = depth_image.shape
            u, v = np.meshgrid(np.arange(w), np.arange(h))
            u_valid = u[valid_mask]
            v_valid = v[valid_mask]
            
            # 确保颜色图像尺寸匹配
            if color_image.shape[:2] == (h, w):
                if len(color_image.shape) == 3 and color_image.shape[2] == 3:
                    # RGB图像
                    colors = color_image[v_valid, u_valid]  # 注意索引顺序
                elif len(color_image.shape) == 2:
                    # 灰度图像，转换为RGB
                    gray_values = color_image[v_valid, u_valid]
                    colors = np.stack([gray_values] * 3, axis=-1)
                else:
                    rospy.logwarn(f"Unsupported color image shape: {color_image.shape}")
            else:
                rospy.logwarn(f"Color image size {color_image.shape[:2]} != depth size {(h, w)}")
                
        # Step 3: 相机坐标系 -> 机器人坐标系
        T_camera_to_robot = self._get_transform_to_robot_base()
        
        # 齐次坐标变换
        camera_points_homo = np.hstack([camera_points, np.ones((camera_points.shape[0], 1))])
        robot_points_homo = (T_camera_to_robot @ camera_points_homo.T).T
        robot_points = robot_points_homo[:, :3]
        
        rospy.logdebug(f"Projected {camera_points.shape[0]} RGBD points to robot coordinates")
        
        return robot_points, colors, valid_mask

    def project_depth_to_3d(self, pixel_coords, depths):
        """
        投影特定像素点到3D机器人坐标系
        
        Args:
            pixel_coords: Nx2 numpy array, 像素坐标 [[u1,v1], [u2,v2], ...]
            depths: Nx1 numpy array, 对应的深度值 (米)
            
        Returns:
            points_3d: Nx3 numpy array, 机器人坐标系下的3D点
        """
        pixel_coords = np.array(pixel_coords)
        depths = np.array(depths)
        
        if pixel_coords.ndim == 1:
            pixel_coords = pixel_coords.reshape(1, -1)
        if depths.ndim == 0:
            depths = depths.reshape(1)
            
        # 转换为相机坐标系
        u = pixel_coords[:, 0]
        v = pixel_coords[:, 1]
        
        # 使用内参矩阵反投影
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        
        # 相机坐标系下的3D点
        x_cam = (u - cx) * depths / fx
        y_cam = (v - cy) * depths / fy
        z_cam = depths
        
        camera_points = np.column_stack([x_cam, y_cam, z_cam])
        
        # 转换到机器人坐标系
        T_camera_to_robot = self._get_transform_to_robot_base()
        camera_points_homo = np.hstack([camera_points, np.ones((camera_points.shape[0], 1))])
        robot_points_homo = (T_camera_to_robot @ camera_points_homo.T).T
        robot_points = robot_points_homo[:, :3]
        
        return robot_points
        
    def project_to_robot_coordinates(self, depth_image, depth_scale=1000.0):
        """
        完整的深度投影管道：深度图像 -> 机器人坐标系点云
        
        Args:
            depth_image: 深度图像
            depth_scale: 深度缩放因子
            
        Returns:
            robot_points: Nx3 numpy array, 机器人坐标系下的3D点  
            valid_mask: 有效点mask
        """
        # Step 1: 深度图 -> 相机坐标系点云
        camera_points, valid_mask = self.deproject_depth_image(depth_image, depth_scale)
        
        if camera_points.shape[0] == 0:
            return camera_points, valid_mask
            
        # Step 2: 相机坐标系 -> 机器人坐标系
        T_camera_to_robot = self._get_transform_to_robot_base()
        
        # 齐次坐标变换
        camera_points_homo = np.hstack([camera_points, np.ones((camera_points.shape[0], 1))])
        robot_points_homo = (T_camera_to_robot @ camera_points_homo.T).T
        robot_points = robot_points_homo[:, :3]
        
        rospy.logdebug(f"Projected {camera_points.shape[0]} points to robot coordinates")
        
        return robot_points, valid_mask
        
    def create_colored_pointcloud2_msg(self, points, colors=None, frame_id="base_link", timestamp=None):
        """创建带颜色的ROS PointCloud2消息"""
        if timestamp is None:
            timestamp = rospy.Time.now()
            
        # 定义点云字段 - 包含颜色
        if colors is not None:
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1), 
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1)
            ]
            point_step = 16  # 4 * 4 bytes
        else:
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1), 
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            point_step = 12  # 3 * 4 bytes
        
        # 创建消息
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = timestamp
        cloud_msg.header.frame_id = frame_id
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = point_step
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        # 构造数据
        if colors is not None:
            # 将RGB值打包为单个uint32
            rgb_packed = np.zeros(colors.shape[0], dtype=np.uint32)
            rgb_packed = (colors[:, 0].astype(np.uint32) << 16) | \
                        (colors[:, 1].astype(np.uint32) << 8) | \
                        colors[:, 2].astype(np.uint32)
            
            # 组合点和颜色数据
            structured_data = np.zeros(points.shape[0], dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
            structured_data['x'] = points[:, 0]
            structured_data['y'] = points[:, 1] 
            structured_data['z'] = points[:, 2]
            structured_data['rgb'] = rgb_packed
            cloud_msg.data = structured_data.tobytes()
        else:
            # 只有位置数据
            cloud_msg.data = points.astype(np.float32).tobytes()
            
        return cloud_msg
        
    def create_pointcloud2_msg(self, points, frame_id="base_link", timestamp=None):
        """创建ROS PointCloud2消息"""
        if timestamp is None:
            timestamp = rospy.Time.now()
            
        # 定义点云字段
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1), 
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        
        # 创建消息
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = timestamp
        cloud_msg.header.frame_id = frame_id
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = points.astype(np.float32).tobytes()
        cloud_msg.is_dense = True
        
        return cloud_msg


class DepthProjectionManager:
    """深度投影管理器 - 管理多个相机的深度投影"""
    
    def __init__(self, camera_names=None, projection_method='urdf'):
        if camera_names is None:
            camera_names = ['chassis_camera', 'top_camera', 'hand_camera']
            
        self.projection_method = projection_method
        self.projectors = {}
        
        for camera_name in camera_names:
            try:
                self.projectors[camera_name] = CameraDepthProjector(
                    camera_name, projection_method=projection_method)
                rospy.loginfo(f"Initialized {camera_name} with {projection_method} method")
            except Exception as e:
                rospy.logerr(f"Failed to initialize {camera_name}: {e}")
                
    def get_projector(self, camera_name):
        """获取指定相机的投影器"""
        return self.projectors.get(camera_name, None)
        
    def project_all_cameras(self, depth_images, depth_scales=None, color_images=None):
        """批量投影多个相机的RGBD图像"""
        if depth_scales is None:
            depth_scales = {name: 1000.0 for name in depth_images.keys()}
            
        results = {}
        for camera_name, depth_image in depth_images.items():
            if camera_name in self.projectors:
                projector = self.projectors[camera_name]
                scale = depth_scales.get(camera_name, 1000.0)
                color_image = color_images.get(camera_name) if color_images else None
                
                if color_image is not None:
                    # 使用RGBD投影
                    robot_points, colors, valid_mask = projector.project_rgbd_to_robot_coordinates(
                        depth_image, color_image, scale)
                    pointcloud_msg = projector.create_colored_pointcloud2_msg(
                        robot_points, colors)
                else:
                    # 纯深度投影
                    robot_points, valid_mask = projector.project_to_robot_coordinates(
                        depth_image, scale)
                    colors = None
                    pointcloud_msg = projector.create_pointcloud2_msg(robot_points)
                    
                results[camera_name] = {
                    'points': robot_points,
                    'colors': colors,
                    'valid_mask': valid_mask,
                    'pointcloud_msg': pointcloud_msg
                }
                
        return results
        
    def compare_projection_methods(self, camera_name, depth_image, color_image=None, depth_scale=1000.0):
        """对比两种投影方法的结果"""
        results = {}
        
        for method in ['urdf', 'calibration']:
            rospy.loginfo(f"Testing {method} projection for {camera_name}")
            
            try:
                # 创建临时投影器
                temp_projector = CameraDepthProjector(camera_name, projection_method=method)
                
                if color_image is not None:
                    points, colors, mask = temp_projector.project_rgbd_to_robot_coordinates(
                        depth_image, color_image, depth_scale)
                else:
                    points, mask = temp_projector.project_to_robot_coordinates(
                        depth_image, depth_scale)
                    colors = None
                    
                results[method] = {
                    'points': points,
                    'colors': colors,
                    'valid_count': np.sum(mask),
                    'frame': temp_projector.camera_frame,
                    'intrinsics': {
                        'K': temp_projector.K.tolist(),
                        'size': [temp_projector.image_width, temp_projector.image_height]
                    }
                }
                
                rospy.loginfo(f"{method}: {len(points)} points, frame: {temp_projector.camera_frame}")
                rospy.loginfo(f"  intrinsics: fx={temp_projector.K[0,0]:.1f}, fy={temp_projector.K[1,1]:.1f}")
                
            except Exception as e:
                rospy.logerr(f"{method} projection failed: {e}")
                results[method] = None
        
        return results


if __name__ == "__main__":
    # 简单测试
    rospy.init_node('depth_projector_test')
    
    try:
        # 测试单个相机投影器
        projector = CameraDepthProjector('chassis_camera')
        rospy.loginfo("CameraDepthProjector initialized successfully")
        
        # 测试管理器
        manager = DepthProjectionManager()
        rospy.loginfo("DepthProjectionManager initialized successfully") 
        
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
