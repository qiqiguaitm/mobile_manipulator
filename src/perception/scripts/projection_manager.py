#!/usr/bin/python3
"""
投影管理器 - 统一管理3D投影方法
支持URDF和Calibration两种TF链路，复用depth_projector_core
"""

import rospy
import numpy as np
import sys
import os
from sensor_msgs.msg import PointCloud2
from perception.msg import (GraspDetectionArray3D, GraspDetection3D, 
                           GraspPose3D, TouchingPoint3D)
from geometry_msgs.msg import Point
import struct

# 添加depth_projector_core路径
sys.path.append(os.path.dirname(__file__))
try:
    from depth_projector_core import CameraDepthProjector
except ImportError as e:
    rospy.logerr(f"无法导入depth_projector_core: {e}")


class FilterManager:
    """3D点过滤管理器"""
    
    def __init__(self):
        pass
    
    def apply_filters(self, points_3d, filters, debug=False):
        """应用多种过滤器到3D点"""
        if points_3d is None or len(points_3d) == 0:
            return []
        
        points_3d = np.array(points_3d)
        initial_count = len(points_3d)
        
        # 深度范围过滤已经在2D阶段完成，这里主要做3D空间过滤
        
        # 1. Z高度过滤 (base_link坐标系)
        z_min = filters.get('min_z_height_m', -0.5)
        z_max = filters.get('max_z_height_m', 2.0)
        z_mask = (points_3d[:, 2] >= z_min) & (points_3d[:, 2] <= z_max)
        
        if debug:
            z_filtered = np.sum(~z_mask)
            if z_filtered > 0:
                rospy.loginfo(f"Z高度过滤: 移除 {z_filtered} 点 (范围: [{z_min}, {z_max}])")
        
        # 2. XY距离过滤
        max_xy_dist = filters.get('max_xy_dist_m', 2.0)
        xy_dist = np.sqrt(points_3d[:, 0]**2 + points_3d[:, 1]**2)
        xy_mask = xy_dist <= max_xy_dist
        
        if debug:
            xy_filtered = np.sum(~xy_mask)
            if xy_filtered > 0:
                rospy.loginfo(f"XY距离过滤: 移除 {xy_filtered} 点 (最大距离: {max_xy_dist}m)")
        
        # 3. 合并所有过滤条件
        final_mask = z_mask & xy_mask
        filtered_points = points_3d[final_mask]
        
        final_count = len(filtered_points)
        filtered_count = initial_count - final_count
        
        if debug and filtered_count > 0:
            rospy.loginfo(f"总过滤: {initial_count} -> {final_count} ({filtered_count} 点被移除)")
        
        return filtered_points.tolist()
    
    def get_filter_stats(self, points_3d):
        """获取点云统计信息"""
        if not points_3d:
            return {}
        
        points_array = np.array(points_3d)
        
        stats = {
            'count': len(points_array),
            'x_range': [float(np.min(points_array[:, 0])), float(np.max(points_array[:, 0]))],
            'y_range': [float(np.min(points_array[:, 1])), float(np.max(points_array[:, 1]))],
            'z_range': [float(np.min(points_array[:, 2])), float(np.max(points_array[:, 2]))],
            'xy_dist_max': float(np.max(np.sqrt(points_array[:, 0]**2 + points_array[:, 1]**2)))
        }
        
        return stats


class PointCloudGenerator:
    """点云生成器"""
    
    def __init__(self):
        pass
    
    def create_pointcloud2_msg(self, points_3d, colors=None, frame_id="base_link", timestamp=None):
        """创建PointCloud2消息"""
        if not points_3d:
            # 返回空点云
            empty_cloud = PointCloud2()
            empty_cloud.header.frame_id = frame_id
            empty_cloud.header.stamp = timestamp or rospy.Time.now()
            empty_cloud.height = 1
            empty_cloud.width = 0
            return empty_cloud
        
        points_array = np.array(points_3d)
        num_points = len(points_array)
        
        # 创建PointCloud2消息
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = frame_id
        cloud_msg.header.stamp = timestamp or rospy.Time.now()
        cloud_msg.height = 1
        cloud_msg.width = num_points
        
        # 点云数据结构
        if colors is not None and len(colors) == num_points:
            # 带颜色的点云 (XYZRGB)
            cloud_msg.fields = [
                # XYZ
                {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},   # FLOAT32
                {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
                {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
                # RGB
                {'name': 'rgb', 'offset': 12, 'datatype': 7, 'count': 1}
            ]
            cloud_msg.point_step = 16  # 4 bytes * 4 fields
            
            # 打包点云数据
            point_data = []
            colors_array = np.array(colors)
            
            for i in range(num_points):
                x, y, z = points_array[i]
                
                # 颜色处理
                if colors_array.ndim == 2 and colors_array.shape[1] >= 3:
                    r, g, b = colors_array[i][:3].astype(np.uint8)
                else:
                    r, g, b = 255, 255, 255  # 默认白色
                
                # RGB打包为32位浮点数
                rgb_packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
                
                point_data.append(struct.pack('ffff', x, y, z, rgb_packed))
                
        else:
            # 纯XYZ点云
            cloud_msg.fields = [
                {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
                {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1}, 
                {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1}
            ]
            cloud_msg.point_step = 12  # 4 bytes * 3 fields
            
            # 打包点云数据
            point_data = []
            for point in points_array:
                x, y, z = point
                point_data.append(struct.pack('fff', x, y, z))
        
        cloud_msg.row_step = cloud_msg.point_step * num_points
        cloud_msg.data = b''.join(point_data)
        cloud_msg.is_dense = True
        
        return cloud_msg


class DepthProjector:
    """深度投影器 - 基于depth_projector_core的封装"""
    
    def __init__(self, method, config_path, camera_name='hand_camera'):
        self.method = method
        self.config_path = config_path
        self.camera_name = camera_name
        
        try:
            # 初始化核心投影器
            self.core_projector = CameraDepthProjector(
                camera_name, method, config_path
            )
            
            # 初始化管理器
            self.filter_manager = FilterManager()
            self.point_cloud_gen = PointCloudGenerator()
            
            rospy.loginfo(f"✅ 投影器初始化成功")
            rospy.loginfo(f"  方法: {method}")
            rospy.loginfo(f"  相机坐标系: {self.core_projector.camera_frame}")
            rospy.loginfo(f"  内参: fx={self.core_projector.K[0,0]:.1f}")
            
        except Exception as e:
            rospy.logerr(f"❌ 投影器初始化失败: {e}")
            raise
    
    def extract_grasp_points_2d(self, detections_2d):
        """从2D检测中提取抓取点"""
        grasp_points = []
        detection_indices = []
        grasp_indices = []
        
        for det_idx, detection in enumerate(detections_2d.detections):
            for grasp_idx, grasp in enumerate(detection.grasp_poses):
                # 抓取中心点
                u = int(grasp.x)
                v = int(grasp.y)
                grasp_points.append([u, v])
                detection_indices.append(det_idx)
                grasp_indices.append(grasp_idx)
        
        return np.array(grasp_points), detection_indices, grasp_indices
    
    def fill_depth_holes(self, grasp_points_2d, depth_image, max_radius=20):
        """使用邻域中位数填补深度空洞"""
        filled_depths = []
        
        for u, v in grasp_points_2d:
            if 0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]:
                depth_value = depth_image[v, u]
                
                # 如果深度为0，尝试邻域搜索
                if depth_value == 0:
                    for radius in [5, 10, 15, max_radius]:
                        u_min = max(0, u - radius)
                        u_max = min(depth_image.shape[1], u + radius + 1)
                        v_min = max(0, v - radius)
                        v_max = min(depth_image.shape[0], v + radius + 1)
                        
                        neighborhood = depth_image[v_min:v_max, u_min:u_max]
                        valid_depths = neighborhood[neighborhood > 0]
                        
                        if len(valid_depths) > 0:
                            depth_value = np.median(valid_depths)
                            rospy.logdebug(f"邻域深度填补: u={u}, v={v}, 半径={radius}, 深度={depth_value:.0f}mm")
                            break
                
                filled_depths.append(depth_value)
            else:
                # 点在图像外
                filled_depths.append(0)
        
        return np.array(filled_depths)
    
    def project_detections_to_3d(self, detections_2d, depth_image, filters, enable_color=False, color_image=None):
        """将2D检测投影到3D空间"""
        if not detections_2d.detections:
            return None
        
        try:
            # 1. 提取2D抓取点
            grasp_points_2d, det_indices, grasp_indices = self.extract_grasp_points_2d(detections_2d)
            
            if len(grasp_points_2d) == 0:
                rospy.logwarn("没有找到2D抓取点")
                return None
            
            # 2. 深度过滤和填补
            depths_raw = []
            valid_points = []
            valid_det_indices = []
            valid_grasp_indices = []
            
            min_depth = filters.get('min_depth_mm', 200)
            max_depth = filters.get('max_depth_mm', 2000)
            
            for i, (u, v) in enumerate(grasp_points_2d):
                if 0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]:
                    depth_value = depth_image[v, u]
                    
                    # 深度填补
                    if depth_value == 0:
                        filled_depth = self.fill_depth_holes([(u, v)], depth_image)[0]
                        if filled_depth > 0:
                            depth_value = filled_depth
                    
                    # 深度范围过滤
                    if min_depth <= depth_value <= max_depth:
                        depths_raw.append(depth_value)
                        valid_points.append([u, v])
                        valid_det_indices.append(det_indices[i])
                        valid_grasp_indices.append(grasp_indices[i])
            
            if not valid_points:
                rospy.logwarn("没有有效的深度点")
                return None
            
            # 3. 3D投影
            valid_points = np.array(valid_points)
            depths_m = np.array(depths_raw) / 1000.0  # 转换为米
            
            points_3d = self.core_projector.project_depth_to_3d(
                valid_points, depths_m
            )
            
            if points_3d is None or len(points_3d) == 0:
                rospy.logwarn("3D投影失败")
                return None
            
            # 4. 3D空间过滤
            filtered_points = self.filter_manager.apply_filters(
                points_3d, filters, debug=True
            )
            
            if not filtered_points:
                rospy.logwarn("所有3D点都被过滤")
                return None
            
            # 5. 提取颜色信息（如果启用）
            colors = None
            if enable_color and color_image is not None:
                colors = []
                for i, (u, v) in enumerate(valid_points):
                    if i < len(filtered_points):  # 确保索引对应
                        if 0 <= u < color_image.shape[1] and 0 <= v < color_image.shape[0]:
                            color = color_image[v, u]  # BGR格式
                            colors.append([color[2], color[1], color[0]])  # 转为RGB
                        else:
                            colors.append([255, 255, 255])  # 默认白色
                
                colors = colors[:len(filtered_points)]  # 截断到实际点数
            
            # 6. 创建3D检测消息
            detections_3d = self._create_3d_detections(
                detections_2d, filtered_points, valid_det_indices, valid_grasp_indices
            )
            
            # 7. 创建点云消息
            pointcloud = self.point_cloud_gen.create_pointcloud2_msg(
                filtered_points, colors, "base_link", detections_2d.header.stamp
            )
            
            # 8. 统计信息
            stats = self.filter_manager.get_filter_stats(filtered_points)
            
            return {
                'detections_3d': detections_3d,
                'pointcloud': pointcloud,
                'stats': stats,
                'num_points': len(filtered_points)
            }
            
        except Exception as e:
            rospy.logerr(f"3D投影处理失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _create_3d_detections(self, detections_2d, points_3d, det_indices, grasp_indices):
        """创建3D检测消息"""
        detections_3d = GraspDetectionArray3D()
        detections_3d.header = detections_2d.header
        detections_3d.header.frame_id = "base_link"
        
        # 按检测对象分组3D点
        detection_groups = {}
        for i, det_idx in enumerate(det_indices):
            if i < len(points_3d):  # 确保索引有效
                if det_idx not in detection_groups:
                    detection_groups[det_idx] = []
                detection_groups[det_idx].append((i, grasp_indices[i]))
        
        # 为每个检测对象创建3D检测
        for det_idx, point_list in detection_groups.items():
            if det_idx >= len(detections_2d.detections):
                continue
                
            detection_2d = detections_2d.detections[det_idx]
            detection_3d = GraspDetection3D()
            
            # 复制2D信息
            detection_3d.bbox_2d = detection_2d.bbox
            detection_3d.score = detection_2d.score
            
            # 添加缺失的属性以兼容可视化器
            from geometry_msgs.msg import Point
            detection_3d.bbox_3d = type('BBox3D', (), {})()  # 创建简单的bbox_3d对象
            detection_3d.bbox_3d.corners = []  # 初始化为空corners列表
            
            # 如果有检测名称，复制过来
            if hasattr(detection_2d, 'name'):
                detection_3d.name = detection_2d.name
            else:
                detection_3d.name = f"detection_{len(detections_3d_msg.detections)}"
            
            # 添加3D抓取姿态
            for point_idx, grasp_idx in point_list:
                if grasp_idx >= len(detection_2d.grasp_poses):
                    continue
                
                grasp_2d = detection_2d.grasp_poses[grasp_idx]
                grasp_3d = GraspPose3D()
                
                # 3D位置
                point_3d = points_3d[point_idx]
                grasp_3d.position.x = point_3d[0]
                grasp_3d.position.y = point_3d[1]
                grasp_3d.position.z = point_3d[2]
                
                # 姿态（简化为单位四元数）
                grasp_3d.orientation.w = 1.0
                
                # 抓取参数
                grasp_3d.width = grasp_2d.width / 1000.0  # 转换为米
                grasp_3d.confidence = grasp_2d.confidence
                
                detection_3d.grasp_poses_3d.append(grasp_3d)
            
            detections_3d.detections.append(detection_3d)
        
        return detections_3d


class ProjectionManager:
    """投影管理器 - 管理所有投影方法"""
    
    def __init__(self, projection_configs, config_path=""):
        self.projection_configs = projection_configs
        self.config_path = config_path
        self.projectors = {}  # Lazy initialization - 延迟到实际使用时创建
        self.projector_instances = {}  # 每个相机-方法组合的投影器实例
        
        rospy.loginfo(f"📐 投影管理器初始化完成，支持方法: {list(projection_configs.keys())}")
    
    def get_projector(self, method):
        """获取指定投影器（lazy initialization）"""
        if method not in self.projectors:
            # Lazy initialization - 首次使用时创建
            if method not in self.projection_configs:
                raise ValueError(f"Projection method {method} not found")
            
            try:
                config = self.projection_configs[method]
                method_config_path = config.get('config_path', self.config_path)
                self.projectors[method] = DepthProjector(method, method_config_path)
                rospy.loginfo(f"✅ 投影器 {method} 延迟初始化成功")
                
            except Exception as e:
                rospy.logerr(f"❌ 投影器 {method} 初始化失败: {e}")
                raise
        
        return self.projectors[method]
    
    def get_projector_for_camera(self, method, camera_name):
        """获取指定相机的投影器（更精确的lazy initialization）"""
        key = f"{method}_{camera_name}"
        
        if key not in self.projector_instances:
            if method not in self.projection_configs:
                raise ValueError(f"Projection method {method} not found")
            
            try:
                config = self.projection_configs[method]
                method_config_path = config.get('config_path', self.config_path)
                self.projector_instances[key] = DepthProjector(
                    method, method_config_path, f"{camera_name}_camera"
                )
                rospy.loginfo(f"✅ 相机专用投影器 {method}:{camera_name} 延迟初始化成功")
                
            except Exception as e:
                rospy.logerr(f"❌ 相机专用投影器 {method}:{camera_name} 初始化失败: {e}")
                raise
        
        return self.projector_instances[key]
    
    def get_available_methods(self):
        """获取所有可用投影方法"""
        return list(self.projectors.keys())


if __name__ == '__main__':
    # 测试代码
    rospy.init_node('projection_manager_test')
    
    # 测试配置
    test_config = {
        'urdf': {
            'config_path': ''
        },
        'calibration': {
            'config_path': '/home/agilex/MobileManipulator/src/robot_drivers/camera_driver/config'
        }
    }
    
    try:
        manager = ProjectionManager(test_config)
        
        # 获取URDF投影器
        urdf_projector = manager.get_projector('urdf')
        
        rospy.loginfo(f"投影器测试完成，可用方法: {manager.get_available_methods()}")
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"测试失败: {e}")