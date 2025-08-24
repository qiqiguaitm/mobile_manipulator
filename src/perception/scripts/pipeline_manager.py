#!/usr/bin/python3
"""
流水线管理器 - 管理相机-检测-投影的处理流水线
每个流水线独立运行，支持不同组合的并行处理
"""

import rospy
import threading
import time
import queue
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from perception.msg import GraspDetectionArray, GraspDetectionArray3D
from cv_bridge import CvBridge
import traceback


class PublisherSet:
    """发布器集合"""
    
    def __init__(self, output_config):
        self.bridge = CvBridge()
        self.publishers = {}
        
        # 创建各种发布器
        if 'detections_2d' in output_config and output_config['detections_2d']:
            self.publishers['detections_2d'] = rospy.Publisher(
                output_config['detections_2d'], 
                GraspDetectionArray, 
                queue_size=1
            )
        
        if 'detections_3d' in output_config and output_config['detections_3d']:
            self.publishers['detections_3d'] = rospy.Publisher(
                output_config['detections_3d'],
                GraspDetectionArray3D,
                queue_size=1
            )
        
        if 'pointcloud' in output_config and output_config['pointcloud']:
            self.publishers['pointcloud'] = rospy.Publisher(
                output_config['pointcloud'],
                PointCloud2,
                queue_size=1
            )
        
        if 'vis_image' in output_config and output_config['vis_image']:
            self.publishers['vis_image'] = rospy.Publisher(
                output_config['vis_image'],
                Image,
                queue_size=1
            )
        
        if 'vis_compressed' in output_config and output_config['vis_compressed']:
            self.publishers['vis_compressed'] = rospy.Publisher(
                output_config['vis_compressed'],
                CompressedImage,
                queue_size=1
            )
    
    def publish_results(self, results):
        """发布所有结果"""
        try:
            # 发布2D检测
            if 'detections_2d' in results and 'detections_2d' in self.publishers:
                self.publishers['detections_2d'].publish(results['detections_2d'])
            
            # 发布3D检测
            if 'detections_3d' in results and 'detections_3d' in self.publishers:
                self.publishers['detections_3d'].publish(results['detections_3d'])
            
            # 发布点云
            if 'pointcloud' in results and 'pointcloud' in self.publishers:
                self.publishers['pointcloud'].publish(results['pointcloud'])
            
            # 发布可视化图像
            if 'vis_image' in results and results['vis_image'] is not None:
                # 原始图像
                if 'vis_image' in self.publishers:
                    vis_msg = self.bridge.cv2_to_imgmsg(results['vis_image'], "bgr8")
                    vis_msg.header = results.get('header', vis_msg.header)
                    self.publishers['vis_image'].publish(vis_msg)
                
                # 压缩图像
                if 'vis_compressed' in self.publishers:
                    import cv2
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                    _, compressed = cv2.imencode('.jpg', results['vis_image'], encode_param)
                    
                    compressed_msg = CompressedImage()
                    compressed_msg.header = results.get('header', compressed_msg.header)
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = compressed.tobytes()
                    self.publishers['vis_compressed'].publish(compressed_msg)
            
        except Exception as e:
            rospy.logerr(f"发布结果失败: {e}")


class PipelineExecutor:
    """流水线执行器 - 执行单条流水线"""
    
    def __init__(self, name, config, camera_mgr, detect_mgr, project_mgr):
        self.name = name
        self.config = config
        self.enabled = config.get('enabled', True)
        
        if not self.enabled:
            rospy.loginfo(f"⏸️ 流水线 {name} 已禁用")
            return
        
        try:
            # 获取组件
            self.camera = camera_mgr.get_camera(config['camera'])
            self.detector = detect_mgr.get_detector(config['detector'])
            self.projector = project_mgr.get_projector_for_camera(
                config['projector'], config['camera']
            )
            
            # 创建发布器
            self.publisher = PublisherSet(config['outputs'])
            
            # 处理控制
            self.running = False
            self.thread = None
            
            rospy.loginfo(f"✅ 流水线 {name} 初始化成功")
            rospy.loginfo(f"  相机: {config['camera']}")
            rospy.loginfo(f"  检测器: {config['detector']}")
            rospy.loginfo(f"  投影器: {config['projector']}")
            
        except Exception as e:
            rospy.logerr(f"❌ 流水线 {name} 初始化失败: {e}")
            self.enabled = False
    
    def start(self):
        """启动流水线处理"""
        if not self.enabled:
            return
        
        if self.running:
            rospy.logwarn(f"流水线 {self.name} 已在运行")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._process_loop, daemon=True)
        self.thread.start()
        rospy.loginfo(f"🚀 流水线 {self.name} 启动")
    
    def stop(self):
        """停止流水线处理"""
        if self.running:
            self.running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=2.0)
            rospy.loginfo(f"🛑 流水线 {self.name} 停止")
    
    def _process_loop(self):
        """主处理循环"""
        # 根据检测器频率设置处理频率
        detector_rate = self.detector.rate_limiter.rate_hz
        rate = rospy.Rate(max(detector_rate * 1.2, 0.1))  # 稍高于检测器频率
        
        rospy.loginfo(f"流水线 {self.name} 开始处理 (频率: {detector_rate}Hz)")
        
        while not rospy.is_shutdown() and self.running:
            try:
                # 1. 获取同步的图像对
                color_image, depth_image, header = self.camera.get_synchronized_images()
                
                if color_image is None or depth_image is None:
                    rate.sleep()
                    continue
                
                # 2. 2D检测
                detection_result = self.detector.detect(color_image, header)
                
                if detection_result is None:
                    rate.sleep()
                    continue
                
                # 确保检测结果格式正确
                if not isinstance(detection_result, dict) or 'detections' not in detection_result:
                    rospy.logwarn(f"流水线 {self.name}: 检测结果格式错误，跳过")
                    rate.sleep()
                    continue
                    
                detections_2d = detection_result['detections']
                vis_image = detection_result.get('vis_image', None)
                num_objects = detection_result.get('num_objects', 0)
                
                # 即使没有检测到对象，也要发布可视化结果（特别是referring检测）
                if num_objects == 0:
                    rospy.logdebug(f"流水线 {self.name}: 没有检测到对象，发布空结果和可视化")
                    # 发布空的检测结果和可视化图像
                    results = {
                        'detections_2d': detections_2d,
                        'detections_3d': None,
                        'pointcloud': None,
                        'vis_image': vis_image or color_image,  # 使用检测器的可视化或原图
                        'header': header
                    }
                    self.publisher.publish_results(results)
                    rate.sleep()
                    continue
                
                # 3. 3D投影
                enable_color = vis_image is not None
                projection_result = self.projector.project_detections_to_3d(
                    detections_2d, 
                    depth_image, 
                    self.config['filters'],
                    enable_color=enable_color,
                    color_image=color_image if enable_color else None
                )
                
                if projection_result is None:
                    rospy.logwarn(f"流水线 {self.name}: 3D投影失败")
                    rate.sleep()
                    continue
                
                detections_3d = projection_result['detections_3d']
                pointcloud = projection_result['pointcloud']
                stats = projection_result['stats']
                num_points = projection_result['num_points']
                
                # 4. 发布结果
                results = {
                    'detections_2d': detections_2d,
                    'detections_3d': detections_3d,
                    'pointcloud': pointcloud,
                    'vis_image': vis_image,
                    'header': header
                }
                
                self.publisher.publish_results(results)
                
                # 5. 日志统计
                rospy.loginfo(f"✅ 流水线 {self.name}: 检测到 {num_objects} 对象，投影 {num_points} 点")
                if stats and rospy.get_param('/unified_perception/debug_mode', False):
                    z_range = stats.get('z_range', [0, 0])
                    rospy.loginfo(f"   Z范围: [{z_range[0]:.3f}, {z_range[1]:.3f}]m")
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"❌ 流水线 {self.name} 处理错误: {e}")
                if rospy.get_param('/unified_perception/debug_mode', False):
                    traceback.print_exc()
                
                # 短暂休息后继续
                time.sleep(0.5)


class SynchronizationHub:
    """同步中心 - 处理多流水线间的同步需求"""
    
    def __init__(self):
        self.sync_events = {}
        self.shared_data = {}
        self.lock = threading.Lock()
    
    def create_sync_group(self, group_name, pipeline_names):
        """创建同步组"""
        with self.lock:
            self.sync_events[group_name] = {
                'pipelines': set(pipeline_names),
                'ready': set(),
                'event': threading.Event()
            }
    
    def signal_ready(self, group_name, pipeline_name):
        """信号流水线就绪"""
        with self.lock:
            if group_name in self.sync_events:
                sync_group = self.sync_events[group_name]
                sync_group['ready'].add(pipeline_name)
                
                # 检查是否所有流水线都就绪
                if sync_group['ready'] == sync_group['pipelines']:
                    sync_group['event'].set()
                    sync_group['ready'].clear()
    
    def wait_sync(self, group_name, timeout=1.0):
        """等待同步"""
        if group_name in self.sync_events:
            return self.sync_events[group_name]['event'].wait(timeout)
        return False
    
    def share_data(self, key, data):
        """共享数据"""
        with self.lock:
            self.shared_data[key] = data
    
    def get_shared_data(self, key):
        """获取共享数据"""
        with self.lock:
            return self.shared_data.get(key, None)


class PipelineManager:
    """流水线管理器 - 管理所有流水线"""
    
    def __init__(self, pipeline_configs, camera_mgr, detect_mgr, project_mgr):
        self.pipelines = {}
        self.sync_hub = SynchronizationHub()
        self.running_pipelines = []
        
        # 创建所有流水线
        for name, config in pipeline_configs.items():
            try:
                pipeline = PipelineExecutor(
                    name, config, camera_mgr, detect_mgr, project_mgr
                )
                self.pipelines[name] = pipeline
                
                if pipeline.enabled:
                    self.running_pipelines.append(name)
                    
            except Exception as e:
                rospy.logerr(f"❌ 流水线 {name} 创建失败: {e}")
        
        rospy.loginfo(f"🚀 流水线管理器初始化完成")
        rospy.loginfo(f"  总流水线: {len(self.pipelines)}")
        rospy.loginfo(f"  活跃流水线: {len(self.running_pipelines)}")
        rospy.loginfo(f"  流水线列表: {self.running_pipelines}")
    
    def start_all(self):
        """启动所有活跃流水线"""
        started_count = 0
        
        for name in self.running_pipelines:
            try:
                self.pipelines[name].start()
                started_count += 1
            except Exception as e:
                rospy.logerr(f"启动流水线 {name} 失败: {e}")
        
        rospy.loginfo(f"🚀 启动了 {started_count}/{len(self.running_pipelines)} 个流水线")
        return started_count
    
    def stop_all(self):
        """停止所有流水线"""
        stopped_count = 0
        
        for name in self.running_pipelines:
            try:
                self.pipelines[name].stop()
                stopped_count += 1
            except Exception as e:
                rospy.logerr(f"停止流水线 {name} 失败: {e}")
        
        rospy.loginfo(f"🛑 停止了 {stopped_count}/{len(self.running_pipelines)} 个流水线")
        return stopped_count
    
    def start_pipeline(self, name):
        """启动指定流水线"""
        if name in self.pipelines:
            self.pipelines[name].start()
        else:
            raise ValueError(f"Pipeline {name} not found")
    
    def stop_pipeline(self, name):
        """停止指定流水线"""
        if name in self.pipelines:
            self.pipelines[name].stop()
        else:
            raise ValueError(f"Pipeline {name} not found")
    
    def get_pipeline_status(self):
        """获取所有流水线状态"""
        status = {}
        for name, pipeline in self.pipelines.items():
            status[name] = {
                'enabled': pipeline.enabled,
                'running': pipeline.running if hasattr(pipeline, 'running') else False
            }
        return status
    
    def get_sync_hub(self):
        """获取同步中心"""
        return self.sync_hub


if __name__ == '__main__':
    # 测试代码
    rospy.init_node('pipeline_manager_test')
    
    # 模拟测试配置
    from camera_manager import CameraManager
    from detection_manager import DetectionManager  
    from projection_manager import ProjectionManager
    
    # 简化的测试配置
    camera_config = {
        'hand': {
            'topics': {
                'color': '/camera/hand/color/image_raw',
                'depth': '/camera/hand/depth/image_rect_raw'
            },
            'intrinsics_source': 'realsense',
            'enabled': True
        }
    }
    
    detect_config = {
        'grasp_anything': {
            'rate_hz': 0.2,
            'enabled': True
        }
    }
    
    project_config = {
        'urdf': {
            'config_path': ''
        }
    }
    
    pipeline_config = {
        'test_pipeline': {
            'camera': 'hand',
            'detector': 'grasp_anything',
            'projector': 'urdf',
            'filters': {
                'min_depth_mm': 200,
                'max_depth_mm': 2000,
                'min_z_height_m': -0.3,
                'max_z_height_m': 1.5
            },
            'outputs': {
                'detections_2d': '/test/grasps',
                'detections_3d': '/test/grasps_3d'
            },
            'enabled': True
        }
    }
    
    try:
        # 创建管理器
        global_settings = {'debug_mode': True}
        camera_mgr = CameraManager(camera_config, global_settings)
        detect_mgr = DetectionManager(detect_config)
        project_mgr = ProjectionManager(project_config)
        
        # 创建流水线管理器
        pipeline_mgr = PipelineManager(
            pipeline_config, camera_mgr, detect_mgr, project_mgr
        )
        
        # 启动所有流水线
        pipeline_mgr.start_all()
        
        rospy.loginfo("流水线管理器测试运行中...")
        rospy.spin()
        
        # 停止所有流水线
        pipeline_mgr.stop_all()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"测试失败: {e}")
        traceback.print_exc()