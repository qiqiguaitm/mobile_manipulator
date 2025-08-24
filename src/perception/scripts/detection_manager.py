#!/usr/bin/python3
"""
检测管理器 - 统一管理多种检测API
支持GraspAnything、ReferringDino等检测算法
"""

import rospy
import numpy as np
import threading
import time
import json
import cv2
from abc import ABC, abstractmethod
from sensor_msgs.msg import CompressedImage, Image
from perception.msg import (GraspDetection, GraspDetectionArray, 
                           GraspPose, TouchingPoint)
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from mmengine.config import Config
# from percept_dino_api_test import ReferingAPI  # 使用HTTP API替代


class RateLimiter:
    """频率限制器"""
    
    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.min_interval = 1.0 / rate_hz if rate_hz > 0 else 0
        self.last_process_time = 0
    
    def can_process(self):
        """检查是否可以处理（基于频率限制）"""
        current_time = time.time()
        if current_time - self.last_process_time >= self.min_interval:
            self.last_process_time = current_time
            return True
        return False
    
    def get_remaining_time(self):
        """获取剩余等待时间"""
        current_time = time.time()
        elapsed = current_time - self.last_process_time
        return max(0, self.min_interval - elapsed)


class BaseDetector(ABC):
    """检测器基类"""
    
    def __init__(self, name, config):
        self.name = name
        self.config = config
        self.rate_limiter = RateLimiter(config.get('rate_hz', 1.0))
        self.enabled = config.get('enabled', True)
        self.bridge = CvBridge()
        
        if self.enabled:
            self._initialize()
            rospy.loginfo(f"🔍 检测器 {name} 初始化完成 (频率: {config.get('rate_hz', 1.0)}Hz)")
        else:
            rospy.loginfo(f"⏸️ 检测器 {name} 已禁用")
    
    @abstractmethod
    def _initialize(self):
        """初始化检测器（子类实现）"""
        pass
    
    @abstractmethod
    def _detect_impl(self, image, **kwargs):
        """实际检测逻辑（子类实现）"""
        pass
    
    def detect(self, image, header=None, **kwargs):
        """统一检测接口"""
        if not self.enabled:
            return None
        
        if not self.rate_limiter.can_process():
            rospy.logdebug(f"检测器 {self.name} 频率限制，跳过此帧")
            return None
        
        try:
            start_time = time.time()
            result = self._detect_impl(image, **kwargs)
            detect_time = time.time() - start_time
            
            if result is not None:
                # 设置消息头
                if hasattr(result, 'header') and header is not None:
                    result.header = header
                
                rospy.loginfo(f"检测器 {self.name} 检测完成: {detect_time:.2f}s")
            
            return result
            
        except Exception as e:
            rospy.logerr(f"检测器 {self.name} 错误: {e}")
            return None


class GraspAnythingDetector(BaseDetector):
    """GraspAnything检测器"""
    
    def _initialize(self):
        """初始化GraspAnything API"""
        try:
            # 动态导入以避免缺少依赖时崩溃
            from percept_dino_api_test import GraspAnythingAPI
            
            # 配置API
            cfg = Config()
            cfg.server_list = self.config.get('endpoint', 
                '/home/agilex/MobileManipulator/src/perception/scripts/server_grasp.json')
            cfg.model_name = self.config.get('model_name', 'full')
            
            self.api = GraspAnythingAPI(cfg)
            self.use_touching_points = self.config.get('use_touching_points', True)
            self.visualize = self.config.get('visualize', False)
            
            rospy.loginfo(f"✅ GraspAnything API 初始化成功")
            rospy.loginfo(f"  模型: {cfg.model_name}")
            rospy.loginfo(f"  接触点: {self.use_touching_points}")
            
        except ImportError as e:
            rospy.logerr(f"❌ 无法导入GraspAnything API: {e}")
            self.enabled = False
        except Exception as e:
            rospy.logerr(f"❌ GraspAnything API 初始化失败: {e}")
            self.enabled = False
    
    def _detect_impl(self, image, **kwargs):
        """GraspAnything检测实现"""
        if not hasattr(self, 'api'):
            return None
        
        try:
            # 调用API
            objs, padded_img = self.api.forward(
                rgb=image,
                use_touching_points=self.use_touching_points
            )
            
            # 转换为ROS消息
            grasp_msg = GraspDetectionArray()
            grasp_msg.header.stamp = rospy.Time.now()
            
            # 处理每个检测到的对象
            for obj in objs[0]:  # objs是[[obj1, obj2, ...]]格式
                grasp_det = self._create_grasp_detection(obj)
                grasp_msg.detections.append(grasp_det)
            
            # 可选：生成可视化图像
            vis_image = None
            if self.visualize and len(objs[0]) > 0:
                vis_image = self.api.vis(objs, image, padded_img, 
                                       img_fp='', to_save=False)
            
            return {
                'detections': grasp_msg,
                'vis_image': vis_image,
                'num_objects': len(grasp_msg.detections)
            }
            
        except Exception as e:
            rospy.logerr(f"GraspAnything检测失败: {e}")
            return None
    
    def _create_grasp_detection(self, obj):
        """将API对象转换为ROS消息"""
        det = GraspDetection()
        
        # 边界框
        det.bbox = obj['dt_bbox']
        
        # 检测置信度
        det.score = obj.get('dt_score', 0.5)
        
        # RLE掩码
        if 'dt_rle' in obj:
            det.rle = json.dumps(obj['dt_rle'])
        
        # 分割掩码（压缩存储）
        if 'dt_mask' in obj:
            mask = obj['dt_mask'].astype(np.uint8) * 255
            _, compressed = cv2.imencode('.png', mask)
            mask_msg = CompressedImage()
            mask_msg.format = "png"
            mask_msg.data = compressed.tobytes()
            det.mask = mask_msg
        
        # Reid特征
        if 'reid_fea' in obj:
            det.reid_feature = obj['reid_fea']
        
        # 抓取姿态
        if 'affs' in obj and 'scores' in obj:
            for aff, score in zip(obj['affs'], obj['scores']):
                pose = GraspPose()
                pose.x = aff[0]
                pose.y = aff[1] 
                pose.width = aff[2]
                pose.height = aff[3]
                pose.theta = aff[4]
                pose.confidence = score
                det.grasp_poses.append(pose)
        
        # 接触点
        if 'touching_points' in obj:
            for tp_pair in obj['touching_points']:
                tp = TouchingPoint()
                tp.point1 = Point(x=tp_pair[0][0], y=tp_pair[0][1], z=0)
                tp.point2 = Point(x=tp_pair[1][0], y=tp_pair[1][1], z=0)
                det.touching_points.append(tp)
        
        return det


class ReferringDetector(BaseDetector):
    """Referring DINO检测器（使用真实DDS云API）"""
    
    def _initialize(self):
        """初始化Referring DINO真实API"""
        try:
            # 动态导入以避免缺少依赖时崩溃
            from percept_dino_api_test import ReferingAPI
            from mmengine.config import Config
            
            rospy.loginfo(f"🔍 ReferringDetector 初始化 (DDS云API)")
            
            # 配置真实的DDS云API
            cfg = Config()
            cfg.uri = r'/v2/task/dino_xseek/detection'
            cfg.status_uri = r'/v2/task_status'
            cfg.token = 'c4cdacb48bc4d1a1a335c88598a18e8c'  # 使用percept_dino_api_test.py中的token
            
            self.api = ReferingAPI(cfg)
            self.default_query = self.config.get('default_query', '物体')
            
            rospy.loginfo(f"  ✅ DDS云API初始化成功")
            rospy.loginfo(f"  Token: {cfg.token[:8]}...")
            rospy.loginfo(f"  默认查询: {self.default_query}")
            
        except ImportError as e:
            rospy.logerr(f"❌ 无法导入ReferingAPI: {e}")
            rospy.logerr("请确保percept_dino_api_test.py和相关依赖可用")
            self.enabled = False
        except Exception as e:
            rospy.logerr(f"❌ ReferringDetector API初始化失败: {e}")
            self.enabled = False
    
    def _detect_impl(self, image, text_query=None, **kwargs):
        """Referring检测实现（使用真实DDS云API）"""
        if not hasattr(self, 'api'):
            rospy.logwarn("ReferringDetector API未初始化")
            return None
            
        # 使用默认查询或传入的查询
        query = text_query or self.default_query
        
        rospy.loginfo(f"ReferringDetector DDS检测: query='{query}', 图像尺寸={image.shape}")
        
        try:
            # 调用真实的DDS云API
            task = self.api.forward(rgb=image, text=query)
            api_result = task.result if hasattr(task, 'result') else {}
            
            rospy.loginfo(f"DDS API返回结果: {api_result}")
            
            # 创建检测结果
            grasp_msg = GraspDetectionArray()
            grasp_msg.header.stamp = rospy.Time.now()
            
            # 解析API结果 - DDS API返回格式: {'objects': [{'bbox': [x1, y1, x2, y2]}]}
            if 'objects' in api_result and len(api_result['objects']) > 0:
                for obj in api_result['objects']:
                    if 'bbox' in obj:
                        det = GraspDetection()
                        
                        # DDS API返回的bbox格式: [x1, y1, x2, y2] 
                        bbox = obj['bbox']
                        det.bbox = [
                            int(bbox[0]),                    # x
                            int(bbox[1]),                    # y
                            int(bbox[2] - bbox[0]),         # width
                            int(bbox[3] - bbox[1])          # height
                        ]
                        
                        # 检测置信度（DDS API可能不返回置信度，使用默认值）
                        det.score = obj.get('confidence', 0.8)
                        
                        # 为referring检测生成简单的抓取姿态（在边界框中心）
                        center_x = int((bbox[0] + bbox[2]) / 2)
                        center_y = int((bbox[1] + bbox[3]) / 2)
                        
                        pose = GraspPose()
                        pose.x = center_x
                        pose.y = center_y
                        pose.width = min(det.bbox[2], det.bbox[3]) * 0.8  # 80%的边界框尺寸
                        pose.height = 30  # 固定高度
                        pose.theta = 0  # 水平方向
                        pose.confidence = det.score
                        det.grasp_poses.append(pose)
                        
                        grasp_msg.detections.append(det)
                        
                        # 添加name属性以兼容后续处理
                        det.name = f"object_{len(grasp_msg.detections)}"
                        
                        rospy.loginfo(f"  检测到对象: bbox={det.bbox}, score={det.score:.3f}")
            
            else:
                rospy.loginfo(f"ReferringDetector: DDS API没有检测到 '{query}'")
            
            # 创建可视化（即使没有检测到对象也创建）
            vis_image = None
            if self.config.get('visualize', False):
                vis_image = image.copy()
                
                if len(grasp_msg.detections) > 0:
                    # 有检测结果时绘制边界框和抓取点
                    for det in grasp_msg.detections:
                        # 绘制边界框
                        cv2.rectangle(vis_image, 
                                     (det.bbox[0], det.bbox[1]),
                                     (det.bbox[0] + det.bbox[2], det.bbox[1] + det.bbox[3]),
                                     (0, 255, 255), 3)  # 黄色边框表示Referring检测
                        
                        # 添加文本
                        cv2.putText(vis_image, f"Referring: '{query}'", 
                                   (det.bbox[0], det.bbox[1] - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                        
                        # 绘制抓取点
                        for pose in det.grasp_poses:
                            cv2.circle(vis_image, (int(pose.x), int(pose.y)), 5, (255, 255, 0), -1)
                            cv2.circle(vis_image, (int(pose.x), int(pose.y)), 8, (0, 255, 255), 2)
                else:
                    # 没有检测结果时显示查询信息
                    h, w = vis_image.shape[:2]
                    cv2.putText(vis_image, f"Referring Query: '{query}' - No objects found", 
                               (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            return {
                'detections': grasp_msg,
                'vis_image': vis_image,
                'num_objects': len(grasp_msg.detections)
            }
            
        except Exception as e:
            rospy.logerr(f"ReferringDetector DDS API调用失败: {e}")
            import traceback
            if rospy.get_param('/unified_perception/debug_mode', False):
                traceback.print_exc()
                
            # 返回空结果
            grasp_msg = GraspDetectionArray()
            grasp_msg.header.stamp = rospy.Time.now()
            return {
                'detections': grasp_msg,
                'vis_image': None,
                'num_objects': 0
            }


class DetectionManager:
    """检测管理器 - 管理所有检测器"""
    
    def __init__(self, detection_configs):
        self.detectors = {}
        
        # 初始化各种检测器
        for api_name, config in detection_configs.items():
            if config.get('enabled', False):
                try:
                    if api_name == 'grasp_anything':
                        self.detectors[api_name] = GraspAnythingDetector(api_name, config)
                    elif api_name == 'referring_dino':
                        self.detectors[api_name] = ReferringDetector(api_name, config)
                    else:
                        rospy.logwarn(f"未知检测器类型: {api_name}")
                        
                except Exception as e:
                    rospy.logerr(f"检测器 {api_name} 初始化失败: {e}")
            else:
                rospy.loginfo(f"⏸️ 检测器 {api_name} 已禁用")
        
        rospy.loginfo(f"🔍 检测管理器初始化完成，活跃检测器: {list(self.detectors.keys())}")
    
    def get_detector(self, detector_name):
        """获取指定检测器"""
        if detector_name in self.detectors:
            return self.detectors[detector_name]
        else:
            raise ValueError(f"Detector {detector_name} not found or not enabled")
    
    def get_active_detectors(self):
        """获取所有活跃检测器名称"""
        return list(self.detectors.keys())
    
    def detect_all(self, image, header=None):
        """使用所有活跃检测器进行检测"""
        results = {}
        
        for name, detector in self.detectors.items():
            if detector.enabled:
                result = detector.detect(image, header)
                results[name] = result
        
        return results


class VisualizationPublisher:
    """可视化发布器"""
    
    def __init__(self, vis_topics):
        self.bridge = CvBridge()
        self.publishers = {}
        
        # 创建可视化发布器
        for topic_name, topic_path in vis_topics.items():
            if topic_path:
                if 'compressed' in topic_name.lower():
                    self.publishers[topic_name] = rospy.Publisher(
                        topic_path, CompressedImage, queue_size=1
                    )
                else:
                    self.publishers[topic_name] = rospy.Publisher(
                        topic_path, Image, queue_size=1
                    )
    
    def publish_visualization(self, image, header, topics_to_publish):
        """发布可视化图像"""
        if image is None:
            return
        
        for topic_name in topics_to_publish:
            if topic_name in self.publishers:
                try:
                    if 'compressed' in topic_name.lower():
                        # 发布压缩图像
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        _, compressed = cv2.imencode('.jpg', image, encode_param)
                        
                        compressed_msg = CompressedImage()
                        compressed_msg.header = header
                        compressed_msg.format = "jpeg"
                        compressed_msg.data = compressed.tobytes()
                        self.publishers[topic_name].publish(compressed_msg)
                    else:
                        # 发布原始图像
                        raw_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                        raw_msg.header = header
                        self.publishers[topic_name].publish(raw_msg)
                        
                except Exception as e:
                    rospy.logerr(f"可视化发布失败 {topic_name}: {e}")


if __name__ == '__main__':
    # 测试代码
    rospy.init_node('detection_manager_test')
    
    # 测试配置
    test_config = {
        'grasp_anything': {
            'endpoint': '/home/agilex/MobileManipulator/src/perception/scripts/server_grasp.json',
            'model_name': 'full',
            'rate_hz': 0.5,
            'use_touching_points': True,
            'visualize': True,
            'enabled': True
        },
        'referring_dino': {
            'endpoint': 'http://localhost:5000/referring',
            'model_name': 'dino', 
            'rate_hz': 1.0,
            'enabled': False
        }
    }
    
    try:
        manager = DetectionManager(test_config)
        
        # 获取GraspAnything检测器
        grasp_detector = manager.get_detector('grasp_anything')
        
        # 创建测试图像（纯色图像）
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 128
        
        rospy.loginfo("开始检测测试...")
        
        rate = rospy.Rate(0.2)  # 0.2Hz测试
        while not rospy.is_shutdown():
            result = grasp_detector.detect(test_image)
            
            if result:
                rospy.loginfo(f"检测结果: {result['num_objects']} 个对象")
            else:
                rospy.loginfo("检测器暂不可用或频率受限")
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"测试失败: {e}")
