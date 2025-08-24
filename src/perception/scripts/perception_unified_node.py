#!/usr/bin/python3
"""
统一感知节点 - 整合多相机、多检测API、多TF链路的可配置感知系统
将原本分散的多个节点整合为单一节点，简化系统架构
"""

import rospy
import yaml
import os
import signal
import sys
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
import traceback

# 导入各管理器模块
from camera_manager import CameraManager
from detection_manager import DetectionManager
from projection_manager import ProjectionManager
from pipeline_manager import PipelineManager


class ConfigManager:
    """配置管理器 - 处理配置加载和验证"""
    
    def __init__(self, config_file=None):
        self.config_file = config_file or rospy.get_param(
            '~config_file', 
            '/home/agilex/MobileManipulator/src/perception/config/perception_unified.yaml'
        )
        self.config = {}
    
    def load_config(self):
        """加载配置文件"""
        try:
            rospy.loginfo(f"📄 加载配置文件: {self.config_file}")
            
            if not os.path.exists(self.config_file):
                raise FileNotFoundError(f"配置文件不存在: {self.config_file}")
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            
            # 验证配置
            self._validate_config()
            
            rospy.loginfo("✅ 配置文件加载成功")
            return self.config
            
        except Exception as e:
            rospy.logerr(f"❌ 配置文件加载失败: {e}")
            raise
    
    def _validate_config(self):
        """验证配置文件格式"""
        required_sections = ['cameras', 'detection_apis', 'projection_methods', 'pipelines']
        
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"配置文件缺少必需的段: {section}")
        
        # 验证流水线配置
        for pipeline_name, pipeline_config in self.config['pipelines'].items():
            # 检查引用的组件是否存在
            camera_name = pipeline_config.get('camera')
            if camera_name and camera_name not in self.config['cameras']:
                raise ValueError(f"流水线 {pipeline_name} 引用了不存在的相机: {camera_name}")
            
            detector_name = pipeline_config.get('detector')
            if detector_name and detector_name not in self.config['detection_apis']:
                raise ValueError(f"流水线 {pipeline_name} 引用了不存在的检测器: {detector_name}")
            
            projector_name = pipeline_config.get('projector')
            if projector_name and projector_name not in self.config['projection_methods']:
                raise ValueError(f"流水线 {pipeline_name} 引用了不存在的投影器: {projector_name}")
    
    def get_config(self):
        """获取当前配置"""
        return self.config
    
    def reload_config(self):
        """重新加载配置"""
        old_config = self.config.copy()
        try:
            self.load_config()
            rospy.loginfo("🔄 配置文件重新加载成功")
            return True
        except Exception as e:
            rospy.logerr(f"配置文件重新加载失败: {e}")
            self.config = old_config  # 恢复旧配置
            return False


class StatusReporter:
    """状态报告器 - 定期报告系统状态"""
    
    def __init__(self, unified_node):
        self.unified_node = unified_node
        self.status_pub = rospy.Publisher(
            '/unified_perception/status', 
            String, 
            queue_size=1
        )
        
        # 定期状态报告
        self.status_timer = rospy.Timer(
            rospy.Duration(30.0),  # 每30秒报告一次
            self._publish_status
        )
    
    def _publish_status(self, event):
        """发布状态信息"""
        try:
            status_info = {
                'node_status': 'running' if self.unified_node.running else 'stopped',
                'active_cameras': len(self.unified_node.camera_mgr.cameras) if hasattr(self.unified_node, 'camera_mgr') else 0,
                'active_detectors': len(self.unified_node.detect_mgr.detectors) if hasattr(self.unified_node, 'detect_mgr') else 0,
                'active_projectors': len(self.unified_node.project_mgr.projectors) if hasattr(self.unified_node, 'project_mgr') else 0,
                'running_pipelines': len(self.unified_node.pipeline_mgr.running_pipelines) if hasattr(self.unified_node, 'pipeline_mgr') else 0,
            }
            
            # 获取流水线状态
            if hasattr(self.unified_node, 'pipeline_mgr'):
                pipeline_status = self.unified_node.pipeline_mgr.get_pipeline_status()
                status_info['pipelines'] = pipeline_status
            
            status_msg = String()
            status_msg.data = yaml.dump(status_info, default_flow_style=False)
            self.status_pub.publish(status_msg)
            
            rospy.loginfo(f"📊 系统状态: {status_info['running_pipelines']} 个流水线运行中")
            
        except Exception as e:
            rospy.logerr(f"状态报告失败: {e}")


class UnifiedPerceptionNode:
    """统一感知节点 - 主要的节点类"""
    
    def __init__(self):
        # 初始化节点
        rospy.init_node('unified_perception_node', anonymous=False)
        
        # 状态标志
        self.running = False
        self.initialized = False
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        rospy.loginfo("="*60)
        rospy.loginfo("🚀 统一感知节点启动中...")
        rospy.loginfo("="*60)
        
        try:
            # 1. 加载配置
            self._load_configuration()
            
            # 2. 设置ROS参数
            self._setup_ros_params()
            
            # 3. 初始化各管理器
            self._initialize_managers()
            
            # 4. 启动流水线
            self._start_pipelines()
            
            # 5. 设置服务和状态报告
            self._setup_services()
            
            # 6. 启动完成
            self.running = True
            self.initialized = True
            
            rospy.loginfo("="*60)
            rospy.loginfo("✅ 统一感知节点启动完成!")
            rospy.loginfo(f"  活跃相机: {len(self.camera_mgr.cameras)}")
            rospy.loginfo(f"  活跃检测器: {len(self.detect_mgr.detectors)}")
            rospy.loginfo(f"  活跃投影器: {len(self.project_mgr.projectors)}")
            rospy.loginfo(f"  运行流水线: {len(self.pipeline_mgr.running_pipelines)}")
            rospy.loginfo("="*60)
            
        except Exception as e:
            rospy.logerr(f"❌ 统一感知节点初始化失败: {e}")
            if rospy.get_param('~debug_mode', False):
                traceback.print_exc()
            sys.exit(1)
    
    def _load_configuration(self):
        """加载配置"""
        self.config_mgr = ConfigManager()
        self.config = self.config_mgr.load_config()
        
        # 记录配置信息
        rospy.loginfo("📋 配置概览:")
        rospy.loginfo(f"  相机配置: {len(self.config['cameras'])} 个")
        rospy.loginfo(f"  检测API: {len(self.config['detection_apis'])} 个")
        rospy.loginfo(f"  投影方法: {len(self.config['projection_methods'])} 个")
        rospy.loginfo(f"  流水线: {len(self.config['pipelines'])} 个")
    
    def _setup_ros_params(self):
        """设置ROS参数"""
        global_settings = self.config.get('global_settings', {})
        
        # 设置全局参数
        for key, value in global_settings.items():
            param_name = f'/unified_perception/{key}'
            rospy.set_param(param_name, value)
            rospy.logdebug(f"设置参数: {param_name} = {value}")
    
    def _initialize_managers(self):
        """初始化各管理器"""
        rospy.loginfo("🔧 初始化管理器...")
        
        # 全局设置
        global_settings = self.config.get('global_settings', {})
        
        # 相机管理器
        rospy.loginfo("📷 初始化相机管理器...")
        self.camera_mgr = CameraManager(
            self.config['cameras'], 
            global_settings
        )
        
        # 检测管理器  
        rospy.loginfo("🔍 初始化检测管理器...")
        self.detect_mgr = DetectionManager(
            self.config['detection_apis']
        )
        
        # 投影管理器
        rospy.loginfo("📐 初始化投影管理器...")
        # 传递配置路径给投影管理器
        config_path = self.config.get('projection_methods', {}).get('calibration', {}).get('config_path', '')
        self.project_mgr = ProjectionManager(
            self.config['projection_methods'],
            config_path
        )
        
        # 流水线管理器
        rospy.loginfo("🚀 初始化流水线管理器...")
        self.pipeline_mgr = PipelineManager(
            self.config['pipelines'],
            self.camera_mgr,
            self.detect_mgr, 
            self.project_mgr
        )
    
    def _start_pipelines(self):
        """启动流水线"""
        rospy.loginfo("▶️ 启动所有流水线...")
        started_count = self.pipeline_mgr.start_all()
        
        if started_count == 0:
            rospy.logwarn("⚠️ 没有流水线启动成功")
        else:
            rospy.loginfo(f"✅ 成功启动 {started_count} 个流水线")
    
    def _setup_services(self):
        """设置ROS服务"""
        # 重启服务
        self.restart_service = rospy.Service(
            '/unified_perception/restart',
            Empty,
            self._restart_service_callback
        )
        
        # 重新加载配置服务
        self.reload_config_service = rospy.Service(
            '/unified_perception/reload_config',
            Empty,
            self._reload_config_service_callback
        )
        
        # 状态报告器
        self.status_reporter = StatusReporter(self)
        
        rospy.loginfo("🔧 ROS服务设置完成")
    
    def _restart_service_callback(self, req):
        """重启服务回调"""
        rospy.loginfo("🔄 收到重启请求...")
        
        try:
            # 停止所有流水线
            self.pipeline_mgr.stop_all()
            
            # 重新加载配置
            if self.config_mgr.reload_config():
                self.config = self.config_mgr.get_config()
                
                # 重新初始化管理器
                self._initialize_managers()
                
                # 重新启动流水线
                self._start_pipelines()
                
                rospy.loginfo("✅ 重启完成")
            else:
                rospy.logerr("❌ 重启失败: 配置重新加载失败")
                
        except Exception as e:
            rospy.logerr(f"❌ 重启失败: {e}")
        
        return EmptyResponse()
    
    def _reload_config_service_callback(self, req):
        """重新加载配置服务回调"""
        rospy.loginfo("🔄 收到重新加载配置请求...")
        
        success = self.config_mgr.reload_config()
        if success:
            self.config = self.config_mgr.get_config()
            rospy.loginfo("✅ 配置重新加载完成（需要重启以生效）")
        else:
            rospy.logerr("❌ 配置重新加载失败")
        
        return EmptyResponse()
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        rospy.loginfo(f"📶 收到信号 {signum}，准备关闭...")
        self.shutdown()
    
    def run(self):
        """运行主循环"""
        if not self.initialized:
            rospy.logerr("❌ 节点未正确初始化，无法运行")
            return
        
        rospy.loginfo("🎯 统一感知节点运行中...")
        
        try:
            # ROS主循环
            rospy.spin()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("📡 ROS中断信号接收")
        except Exception as e:
            rospy.logerr(f"❌ 运行时错误: {e}")
            if rospy.get_param('~debug_mode', False):
                traceback.print_exc()
        finally:
            self.shutdown()
    
    def shutdown(self):
        """关闭节点"""
        if not self.running:
            return
        
        rospy.loginfo("🛑 正在关闭统一感知节点...")
        self.running = False
        
        try:
            # 停止所有流水线
            if hasattr(self, 'pipeline_mgr'):
                self.pipeline_mgr.stop_all()
            
            # 关闭相机
            if hasattr(self, 'camera_mgr'):
                self.camera_mgr.shutdown()
            
            # 停止状态报告
            if hasattr(self, 'status_reporter') and hasattr(self.status_reporter, 'status_timer'):
                self.status_reporter.status_timer.shutdown()
            
            rospy.loginfo("✅ 统一感知节点已关闭")
            
        except Exception as e:
            rospy.logerr(f"❌ 关闭过程中出现错误: {e}")


def main():
    """主函数"""
    try:
        # 创建统一感知节点
        node = UnifiedPerceptionNode()
        
        # 运行节点
        node.run()
        
    except Exception as e:
        rospy.logerr(f"❌ 主函数异常: {e}")
        if rospy.get_param('/unified_perception/debug_mode', False):
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()