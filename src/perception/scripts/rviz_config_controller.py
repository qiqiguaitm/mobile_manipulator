#!/usr/bin/python3
"""
RViz配置控制器
根据运行时配置动态控制RViz显示组件的启用/禁用
"""

import rospy
import yaml
import os
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class RvizConfigController:
    """RViz配置控制器"""
    
    def __init__(self):
        rospy.init_node('rviz_config_controller', anonymous=True)
        
        # 获取配置文件路径
        self.config_file = rospy.get_param(
            '~config_file',
            '/tmp/perception_unified_runtime.yaml'
        )
        
        # 加载配置
        self.config = self.load_config()
        
        # 分析启用的组件
        self.enabled_cameras = self.analyze_enabled_cameras()
        self.enabled_detectors = self.analyze_enabled_detectors()
        
        # 发布RViz显示控制信息
        self.publish_rviz_config()
        
        rospy.loginfo(f"🎮 RViz配置控制器启动")
        rospy.loginfo(f"  启用的相机: {self.enabled_cameras}")
        rospy.loginfo(f"  启用的检测器: {self.enabled_detectors}")
    
    def load_config(self):
        """加载运行时配置"""
        try:
            if not os.path.exists(self.config_file):
                rospy.logwarn(f"配置文件不存在: {self.config_file}")
                return {}
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"加载配置失败: {e}")
            return {}
    
    def analyze_enabled_cameras(self):
        """分析启用的相机"""
        enabled = set()
        
        # 从流水线配置中分析
        if 'pipelines' in self.config:
            for pipeline_name, pipeline_config in self.config['pipelines'].items():
                if pipeline_config.get('enabled', False):
                    camera = pipeline_config.get('camera')
                    if camera:
                        enabled.add(camera)
        
        return list(enabled)
    
    def analyze_enabled_detectors(self):
        """分析启用的检测器"""
        enabled = {}
        
        # 从流水线配置中分析每个相机启用的检测器
        if 'pipelines' in self.config:
            for pipeline_name, pipeline_config in self.config['pipelines'].items():
                if pipeline_config.get('enabled', False):
                    camera = pipeline_config.get('camera')
                    detector = pipeline_config.get('detector')
                    
                    if camera and detector:
                        if camera not in enabled:
                            enabled[camera] = set()
                        
                        # 映射检测器名称到类型
                        if 'grasp' in detector.lower():
                            enabled[camera].add('grasp')
                        if 'refer' in detector.lower() or 'dino' in detector.lower():
                            enabled[camera].add('refer')
        
        # 转换集合为列表
        for camera in enabled:
            enabled[camera] = list(enabled[camera])
        
        return enabled
    
    def publish_rviz_config(self):
        """发布RViz配置参数"""
        # 设置RViz参数（供其他节点使用）
        rospy.set_param('/rviz_config/enabled_cameras', self.enabled_cameras)
        rospy.set_param('/rviz_config/enabled_detectors', self.enabled_detectors)
        
        # 为每个相机设置详细参数
        for camera in ['hand', 'chassis', 'top']:
            camera_enabled = camera in self.enabled_cameras
            rospy.set_param(f'/rviz_config/{camera}/enabled', camera_enabled)
            
            if camera_enabled and camera in self.enabled_detectors:
                detectors = self.enabled_detectors[camera]
                rospy.set_param(f'/rviz_config/{camera}/grasp_enabled', 'grasp' in detectors)
                rospy.set_param(f'/rviz_config/{camera}/refer_enabled', 'refer' in detectors)
            else:
                rospy.set_param(f'/rviz_config/{camera}/grasp_enabled', False)
                rospy.set_param(f'/rviz_config/{camera}/refer_enabled', False)
        
        # 打印配置摘要
        rospy.loginfo("📊 RViz显示配置:")
        for camera in ['hand', 'chassis', 'top']:
            if rospy.get_param(f'/rviz_config/{camera}/enabled', False):
                grasp = "✓" if rospy.get_param(f'/rviz_config/{camera}/grasp_enabled', False) else "✗"
                refer = "✓" if rospy.get_param(f'/rviz_config/{camera}/refer_enabled', False) else "✗"
                rospy.loginfo(f"  {camera}: Grasp[{grasp}] Refer[{refer}]")
    
    def get_rviz_config_recommendation(self):
        """获取RViz配置建议"""
        recommendations = []
        
        # 根据启用的组件给出建议
        for camera in self.enabled_cameras:
            recommendations.append(f"启用 '{camera.title()} Camera' 显示组")
            
            if camera in self.enabled_detectors:
                for detector in self.enabled_detectors[camera]:
                    if detector == 'grasp':
                        recommendations.append(f"  - 启用 {camera.title()} Grasp 显示")
                    elif detector == 'refer':
                        recommendations.append(f"  - 启用 {camera.title()} Referring 显示")
        
        return recommendations
    
    def print_instructions(self):
        """打印RViz配置说明"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("📌 RViz显示组件配置建议:")
        rospy.loginfo("="*50)
        
        recommendations = self.get_rviz_config_recommendation()
        for rec in recommendations:
            rospy.loginfo(rec)
        
        rospy.loginfo("\n💡 在RViz中手动调整:")
        rospy.loginfo("1. 展开左侧Displays面板")
        rospy.loginfo("2. 根据上述建议启用/禁用相应的Camera组")
        rospy.loginfo("3. 每个Camera组内可独立控制Grasp和Referring显示")
        rospy.loginfo("="*50 + "\n")
    
    def run(self):
        """运行控制器"""
        # 打印配置说明
        self.print_instructions()
        
        # 定期刷新（可选）
        rate = rospy.Rate(0.1)  # 10秒刷新一次
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = RvizConfigController()
        controller.run()
    except rospy.ROSInterruptException:
        pass