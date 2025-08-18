#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
验证控制相关节点的200Hz频率设置
"""

import rospy
import subprocess
import time
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class RateVerifier:
    def __init__(self):
        rospy.init_node('rate_verifier', anonymous=True)
        
        # 记录各个topic的频率
        self.topic_rates = {}
        self.topic_last_time = {}
        self.topic_msg_count = {}
        
        # 订阅关键topic
        self.subscribers = []
        
        # 控制相关的topics
        control_topics = [
            ('/joint_states', JointState),
            ('/arm_controller/command', JointState),
            ('/motion_smoother/smoothed_command', JointState),
        ]
        
        for topic, msg_type in control_topics:
            sub = rospy.Subscriber(topic, msg_type, 
                                 lambda msg, t=topic: self.topic_callback(msg, t))
            self.subscribers.append(sub)
        
        rospy.loginfo("开始监控控制节点频率...")
        
    def topic_callback(self, msg, topic_name):
        """记录topic消息频率"""
        current_time = rospy.Time.now().to_sec()
        
        if topic_name not in self.topic_last_time:
            self.topic_last_time[topic_name] = current_time
            self.topic_msg_count[topic_name] = 0
            return
        
        self.topic_msg_count[topic_name] += 1
        
        # 每秒计算一次频率
        time_diff = current_time - self.topic_last_time[topic_name]
        if time_diff >= 1.0:
            rate = self.topic_msg_count[topic_name] / time_diff
            self.topic_rates[topic_name] = rate
            self.topic_msg_count[topic_name] = 0
            self.topic_last_time[topic_name] = current_time
    
    def check_node_rates(self):
        """检查节点内部设置的频率"""
        rospy.loginfo("\n=== 检查节点频率配置 ===")
        
        # 检查参数服务器上的频率设置
        params_to_check = [
            '/motion_smoother/control_frequency',
            '/continuous_collision_checker/check_frequency',
            '/safety_check_rate',
        ]
        
        for param in params_to_check:
            if rospy.has_param(param):
                value = rospy.get_param(param)
                status = "✓" if value >= 200 else "✗"
                rospy.loginfo(f"{status} {param}: {value} Hz")
            else:
                rospy.loginfo(f"? {param}: 未设置")
        
        # 检查源代码中的频率设置
        rospy.loginfo("\n=== 源代码频率设置 ===")
        
        files_to_check = [
            ('/home/agilex/MobileManipulator/src/arm_controller/scripts/piper_ctrl_single_node.py', 
             'Rate(200)', '主控制器'),
            ('/home/agilex/MobileManipulator/src/arm_planner/scripts/safety/continuous_collision_checker.py',
             'self.check_frequency = 200', '碰撞检测器'),
            ('/home/agilex/MobileManipulator/src/arm_planner/scripts/safety/motion_smoother.py',
             'Rate(200)', '运动平滑器'),
        ]
        
        for file_path, pattern, name in files_to_check:
            try:
                with open(file_path, 'r') as f:
                    content = f.read()
                    if pattern in content:
                        rospy.loginfo(f"✓ {name}: 已设置为200Hz")
                    else:
                        rospy.logwarn(f"✗ {name}: 未找到200Hz设置")
            except Exception as e:
                rospy.logerr(f"无法读取 {name}: {e}")
    
    def print_results(self):
        """打印监控结果"""
        rospy.loginfo("\n=== Topic发布频率监控 ===")
        for topic, rate in self.topic_rates.items():
            status = "✓" if rate >= 190 else "✗"  # 允许10Hz误差
            rospy.loginfo(f"{status} {topic}: {rate:.1f} Hz")
        
        if not self.topic_rates:
            rospy.logwarn("未检测到任何topic消息")

def main():
    verifier = RateVerifier()
    
    # 先检查静态配置
    verifier.check_node_rates()
    
    # 监控5秒钟的topic频率
    rospy.loginfo("\n监控topic频率5秒...")
    start_time = time.time()
    
    while not rospy.is_shutdown() and (time.time() - start_time) < 5:
        rospy.sleep(0.1)
    
    # 打印结果
    verifier.print_results()
    
    rospy.loginfo("\n频率验证完成！")

if __name__ == '__main__':
    main()