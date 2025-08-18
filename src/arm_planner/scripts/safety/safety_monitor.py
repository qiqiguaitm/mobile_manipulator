#!/usr/bin/env python3
"""
安全监控器 - 记录和分析安全事件
"""

import rospy
from std_msgs.msg import String, Float32MultiArray
import os
import datetime
import json

class SafetyMonitor:
    def __init__(self):
        rospy.init_node('safety_monitor')
        
        # 日志配置
        self.log_path = rospy.get_param('~log_path', '/var/log/robot_safety/')
        self.max_log_size_mb = rospy.get_param('~max_log_size_mb', 100)
        
        # 创建日志目录
        os.makedirs(self.log_path, exist_ok=True)
        
        # 日志文件
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_path, f"safety_log_{timestamp}.json")
        
        # 事件缓冲
        self.events = []
        
        # 订阅器
        self.safety_level_sub = rospy.Subscriber(
            '/safety_level', String, self.safety_level_callback
        )
        self.collision_dist_sub = rospy.Subscriber(
            '/collision_distances', Float32MultiArray, self.collision_callback
        )
        
        rospy.loginfo(f"Safety monitor started, logging to {self.log_file}")
    
    def safety_level_callback(self, msg):
        """记录安全等级变化"""
        event = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'safety_level_change',
            'data': msg.data
        }
        self.log_event(event)
    
    def collision_callback(self, msg):
        """记录碰撞距离"""
        if len(msg.data) > 0 and msg.data[0] < 0.05:  # 只记录小于50mm的情况
            event = {
                'timestamp': rospy.Time.now().to_sec(),
                'type': 'collision_warning',
                'min_distance': msg.data[0],
                'all_distances': list(msg.data)
            }
            self.log_event(event)
    
    def log_event(self, event):
        """记录事件到文件"""
        self.events.append(event)
        
        # 写入文件
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(event) + '\n')
        except Exception as e:
            rospy.logerr(f"Failed to write log: {e}")
        
        # 检查文件大小
        self.check_log_size()
    
    def check_log_size(self):
        """检查并轮转日志文件"""
        try:
            size_mb = os.path.getsize(self.log_file) / 1024 / 1024
            if size_mb > self.max_log_size_mb:
                # 轮转日志
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                new_file = os.path.join(self.log_path, f"safety_log_{timestamp}.json")
                self.log_file = new_file
                rospy.loginfo(f"Log rotated to {new_file}")
        except:
            pass
    
    def shutdown(self):
        """关闭时生成统计报告"""
        if self.events:
            rospy.loginfo(f"Safety monitor recorded {len(self.events)} events")
            
            # 统计各类事件
            event_types = {}
            for event in self.events:
                event_type = event.get('type', 'unknown')
                event_types[event_type] = event_types.get(event_type, 0) + 1
            
            rospy.loginfo("Event statistics:")
            for event_type, count in event_types.items():
                rospy.loginfo(f"  {event_type}: {count}")
    
    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = SafetyMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass