#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
修复joint_states时间戳冲突问题
通过创建一个中间节点来同步和过滤重复的joint_states消息
"""

import rospy
from sensor_msgs.msg import JointState
import threading

class JointStateFilter:
    def __init__(self):
        rospy.init_node('joint_state_filter')
        
        # 参数
        self.min_time_gap = rospy.get_param('~min_time_gap', 0.005)  # 最小时间间隔 5ms
        self.queue_size = rospy.get_param('~queue_size', 1)
        
        # 状态变量
        self.last_publish_time = 0
        self.lock = threading.Lock()
        self.latest_msg = None
        
        # 订阅原始joint_states
        self.sub = rospy.Subscriber('/joint_states_raw', JointState, self.joint_state_callback,
                                   queue_size=10, tcp_nodelay=True)
        
        # 发布过滤后的joint_states
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=self.queue_size)
        
        rospy.loginfo(f"Joint State Filter启动 - 最小间隔: {self.min_time_gap*1000}ms")
        
    def joint_state_callback(self, msg):
        """处理接收到的joint_state消息"""
        current_time = rospy.Time.now().to_sec()
        
        with self.lock:
            # 检查时间间隔
            time_since_last = current_time - self.last_publish_time
            
            if time_since_last >= self.min_time_gap:
                # 更新时间戳确保单调递增
                msg.header.stamp = rospy.Time.now()
                
                # 发布消息
                self.pub.publish(msg)
                self.last_publish_time = current_time
                
                # 记录最新消息
                self.latest_msg = msg
            else:
                # 时间间隔太短，跳过这条消息
                rospy.logdebug(f"跳过消息 - 间隔太短: {time_since_last*1000:.1f}ms")

def main():
    filter_node = JointStateFilter()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass