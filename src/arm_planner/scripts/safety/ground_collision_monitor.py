#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import tf2_ros
import numpy as np

class GroundCollisionMonitor:
    def __init__(self):
        rospy.init_node('ground_collision_monitor')
        
        # 参数
        self.min_height_threshold = rospy.get_param('~min_height_threshold', -0.05)
        self.check_frequency = rospy.get_param('~check_frequency', 50.0)
        self.enable_emergency_stop = rospy.get_param('~enable_emergency_stop', True)
        self.warning_height_offset = rospy.get_param('~warning_height_offset', 0.05)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.ground_warning_pub = rospy.Publisher('/ground_collision_warning', Bool, queue_size=1)
        
        # 监控的链接
        self.monitored_links = [
            'link6', 'gripper_base', 
            'gripper_finger1', 'gripper_finger2'
        ]
        
        # 状态
        self.is_warning = False
        self.is_emergency = False
        
        rospy.loginfo("地面碰撞监控器已启动")
        rospy.loginfo(f"最小高度阈值: {self.min_height_threshold}m")
        rospy.loginfo(f"警告高度偏移: {self.warning_height_offset}m")
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.check_frequency), self.check_collision)
        
    def check_collision(self, event):
        """检查末端是否接近地面"""
        try:
            for link in self.monitored_links:
                try:
                    # 获取链接相对于base_link的变换
                    transform = self.tf_buffer.lookup_transform(
                        'base_link', link, rospy.Time(0), rospy.Duration(0.1)
                    )
                    
                    # 获取Z坐标
                    z_height = transform.transform.translation.z
                    
                    # 检查是否低于阈值
                    if z_height < self.min_height_threshold:
                        if not self.is_emergency:
                            rospy.logerr(f"紧急停止！{link}高度({z_height:.3f}m)低于阈值({self.min_height_threshold}m)")
                            self.is_emergency = True
                            if self.enable_emergency_stop:
                                self.emergency_stop_pub.publish(Bool(True))
                    
                    # 检查警告区域
                    elif z_height < (self.min_height_threshold + self.warning_height_offset):
                        if not self.is_warning:
                            rospy.logwarn(f"警告！{link}接近地面，高度: {z_height:.3f}m")
                            self.is_warning = True
                            self.ground_warning_pub.publish(Bool(True))
                    else:
                        # 恢复正常
                        if self.is_warning or self.is_emergency:
                            self.is_warning = False
                            self.is_emergency = False
                            self.ground_warning_pub.publish(Bool(False))
                            if self.enable_emergency_stop:
                                self.emergency_stop_pub.publish(Bool(False))
                                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    # TF查找失败，忽略
                    pass
                    
        except Exception as e:
            rospy.logerr(f"地面碰撞检查错误: {e}")

    def shutdown(self):
        """关闭时清理"""
        rospy.loginfo("地面碰撞监控器关闭")
        # 发送解除紧急停止信号
        if self.enable_emergency_stop:
            self.emergency_stop_pub.publish(Bool(False))
        self.ground_warning_pub.publish(Bool(False))

if __name__ == '__main__':
    try:
        monitor = GroundCollisionMonitor()
        rospy.on_shutdown(monitor.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass