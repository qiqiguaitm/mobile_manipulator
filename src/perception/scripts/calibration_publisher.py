#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
发布相机标定信息到/calibration/{camera_name}话题
"""

import rospy
from sensor_msgs.msg import CameraInfo
import threading

class CalibrationPublisher:
    """
    将camera_info重新发布到/calibration/xxx话题
    """
    
    def __init__(self):
        rospy.init_node('calibration_publisher')
        
        # 相机列表
        self.cameras = ['hand', 'chassis', 'top']
        
        # 创建发布器和订阅器
        self.publishers = {}
        self.subscribers = []
        self.latest_info = {}
        self.lock = threading.Lock()
        
        for camera in self.cameras:
            # 创建发布器
            pub_topic = f"/calibration/{camera}"
            self.publishers[camera] = rospy.Publisher(
                pub_topic, CameraInfo, queue_size=1, latch=True)
            rospy.loginfo(f"Publishing calibration to {pub_topic}")
            
            # 订阅深度相机信息（用于3D投影的主要标定）
            sub_topic = f"/camera/{camera}/depth/camera_info"
            sub = rospy.Subscriber(sub_topic, CameraInfo, 
                                  self.camera_info_callback, camera)
            self.subscribers.append(sub)
            rospy.loginfo(f"Subscribing to {sub_topic}")
            
    def camera_info_callback(self, msg, camera_name):
        """接收并转发camera_info"""
        with self.lock:
            # 保存最新的标定信息
            self.latest_info[camera_name] = msg
            
            # 发布到/calibration/xxx
            self.publishers[camera_name].publish(msg)
            
    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            with self.lock:
                # 定期重新发布（latch模式下这不是必需的，但保险起见）
                for camera, info in self.latest_info.items():
                    self.publishers[camera].publish(info)
                    
            rate.sleep()
            
        rospy.loginfo("Calibration publisher shutting down")


if __name__ == "__main__":
    try:
        publisher = CalibrationPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass