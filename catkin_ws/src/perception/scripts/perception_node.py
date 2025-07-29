#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        
        # Load topic configurations
        topics = rospy.get_param('topics')
        
        # Publishers for perception outputs
        self.object_pub = rospy.Publisher(topics['outputs']['object_list']['name'], PoseStamped, queue_size=10)
        
        # Subscribers for camera inputs
        self.bridge = CvBridge()
        self.top_camera_sub = rospy.Subscriber(topics['inputs']['top_camera']['name'], Image, self.top_camera_callback)
        self.hand_camera_sub = rospy.Subscriber(topics['inputs']['hand_camera']['name'], Image, self.hand_camera_callback)
        self.chassis_camera_sub = rospy.Subscriber(topics['inputs']['chassis_camera']['name'], Image, self.chassis_camera_callback)
        
        # Create a fixed pose message for continuous publishing
        self.fixed_pose = PoseStamped()
        self.fixed_pose.header.frame_id = "fixed_frame"
        self.fixed_pose.pose.position.x = 1.0
        self.fixed_pose.pose.position.y = 2.0
        self.fixed_pose.pose.position.z = 3.0
        
        rospy.loginfo("Perception node initialized with camera subscribers")

    def top_camera_callback(self, msg):
        """Process top camera images"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received top camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except Exception as e:
            rospy.logerr("Error processing top camera image: %s", str(e))

    def hand_camera_callback(self, msg):
        """Process hand camera images"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received hand camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except Exception as e:
            rospy.logerr("Error processing hand camera image: %s", str(e))

    def chassis_camera_callback(self, msg):
        """Process chassis camera images"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Process image (placeholder for actual perception logic)
            rospy.logdebug("Received chassis camera image: %dx%d", cv_image.shape[1], cv_image.shape[0])
        except Exception as e:
            rospy.logerr("Error processing chassis camera image: %s", str(e))

    def run(self):
        """Main loop - continuously publish fixed output"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Update timestamp
            self.fixed_pose.header.stamp = rospy.Time.now()
            # Publish fixed pose
            self.object_pub.publish(self.fixed_pose)
            rospy.logdebug("Published fixed pose: x=%f, y=%f, z=%f", 
                          self.fixed_pose.pose.position.x,
                          self.fixed_pose.pose.position.y,
                          self.fixed_pose.pose.position.z)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PerceptionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass