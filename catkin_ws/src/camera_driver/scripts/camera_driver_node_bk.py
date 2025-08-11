#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError

class MultiCameraDriverNode:
    def __init__(self):
        rospy.init_node('multi_camera_driver_node')
        
        # Get parameters
        self.device_id = rospy.get_param('~device_id', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        
        # Publishers for multiple cameras
        self.camera_pubs = {}
        self.camera_info_pubs = {}
        
        # Create publishers for each camera position
        camera_positions = ['chassis', 'hand', 'top']
        for position in camera_positions:
            topic_name = f'/camera/{position}/dual/color/image_raw'
            info_topic_name = f'/camera/{position}/camera_info'
            self.camera_pubs[position] = rospy.Publisher(topic_name, Image, queue_size=1)
            self.camera_info_pubs[position] = rospy.Publisher(info_topic_name, CameraInfo, queue_size=1)
        
        # Also maintain the original single camera topics for compatibility
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Open camera device
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera device %d", self.device_id)
            raise RuntimeError("Failed to open camera device")
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Initialize camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.width = self.image_width
        self.camera_info_msg.height = self.image_height
        # These are example values - should be calibrated for your specific camera
        self.camera_info_msg.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
        self.camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info_msg.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        rospy.loginfo("Camera driver node initialized with device %d", self.device_id)
        
    def capture_and_publish(self):
        """Capture image from camera and publish it to all camera topics"""
        ret, cv_image = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image from camera")
            return
        
        try:
            # Convert OpenCV image to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header.stamp = rospy.Time.now()
            
            # Update camera info
            self.camera_info_msg.header.stamp = image_msg.header.stamp
            
            # Publish to all camera topics
            for position in self.camera_pubs:
                # Publish image
                image_msg.header.frame_id = f"{position}_camera_link"
                self.camera_pubs[position].publish(image_msg)
                
                # Publish camera info
                self.camera_info_msg.header.frame_id = f"{position}_camera_link"
                self.camera_info_pubs[position].publish(self.camera_info_msg)
            
            # Also publish to original single camera topics for compatibility
            image_msg.header.frame_id = "camera_link"
            self.image_pub.publish(image_msg)
            self.camera_info_msg.header.frame_id = "camera_link"
            self.camera_info_pub.publish(self.camera_info_msg)
            
            rospy.logdebug("Published camera images: %dx%d", cv_image.shape[1], cv_image.shape[0])
            
        except CvBridgeError as e:
            rospy.logerr("CV bridge error: %s", str(e))
        except Exception as e:
            rospy.logerr("Error publishing image: %s", str(e))
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.frame_rate)
        while not rospy.is_shutdown():
            self.capture_and_publish()
            rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        rospy.loginfo("Camera driver node shutdown")

if __name__ == '__main__':
    try:
        node = MultiCameraDriverNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Multi-camera driver node error: %s", str(e))