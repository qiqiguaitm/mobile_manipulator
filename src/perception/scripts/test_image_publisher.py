#!/usr/bin/env python3
"""
Test image publisher for perception system testing
Publishes the same test image to both chassis and hand camera topics
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class TestImagePublisher:
    def __init__(self):
        rospy.init_node('test_image_publisher')
        
        # Parameters
        self.image_path = rospy.get_param('~image_path', 
                                         '/home/agilex/MobileManipulator/src/perception/scripts/dino_test.jpg')
        self.rate = rospy.get_param('~rate', 1.0)
        
        # Load image
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            rospy.logerr(f"Failed to load image: {self.image_path}")
            return
        
        # Resize to 640x480
        self.image_640 = cv2.resize(self.image, (640, 480))
        rospy.loginfo(f"Loaded test image: {self.image_640.shape}")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.chassis_pub = rospy.Publisher(
            '/camera/chassis/color/image_raw',
            Image,
            queue_size=1
        )
        
        self.hand_pub = rospy.Publisher(
            '/camera/hand/color/image_raw',
            Image,
            queue_size=1
        )
        
        rospy.loginfo(f"Publishing test images at {self.rate} Hz")
    
    def run(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Create image message
            img_msg = self.bridge.cv2_to_imgmsg(self.image_640, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera"
            
            # Publish to both topics
            self.chassis_pub.publish(img_msg)
            self.hand_pub.publish(img_msg)
            
            rate.sleep()


def main():
    try:
        publisher = TestImagePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()