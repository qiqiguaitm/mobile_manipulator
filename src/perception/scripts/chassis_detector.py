#!/usr/bin/env python3
"""
Chassis Camera Object Detection Node
Uses ReferingAPI for object detection with text queries
"""

import rospy
from sensor_msgs.msg import Image, CompressedImage
from perception.msg import ObjectDetection, ObjectDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue
import time
from percept_dino_api_test import ReferingAPI
from mmengine.config import Config


class ChassisDetectorNode:
    def __init__(self):
        rospy.init_node('chassis_detector_node')
        
        # Parameters
        self.rate = rospy.get_param('~rate', 0.2)  # 0.2 Hz based on test
        self.query_text = rospy.get_param('~query_text', 'object')
        self.visualize = rospy.get_param('~visualize', True)
        
        # Initialize API
        cfg = Config()
        cfg.uri = r'/v2/task/dino_xseek/detection'
        cfg.status_uri = r'/v2/task_status'
        cfg.token = 'a5b2c267a8346db8a141eba6162c3adc'
        self.api = ReferingAPI(cfg)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Queue for async processing
        self.image_queue = queue.Queue(maxsize=2)
        
        # Publishers
        self.det_pub = rospy.Publisher(
            '/perception/chassis/detections', 
            ObjectDetectionArray, 
            queue_size=1
        )
        
        if self.visualize:
            self.vis_pub = rospy.Publisher(
                '/perception/chassis/vis_image',
                CompressedImage,
                queue_size=1
            )
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            '/camera/chassis/color/image_raw',
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # Start processing thread
        self.running = True
        self.process_thread = threading.Thread(target=self.process_loop)
        self.process_thread.start()
        
        rospy.loginfo(f"Chassis detector started, rate: {self.rate} Hz")
    
    def image_callback(self, msg):
        """Store latest image in queue"""
        if not self.image_queue.full():
            self.image_queue.put(msg)
    
    def process_loop(self):
        """Main processing loop"""
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown() and self.running:
            try:
                # Get latest image
                if not self.image_queue.empty():
                    # Clear queue and get latest
                    while not self.image_queue.empty():
                        img_msg = self.image_queue.get()
                    
                    self.process_image(img_msg)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Process loop error: {e}")
                continue
    
    def process_image(self, img_msg):
        """Process single image"""
        try:
            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # API call
            start_time = time.time()
            task = self.api.forward(rgb=cv_image, text=self.query_text)
            api_time = time.time() - start_time
            
            # Parse results
            result = task.result
            
            # Create detection message
            det_msg = ObjectDetectionArray()
            det_msg.header = img_msg.header
            
            if result and 'objects' in result:
                for obj in result['objects']:
                    det = ObjectDetection()
                    det.bbox = obj['bbox']  # [x1, y1, x2, y2]
                    det.confidence = 0.8  # Default confidence
                    det.label = self.query_text
                    det_msg.detections.append(det)
            
            # Publish detections
            self.det_pub.publish(det_msg)
            
            # Visualize if enabled
            if self.visualize:
                vis_image = self.visualize_detections(cv_image, det_msg.detections)
                self.publish_vis_image(vis_image, img_msg.header)
            
            rospy.loginfo(f"Detected {len(det_msg.detections)} objects in {api_time:.2f}s")
            
        except Exception as e:
            rospy.logerr(f"Processing error: {e}")
    
    def visualize_detections(self, image, detections):
        """Draw detection results on image"""
        vis_img = image.copy()
        
        for det in detections:
            # Draw bbox
            x1, y1, x2, y2 = [int(x) for x in det.bbox]
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{det.label}: {det.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            
            # Background for text
            cv2.rectangle(vis_img, 
                         (x1, y1 - label_size[1] - 4),
                         (x1 + label_size[0], y1),
                         (0, 255, 0), -1)
            
            # Text
            cv2.putText(vis_img, label,
                       (x1, y1 - 2),
                       cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (0, 0, 0), 2)
        
        # Add status text
        status = f"Chassis Detector | Objects: {len(detections)} | Query: '{self.query_text}'"
        cv2.putText(vis_img, status,
                   (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX,
                   0.7, (0, 255, 0), 2)
        
        return vis_img
    
    def publish_vis_image(self, image, header):
        """Publish visualization image as compressed"""
        try:
            # Compress image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, compressed = cv2.imencode('.jpg', image, encode_param)
            
            # Create message
            msg = CompressedImage()
            msg.header = header
            msg.format = "jpeg"
            msg.data = compressed.tobytes()
            
            self.vis_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Vis publish error: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        if self.process_thread.is_alive():
            self.process_thread.join(timeout=2.0)


def main():
    try:
        node = ChassisDetectorNode()
        rospy.spin()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()