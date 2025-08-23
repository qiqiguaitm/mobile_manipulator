#!/usr/bin/python3
"""
Hand Camera Grasp Detection Node  
Uses GraspAnythingAPI for grasp detection
"""

import rospy
from sensor_msgs.msg import Image, CompressedImage
from perception.msg import (GraspDetection, GraspDetectionArray, 
                           GraspPose, TouchingPoint)
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue
import time
import json
from percept_dino_api_test import GraspAnythingAPI
from mmengine.config import Config


class HandGraspDetectorNode:
    def __init__(self):
        rospy.init_node('hand_grasp_detector_node')
        
        # Parameters
        self.rate = rospy.get_param('~rate', 0.5)  # 0.5 Hz based on test
        self.visualize = rospy.get_param('~visualize', True)
        self.use_touching_points = rospy.get_param('~use_touching_points', True)
        
        # Initialize API
        cfg = Config()
        cfg.server_list = rospy.get_param('~server_list', 
            '/home/agilex/MobileManipulator/src/perception/scripts/server_grasp.json')
        cfg.model_name = rospy.get_param('~model_name', 'full')
        self.api = GraspAnythingAPI(cfg)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Queue for async processing
        self.image_queue = queue.Queue(maxsize=2)
        
        # Publishers
        self.grasp_pub = rospy.Publisher(
            '/perception/hand/grasps',
            GraspDetectionArray,
            queue_size=1
        )
        
        if self.visualize:
            # Compressed image for bandwidth efficiency
            self.vis_pub_compressed = rospy.Publisher(
                '/perception/hand/vis_image/compressed',
                CompressedImage,
                queue_size=1
            )
            # Raw image for RViz visualization
            self.vis_pub_raw = rospy.Publisher(
                '/perception/hand/vis_image',
                Image,
                queue_size=1
            )
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            '/camera/hand/color/image_raw',
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # Start processing thread
        self.running = True
        self.process_thread = threading.Thread(target=self.process_loop)
        self.process_thread.start()
        
        rospy.loginfo(f"Hand grasp detector started, rate: {self.rate} Hz")
    
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
            objs, padded_img = self.api.forward(
                rgb=cv_image, 
                use_touching_points=self.use_touching_points
            )
            api_time = time.time() - start_time
            
            # Create grasp message
            grasp_msg = GraspDetectionArray()
            grasp_msg.header = img_msg.header
            
            # Process each detected object
            for obj in objs[0]:  # objs is [[obj1, obj2, ...]]
                grasp_det = self.create_grasp_detection(obj)
                grasp_msg.detections.append(grasp_det)
            
            # Publish grasps
            self.grasp_pub.publish(grasp_msg)
            
            # Visualize if enabled
            if self.visualize and len(objs[0]) > 0:
                vis_image = self.api.vis(objs, cv_image, padded_img, 
                                         img_fp='', to_save=False)
                self.publish_vis_image(vis_image, img_msg.header)
            
            rospy.loginfo(f"Detected {len(grasp_msg.detections)} graspable objects in {api_time:.2f}s")
            
        except Exception as e:
            rospy.logerr(f"Processing error: {e}")
    
    def create_grasp_detection(self, obj):
        """Convert API object to ROS message"""
        det = GraspDetection()
        
        # Bounding box
        det.bbox = obj['dt_bbox']
        
        # Detection score
        det.score = obj.get('dt_score', 0.5)
        
        # RLE mask
        if 'dt_rle' in obj:
            det.rle = json.dumps(obj['dt_rle'])
        
        # Segmentation mask (compress it)
        if 'dt_mask' in obj:
            mask = obj['dt_mask'].astype(np.uint8) * 255
            _, compressed = cv2.imencode('.png', mask)
            mask_msg = CompressedImage()
            mask_msg.format = "png"
            mask_msg.data = compressed.tobytes()
            det.mask = mask_msg
        
        # Reid feature
        if 'reid_fea' in obj:
            det.reid_feature = obj['reid_fea']
        
        # Grasp poses
        if 'affs' in obj and 'scores' in obj:
            for aff, score in zip(obj['affs'], obj['scores']):
                pose = GraspPose()
                pose.x = aff[0]
                pose.y = aff[1]
                pose.width = aff[2]
                pose.height = aff[3]
                pose.theta = aff[4]
                pose.confidence = score
                det.grasp_poses.append(pose)
        
        # Touching points
        if 'touching_points' in obj:
            for tp_pair in obj['touching_points']:
                tp = TouchingPoint()
                tp.point1 = Point(x=tp_pair[0][0], y=tp_pair[0][1], z=0)
                tp.point2 = Point(x=tp_pair[1][0], y=tp_pair[1][1], z=0)
                det.touching_points.append(tp)
        
        return det
    
    def publish_vis_image(self, image, header):
        """Publish visualization image in both raw and compressed formats"""
        try:
            # Publish raw image for RViz
            raw_msg = Image()
            raw_msg.header = header
            raw_msg.height = image.shape[0]
            raw_msg.width = image.shape[1]
            raw_msg.encoding = "bgr8"
            raw_msg.is_bigendian = 0
            raw_msg.step = raw_msg.width * 3
            raw_msg.data = image.tobytes()
            self.vis_pub_raw.publish(raw_msg)
            
            # Also publish compressed for bandwidth efficiency
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, compressed = cv2.imencode('.jpg', image, encode_param)
            
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed.tobytes()
            self.vis_pub_compressed.publish(compressed_msg)
            
        except Exception as e:
            rospy.logerr(f"Vis publish error: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        if self.process_thread.is_alive():
            self.process_thread.join(timeout=2.0)


def main():
    try:
        node = HandGraspDetectorNode()
        rospy.spin()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()