#!/usr/bin/python3
"""
3D Grasp Processor V2 - with relaxed synchronization
Uses latest messages instead of strict time sync
"""


import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from perception.msg import (GraspDetection, GraspDetectionArray, 
                           GraspPose3D, GraspDetection3D, GraspDetectionArray3D,
                           TouchingPoint3D)
from geometry_msgs.msg import Point, Pose, Quaternion
from cv_bridge import CvBridge
import tf.transformations as tf_trans
import threading


class DepthGraspProcessorV2:
    def __init__(self):
        rospy.init_node('depth_grasp_processor_v2')
        
        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.depth_scale = 0.001  # mm to meters
        
        self.bridge = CvBridge()
        
        # Latest messages (no strict sync)
        self.latest_depth = None
        self.latest_grasps = None
        self.depth_lock = threading.Lock()
        self.grasp_lock = threading.Lock()
        
        # Publishers
        self.grasp3d_pub = rospy.Publisher(
            '/perception/hand/grasps_3d', 
            GraspDetectionArray3D,
            queue_size=1
        )
        
        # Subscribers
        self.camera_info_sub = rospy.Subscriber(
            '/camera/hand/depth/camera_info',
            CameraInfo,
            self.camera_info_callback
        )
        
        self.depth_sub = rospy.Subscriber(
            '/camera/hand/aligned_depth_to_color/image_raw',
            Image,
            self.depth_callback
        )
        
        self.grasp_sub = rospy.Subscriber(
            '/perception/hand/grasps',
            GraspDetectionArray,
            self.grasp_callback
        )
        
        rospy.loginfo("3D Grasp processor V2 initialized (relaxed sync)")
    
    def camera_info_callback(self, msg):
        """Extract camera intrinsics"""
        if self.fx is None:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            rospy.loginfo(f"Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        with self.depth_lock:
            self.latest_depth = msg
            rospy.logdebug("Updated depth image")
    
    def grasp_callback(self, msg):
        """Process grasps when received"""
        with self.grasp_lock:
            self.latest_grasps = msg
        
        # Try to process with latest depth
        self.process_if_ready()
    
    def process_if_ready(self):
        """Process if we have all required data"""
        if self.fx is None:
            rospy.logwarn_throttle(5, "Waiting for camera intrinsics...")
            return
        
        with self.depth_lock:
            depth_msg = self.latest_depth
        
        if depth_msg is None:
            rospy.logwarn_throttle(5, "Waiting for depth image...")
            return
        
        with self.grasp_lock:
            grasp_msg = self.latest_grasps
        
        if grasp_msg is None:
            return
        
        # Just use latest depth image, don't check time
        # time_diff = abs((grasp_msg.header.stamp - depth_msg.header.stamp).to_sec())
        # rospy.logdebug(f"Time diff: {time_diff:.3f}s")
        
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            depth_meters = depth_image.astype(np.float32) * self.depth_scale
            
            # Create 3D grasp message
            grasp3d_msg = GraspDetectionArray3D()
            grasp3d_msg.header = grasp_msg.header
            
            # Process each detection
            for detection in grasp_msg.detections:
                grasp3d = self.convert_to_3d(detection, depth_meters)
                if grasp3d is not None:
                    grasp3d_msg.detections.append(grasp3d)
            
            # Publish
            if grasp3d_msg.detections:
                self.grasp3d_pub.publish(grasp3d_msg)
                rospy.loginfo(f"Published {len(grasp3d_msg.detections)} 3D grasps (time_diff={time_diff:.3f}s)")
            
        except Exception as e:
            rospy.logerr(f"Processing error: {e}")
    
    def convert_to_3d(self, detection, depth_image):
        """Convert 2D detection to 3D"""
        grasp3d = GraspDetection3D()
        
        # Copy 2D info
        grasp3d.bbox_2d = detection.bbox
        grasp3d.score = detection.score
        grasp3d.rle = detection.rle
        grasp3d.mask = detection.mask
        grasp3d.reid_feature = detection.reid_feature
        
        # Get object center from bbox
        x1, y1, x2, y2 = detection.bbox
        cx_2d = int((x1 + x2) / 2)
        cy_2d = int((y1 + y2) / 2)
        
        # Bounds check
        if cx_2d < 0 or cx_2d >= depth_image.shape[1] or cy_2d < 0 or cy_2d >= depth_image.shape[0]:
            return None
        
        # Sample depth in region (handle noise)
        roi_size = 5
        y_min = max(0, cy_2d - roi_size)
        y_max = min(depth_image.shape[0], cy_2d + roi_size)
        x_min = max(0, cx_2d - roi_size)
        x_max = min(depth_image.shape[1], cx_2d + roi_size)
        
        depth_roi = depth_image[y_min:y_max, x_min:x_max]
        valid_depths = depth_roi[depth_roi > 0.1]  # Filter invalid depths
        
        if len(valid_depths) == 0:
            rospy.logwarn(f"No valid depth for object at ({cx_2d}, {cy_2d})")
            return None
        
        # Use median for robustness
        depth = np.median(valid_depths)
        
        # Project to 3D
        x_3d = (cx_2d - self.cx) * depth / self.fx
        y_3d = (cy_2d - self.cy) * depth / self.fy
        z_3d = depth
        
        grasp3d.center_3d = Point(x=x_3d, y=y_3d, z=z_3d)
        
        # Calculate 3D bbox (simple approach)
        width_pixels = x2 - x1
        height_pixels = y2 - y1
        
        width_3d = width_pixels * depth / self.fx
        height_3d = height_pixels * depth / self.fy
        depth_3d = width_3d * 0.3  # Heuristic: depth proportional to width
        
        grasp3d.dimensions = Point(x=width_3d, y=height_3d, z=depth_3d)
        
        # Convert 2D grasp poses to 3D
        for grasp_2d in detection.grasp_poses:
            grasp_3d = self.grasp_2d_to_3d(grasp_2d, depth_image)
            if grasp_3d is not None:
                grasp3d.grasp_poses_3d.append(grasp_3d)
        
        # Convert touching points to 3D
        for tp in detection.touching_points:
            tp1_3d = self.point_2d_to_3d(tp.point1.x, tp.point1.y, depth_image)
            tp2_3d = self.point_2d_to_3d(tp.point2.x, tp.point2.y, depth_image)
            
            if tp1_3d and tp2_3d:
                tp3d = TouchingPoint3D()
                tp3d.point1 = tp1_3d
                tp3d.point2 = tp2_3d
                grasp3d.touching_points_3d.append(tp3d)
        
        return grasp3d
    
    def grasp_2d_to_3d(self, grasp_2d, depth_image):
        """Convert 2D grasp pose to 3D pose"""
        grasp_3d = GraspPose3D()
        
        # Get depth at grasp center
        cx = int(grasp_2d.x)
        cy = int(grasp_2d.y)
        
        if cx < 0 or cx >= depth_image.shape[1] or cy < 0 or cy >= depth_image.shape[0]:
            return None
        
        depth = depth_image[cy, cx]
        if depth < 0.1:  # Invalid depth
            return None
        
        # 3D position
        x_3d = (cx - self.cx) * depth / self.fx
        y_3d = (cy - self.cy) * depth / self.fy
        z_3d = depth
        
        grasp_3d.position = Point(x=x_3d, y=y_3d, z=z_3d)
        
        # Gripper orientation from 2D angle
        theta = grasp_2d.theta
        q = tf_trans.quaternion_from_euler(0, np.pi/4, theta)  # 45-degree approach
        grasp_3d.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Gripper width in 3D
        grasp_3d.width = grasp_2d.width * depth / self.fx
        grasp_3d.confidence = grasp_2d.confidence
        
        return grasp_3d
    
    def point_2d_to_3d(self, x_2d, y_2d, depth_image):
        """Project 2D point to 3D"""
        x, y = int(x_2d), int(y_2d)
        
        if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
            return None
        
        depth = depth_image[y, x]
        if depth < 0.1:
            return None
        
        x_3d = (x - self.cx) * depth / self.fx
        y_3d = (y - self.cy) * depth / self.fy
        z_3d = depth
        
        return Point(x=x_3d, y=y_3d, z=z_3d)


def main():
    try:
        processor = DepthGraspProcessorV2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()