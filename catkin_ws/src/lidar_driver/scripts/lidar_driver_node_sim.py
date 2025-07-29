#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import math
import std_msgs.msg

class LidarDriverNode:
    def __init__(self):
        rospy.init_node('lidar_driver_node')
        
        # Publisher for processed lidar scan data
        self.lidar_pub = rospy.Publisher('/sensors/lidar/processed_scan', LaserScan, queue_size=10)
        
        # Publisher for simulated point cloud data (for testing)
        self.pointcloud_pub = rospy.Publisher('/lidar/chassis/point_cloud', PointCloud2, queue_size=10)
        
        # Publisher for simulated laser scan data (for testing)
        self.laser_pub = rospy.Publisher('/sensors/lidar/scan', LaserScan, queue_size=10)
        
        # Subscriber for laser scan data (if available)
        self.laser_sub = rospy.Subscriber('/sensors/lidar/scan', LaserScan, self.laser_callback)
        
        # Subscriber for point cloud data (if available)
        self.pointcloud_sub = rospy.Subscriber('/lidar/chassis/point_cloud', PointCloud2, self.pointcloud_callback)
        
        # Timer for publishing simulated data
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_simulated_data)
        
        rospy.loginfo("Lidar driver node initialized - forwarding and processing LiDAR data")
    
    def laser_callback(self, msg):
        """Callback function to process laser scan data"""
        # Forward the laser scan data as is
        self.lidar_pub.publish(msg)
    
    def pointcloud_callback(self, msg):
        """Callback function to process point cloud data"""
        # Log point cloud information
        point_count = msg.width * msg.height
        rospy.logdebug("Received point cloud with %d points", point_count)
    
    def publish_simulated_data(self, event):
        """Publish simulated laser scan and point cloud data for testing"""
        # Create a simulated laser scan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"
        scan.angle_min = -math.pi/2
        scan.angle_max = math.pi/2
        scan.angle_increment = math.pi/180  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Generate simulated ranges
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = []
        for i in range(num_readings):
            # Simulate some obstacles at different distances
            angle = scan.angle_min + i * scan.angle_increment
            # Simple simulation with some obstacles
            distance = 3.0 + 2.0 * math.sin(angle)  # Varying distance
            scan.ranges.append(min(distance, scan.range_max))
        
        # Publish the simulated laser scan
        self.laser_pub.publish(scan)
        
        # Create a simulated point cloud
        points = []
        for i in range(0, len(scan.ranges), 10):  # Sample every 10th point
            if not math.isnan(scan.ranges[i]) and scan.ranges[i] < scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                x = scan.ranges[i] * math.cos(angle)
                y = scan.ranges[i] * math.sin(angle)
                z = 0.0
                points.append([x, y, z])
        
        if points:
            point_cloud = point_cloud2.create_cloud_xyz32(std_msgs.msg.Header(), points)
            point_cloud.header.stamp = rospy.Time.now()
            point_cloud.header.frame_id = "laser_frame"
            self.pointcloud_pub.publish(point_cloud)
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LidarDriverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in lidar driver node: %s", str(e))