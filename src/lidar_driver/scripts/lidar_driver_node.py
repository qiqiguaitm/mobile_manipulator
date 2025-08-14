#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import math

class LidarDriverNode:
    def __init__(self):
        rospy.init_node('lidar_driver_node')
        
        # Publisher for processed lidar scan data
        self.lidar_pub = rospy.Publisher('/sensors/lidar/processed_scan', LaserScan, queue_size=10)
        
        # Subscriber for laser scan data from pointcloud_to_laserscan
        self.laser_sub = rospy.Subscriber('/sensors/lidar/scan', LaserScan, self.laser_callback)
        
        # Subscriber for point cloud data from rslidar_sdk
        self.pointcloud_sub = rospy.Subscriber('/lidar/chassis/point_cloud', PointCloud2, self.pointcloud_callback)
        
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