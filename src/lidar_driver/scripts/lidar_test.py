#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry

def pointcloud_callback(msg):
    rospy.loginfo("Received point cloud with %d points", msg.width * msg.height)

def laser_callback(msg):
    valid_ranges = [r for r in msg.ranges if not (r == float('inf') or r == float('nan'))]
    if valid_ranges:
        min_range = min(valid_ranges)
        max_range = max(valid_ranges)
        rospy.loginfo("Received laser scan with %d points, range: %.2f - %.2f meters", 
                      len(msg.ranges), min_range, max_range)
    else:
        rospy.loginfo("Received laser scan with %d points, no valid ranges", len(msg.ranges))

def processed_laser_callback(msg):
    valid_ranges = [r for r in msg.ranges if not (r == float('inf') or r == float('nan'))]
    if valid_ranges:
        min_range = min(valid_ranges)
        max_range = max(valid_ranges)
        rospy.loginfo("Received processed laser scan with %d points, range: %.2f - %.2f meters", 
                      len(msg.ranges), min_range, max_range)
    else:
        rospy.loginfo("Received processed laser scan with %d points, no valid ranges", len(msg.ranges))

def odom_callback(msg):
    rospy.loginfo("Received odometry: position (%.2f, %.2f, %.2f)", 
                  msg.pose.pose.position.x,
                  msg.pose.pose.position.y,
                  msg.pose.pose.position.z)

def main():
    rospy.init_node('lidar_test_listener', anonymous=True)
    
    # Subscribe to point cloud topic
    rospy.Subscriber('/lidar/chassis/point_cloud', PointCloud2, pointcloud_callback)
    
    # Subscribe to laser scan topic from pointcloud_to_laserscan
    rospy.Subscriber('/sensors/lidar/scan', LaserScan, laser_callback)
    
    # Subscribe to processed laser scan topic from lidar_driver_node
    rospy.Subscriber('/sensors/lidar/processed_scan', LaserScan, processed_laser_callback)
    
    # Subscribe to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rospy.loginfo("LiDAR test listener started. Waiting for data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass