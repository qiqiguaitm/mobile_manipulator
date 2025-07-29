#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf

class OdomTestNode:
    def __init__(self):
        rospy.init_node('odom_test_node')
        
        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Subscriber for laser scan data
        self.scan_sub = rospy.Subscriber('/lidar/chassis/scan', LaserScan, self.scan_callback)
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Odometry test node initialized")
    
    def odom_callback(self, msg):
        """Callback function to process odometry data"""
        rospy.loginfo("Received odometry: position (%.3f, %.3f, %.3f), orientation (%.3f, %.3f, %.3f, %.3f)", 
                      msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z,
                      msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
    
    def scan_callback(self, msg):
        """Callback function to process laser scan data"""
        valid_ranges = [r for r in msg.ranges if not (r == float('inf') or r == float('nan'))]
        if valid_ranges:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            rospy.loginfo("Received laser scan with %d points, range: %.2f - %.2f meters", 
                          len(msg.ranges), min_range, max_range)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            try:
                # Check if TF transform is available
                (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                rospy.loginfo("TF transform available: translation (%.3f, %.3f, %.3f)", trans[0], trans[1], trans[2])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("TF transform not available yet")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = OdomTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in odom test node: %s", str(e))