#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
import tf
import math

class ChassisControllerNode:
    def __init__(self):
        rospy.init_node('chassis_controller_node')
        
        # Load topic configurations
        topics = rospy.get_param('topics')
        
        # Publishers for controller outputs
        self.state_pub = rospy.Publisher(topics['outputs']['state']['name'], PoseStamped, queue_size=10)
        
        # Subscribers for control inputs
        self.cmd_vel_sub = rospy.Subscriber(topics['inputs']['cmd_vel']['name'], Twist, self.cmd_vel_callback)
        
        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocity commands
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("ChassisController node initialized")
    
    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        rospy.logdebug("Received velocity command: linear_x=%f, angular_z=%f", self.linear_x, self.angular_z)
    
    def update_robot_state(self):
        """Update robot state based on velocity commands"""
        # Simple differential drive model
        dt = 0.01  # 100 Hz update rate
        
        # Update position
        self.x += self.linear_x * math.cos(self.theta) * dt
        self.y += self.linear_x * math.sin(self.theta) * dt
        self.theta += self.angular_z * dt
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Broadcast TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )
        
        # Publish state
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.state_pub.publish(pose_msg)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            self.update_robot_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ChassisControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass