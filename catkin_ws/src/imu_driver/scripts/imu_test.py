#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    """Callback function to process IMU data"""
    rospy.loginfo("Received IMU data:")
    rospy.loginfo("  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    rospy.loginfo("  Angular Velocity: x=%.3f, y=%.3f, z=%.3f", 
                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    rospy.loginfo("  Linear Acceleration: x=%.3f, y=%.3f, z=%.3f", 
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

def main():
    rospy.init_node('imu_test_listener', anonymous=True)
    
    # Subscribe to IMU topic
    rospy.Subscriber('/sensors/imu/data', Imu, imu_callback)
    
    rospy.loginfo("IMU test listener started. Waiting for data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass