#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    """Callback function to process IMU data"""
    rospy.loginfo("Received IMU data on /imu/data:")
    rospy.loginfo("  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    rospy.loginfo("  Angular Velocity: x=%.3f, y=%.3f, z=%.3f", 
                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    rospy.loginfo("  Linear Acceleration: x=%.3f, y=%.3f, z=%.3f", 
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    rospy.signal_shutdown("Received first IMU message")

def main():
    rospy.init_node('imu_topic_test', anonymous=True)
    
    # Subscribe to the new IMU topic
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    
    rospy.loginfo("IMU topic test started. Waiting for data on /imu/data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass