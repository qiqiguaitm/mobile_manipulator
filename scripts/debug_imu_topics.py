#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    rospy.loginfo("Received IMU data on topic: %s", rospy.get_name())

def main():
    rospy.init_node('debug_imu_topics', anonymous=True)
    
    # Subscribe to both possible topics to see which one has data
    rospy.Subscriber('/imu/data', Imu, imu_callback, callback_args='/imu/data')
    rospy.Subscriber('/sensors/imu/data', Imu, imu_callback, callback_args='/sensors/imu/data')
    
    rospy.loginfo("Debug node started. Listening for IMU data on both topics...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass