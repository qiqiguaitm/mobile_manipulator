#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import serial
import struct
import time

# Constants
GRA_ACC = 9.8
DEG_TO_RAD = 0.01745329
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

class ImuDriverNode:
    def __init__(self):
        rospy.init_node('imu_driver_node')
        
        # Get parameters
        self.port = rospy.get_param('~port', DEFAULT_PORT)
        self.baud = rospy.get_param('~baud', DEFAULT_BAUD)
        
        # Publisher for IMU data
        self.imu_pub = rospy.Publisher('/sensors/imu/data', Imu, queue_size=10)
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            rospy.loginfo("IMU serial connection opened on %s at %d baud", self.port, self.baud)
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port %s: %s", self.port, str(e))
            raise
        
        rospy.loginfo("IMU driver node initialized")
    
    def read_imu_data(self):
        """Read and parse IMU data from serial port"""
        # This is a simplified implementation based on the C++ code
        # In a real implementation, you would need to properly parse the IMU protocol
        try:
            # Read data from serial port
            data = self.ser.read(1024)
            if len(data) > 0:
                # Process the data (this is a placeholder - real implementation would parse the protocol)
                # For now, we'll just return None to indicate no valid data
                return None
        except serial.SerialException as e:
            rospy.logerr("Serial read error: %s", str(e))
            return None
        
        return None
    
    def create_imu_message(self, imu_data):
        """Create IMU message from raw data"""
        # This is a placeholder - in a real implementation, you would extract data from the parsed packet
        # For now, we'll return None to indicate no valid data
        return None
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(100)  # 100 Hz
        
        while not rospy.is_shutdown():
            try:
                # Read IMU data
                imu_data = self.read_imu_data()
                
                if imu_data is not None:
                    # Create and publish IMU message
                    imu_msg = self.create_imu_message(imu_data)
                    if imu_msg is not None:
                        imu_msg.header.stamp = rospy.Time.now()
                        imu_msg.header.frame_id = "imu_link"
                        self.imu_pub.publish(imu_msg)
                
            except Exception as e:
                rospy.logerr("Error in IMU data processing: %s", str(e))
            
            rate.sleep()
        
        # Close serial connection
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("IMU serial connection closed")

if __name__ == '__main__':
    try:
        node = ImuDriverNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("IMU driver node error: %s", str(e))