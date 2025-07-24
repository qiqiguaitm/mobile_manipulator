#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState, Image, PointCloud2, Imu
from task_mgr.msg import task_mgr
from arm_planner.msg import ArmTrajectory

def test_message_flow():
    """Test the message flow between modules"""
    rospy.init_node('message_flow_test')
    
    # Publishers to simulate sensor data
    top_camera_pub = rospy.Publisher('/camera/top/dual/color/image_raw', Image, queue_size=10)
    hand_camera_pub = rospy.Publisher('/camera/hand/dual/color/image_raw', Image, queue_size=10)
    chassis_camera_pub = rospy.Publisher('/camera/chassis/dual/color/image_raw', Image, queue_size=10)
    lidar_pub = rospy.Publisher('/lidar/chassis/point_cloud', PointCloud2, queue_size=10)
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    
    # Wait for the system to initialize
    time.sleep(2)
    
    # Publish some test data
    image_msg = Image()
    image_msg.header.stamp = rospy.Time.now()
    image_msg.header.frame_id = "camera_frame"
    image_msg.height = 480
    image_msg.width = 640
    image_msg.encoding = "rgb8"
    image_msg.is_bigendian = False
    image_msg.step = 640 * 3
    image_msg.data = bytearray([0] * (640 * 480 * 3))  # Black image
    
    pointcloud_msg = PointCloud2()
    pointcloud_msg.header.stamp = rospy.Time.now()
    pointcloud_msg.header.frame_id = "lidar_frame"
    
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_frame"
    
    # Publish sensor data
    top_camera_pub.publish(image_msg)
    hand_camera_pub.publish(image_msg)
    chassis_camera_pub.publish(image_msg)
    lidar_pub.publish(pointcloud_msg)
    imu_pub.publish(imu_msg)
    
    rospy.loginfo("Published test sensor data")
    
    # Wait to see if modules respond
    time.sleep(5)
    
    rospy.loginfo("Message flow test completed")

if __name__ == '__main__':
    try:
        test_message_flow()
    except rospy.ROSInterruptException:
        pass