#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_camera_frames():
    rospy.init_node('camera_tf_publisher')
    
    br = tf2_ros.StaticTransformBroadcaster()
    transforms = []
    
    # Base link to camera link
    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_link"
    t1.child_frame_id = "camera_link"
    t1.transform.translation.x = 0.3  # 30cm forward
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.5  # 50cm up
    t1.transform.rotation.x = 0.0
    t1.transform.rotation.y = 0.0
    t1.transform.rotation.z = 0.0
    t1.transform.rotation.w = 1.0
    transforms.append(t1)
    
    # Camera link to camera color frame
    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "camera_link"
    t2.child_frame_id = "camera_color_frame"
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0
    t2.transform.rotation.x = 0.0
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0
    transforms.append(t2)
    
    # Camera color frame to camera depth frame
    t3 = TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "camera_color_frame"
    t3.child_frame_id = "camera_depth_frame"
    t3.transform.translation.x = 0.0
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 0.0
    t3.transform.rotation.x = 0.0
    t3.transform.rotation.y = 0.0
    t3.transform.rotation.z = 0.0
    t3.transform.rotation.w = 1.0
    transforms.append(t3)
    
    # Camera depth frame to camera depth optical frame (rotate for ROS convention)
    t4 = TransformStamped()
    t4.header.stamp = rospy.Time.now()
    t4.header.frame_id = "camera_depth_frame"
    t4.child_frame_id = "camera_depth_optical_frame"
    # Rotation from camera frame to optical frame (z forward, x right, y down)
    t4.transform.translation.x = 0.0
    t4.transform.translation.y = 0.0
    t4.transform.translation.z = 0.0
    # Rotate -90 degrees around x, then -90 degrees around z
    t4.transform.rotation.x = -0.5
    t4.transform.rotation.y = 0.5
    t4.transform.rotation.z = -0.5
    t4.transform.rotation.w = 0.5
    transforms.append(t4)
    
    # Camera color optical frame
    t5 = TransformStamped()
    t5.header.stamp = rospy.Time.now()
    t5.header.frame_id = "camera_color_frame"
    t5.child_frame_id = "camera_color_optical_frame"
    t5.transform.translation.x = 0.0
    t5.transform.translation.y = 0.0
    t5.transform.translation.z = 0.0
    t5.transform.rotation.x = -0.5
    t5.transform.rotation.y = 0.5
    t5.transform.rotation.z = -0.5
    t5.transform.rotation.w = 0.5
    transforms.append(t5)
    
    # World frame (optional, for global reference)
    t6 = TransformStamped()
    t6.header.stamp = rospy.Time.now()
    t6.header.frame_id = "world"
    t6.child_frame_id = "base_link"
    t6.transform.translation.x = 0.0
    t6.transform.translation.y = 0.0
    t6.transform.translation.z = 0.0
    t6.transform.rotation.x = 0.0
    t6.transform.rotation.y = 0.0
    t6.transform.rotation.z = 0.0
    t6.transform.rotation.w = 1.0
    transforms.append(t6)
    
    br.sendTransform(transforms)
    
    rospy.loginfo("Camera TF frames published")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_camera_frames()
    except rospy.ROSInterruptException:
        pass