#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def fix_world_tf():
    """发布world到world_link的静态变换，解决TF树断开的问题"""
    rospy.init_node('world_tf_fix', anonymous=True)
    
    print("\n" + "="*60)
    print("修复World TF问题")
    print("="*60)
    
    # 创建静态变换发布器
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    # 创建world到world_link的恒等变换
    static_transform = TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "world"
    static_transform.child_frame_id = "world_link"
    
    # 恒等变换（位置和旋转都是0）
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0
    
    # 发布静态变换
    static_broadcaster.sendTransform(static_transform)
    
    print("\n✓ 已发布 world -> world_link 静态变换")
    print("  这将连接两个TF树，解决警告信息")
    
    # 也可以反向发布，以防有节点期望world_link作为父坐标系
    reverse_transform = TransformStamped()
    reverse_transform.header.stamp = rospy.Time.now()
    reverse_transform.header.frame_id = "world_link"
    reverse_transform.child_frame_id = "world"
    reverse_transform.transform = static_transform.transform
    
    # 注意：通常只需要一个方向的变换
    # static_broadcaster.sendTransform(reverse_transform)
    
    print("\n提示：")
    print("1. 这是一个临时解决方案")
    print("2. 最好找到使用'world'坐标系的节点并修改为'world_link'")
    print("3. 检查是否有外部包（如perception或slam）在使用'world'")
    
    print("\n节点保持运行中... (Ctrl+C退出)")
    
    # 定期检查并报告TF状态
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(0.1)  # 每10秒检查一次
    while not rospy.is_shutdown():
        try:
            # 测试变换是否工作
            trans = tf_buffer.lookup_transform("world", "base_link", rospy.Time(0))
            # 如果成功，不输出任何信息（避免刷屏）
        except Exception as e:
            print("\n⚠ TF查找失败: %s" % str(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        fix_world_tf()
    except rospy.ROSInterruptException:
        print("\n已停止World TF修复节点")