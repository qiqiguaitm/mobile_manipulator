#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion
import math

def check_tf_tree():
    """检查TF树的详细信息"""
    rospy.init_node('check_tf_tree', anonymous=True)
    
    # 创建TF监听器
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tf_listener = tf.TransformListener()
    
    # 等待TF树建立
    rospy.sleep(2.0)
    
    print("\n=== TF树检查 ===")
    
    # 关键frame列表
    frames_to_check = [
        "world",
        "world_link",
        "base_link",
        "box_Link",
        "lifting_Link",
        "lidar_Link",
        "arm_base",
        "link1", "link2", "link3", "link4", "link5", "link6",
        "gripper_base",
        "link7", "link8"
    ]
    
    # 检查每个frame是否存在
    print("\n检查frame存在性：")
    existing_frames = []
    for frame in frames_to_check:
        try:
            # 尝试获取到base_link的变换
            trans = tfBuffer.lookup_transform("base_link", frame, rospy.Time(), rospy.Duration(0.1))
            existing_frames.append(frame)
            print(f"  ✅ {frame}")
        except:
            try:
                # 反向尝试
                trans = tfBuffer.lookup_transform(frame, "base_link", rospy.Time(), rospy.Duration(0.1))
                existing_frames.append(frame)
                print(f"  ✅ {frame} (反向)")
            except:
                print(f"  ❌ {frame}")
    
    # 获取所有可用的frame
    print("\n所有可用的TF frames:")
    try:
        all_frames = tf_listener.getFrameStrings()
        for frame in sorted(all_frames):
            if frame not in frames_to_check:
                print(f"  - {frame}")
    except:
        print("  无法获取frame列表")
    
    # 检查关键变换关系
    print("\n=== 关键变换关系 ===")
    
    key_transforms = [
        ("world_link", "base_link"),
        ("base_link", "box_Link"),
        ("box_Link", "lifting_Link"),
        ("box_Link", "arm_base"),
        ("arm_base", "link1"),
        ("link1", "link2"),
        ("link2", "link3"),
        ("link6", "gripper_base"),
        ("gripper_base", "link7"),
        ("gripper_base", "link8"),
        ("base_link", "lidar_Link"),
        ("lifting_Link", "lidar_Link"),  # 这个可能有问题
        ("lifting_Link", "link1"),       # 检查是否有意外的关系
        ("lifting_Link", "link2"),
        ("lifting_Link", "link3")
    ]
    
    for parent, child in key_transforms:
        try:
            trans = tfBuffer.lookup_transform(parent, child, rospy.Time(), rospy.Duration(0.5))
            
            # 提取位置和旋转
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            quat = [trans.transform.rotation.x, 
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w]
            
            # 转换为欧拉角
            roll, pitch, yaw = euler_from_quaternion(quat)
            
            print(f"\n{parent} → {child}:")
            print(f"  位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            print(f"  旋转: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
            
            # 检查异常值
            if abs(x) > 10 or abs(y) > 10 or abs(z) > 10:
                print("  ⚠️  警告：位置值异常大！")
            
        except Exception as e:
            print(f"\n{parent} → {child}: ❌ 无法获取变换")
            # print(f"  错误: {e}")
    
    # 特别检查：碰撞相关的frame关系
    print("\n=== 碰撞检测相关的空间关系 ===")
    
    collision_checks = [
        ("lifting_Link", "link1", "检查link1与lifting的相对位置"),
        ("lifting_Link", "link2", "检查link2与lifting的相对位置"),
        ("lifting_Link", "link3", "检查link3与lifting的相对位置"),
        ("lidar_Link", "link1", "检查link1与lidar的相对位置"),
        ("lidar_Link", "link2", "检查link2与lidar的相对位置"),
        ("link7", "link8", "检查夹爪之间的相对位置")
    ]
    
    for frame1, frame2, desc in collision_checks:
        try:
            trans = tfBuffer.lookup_transform(frame1, frame2, rospy.Time(), rospy.Duration(0.5))
            
            # 计算距离
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            distance = math.sqrt(x*x + y*y + z*z)
            
            print(f"\n{desc}:")
            print(f"  {frame1} → {frame2}")
            print(f"  相对位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            print(f"  距离: {distance:.3f}m")
            
            if distance < 0.05:
                print("  ⚠️  警告：距离过近，可能存在碰撞风险！")
            
        except:
            print(f"\n{desc}: ❌ 无法计算")
    
    # 检查gripper_base的问题
    print("\n=== Gripper Base 特殊检查 ===")
    try:
        # 检查gripper_base是否正确连接到link6
        trans = tfBuffer.lookup_transform("link6", "gripper_base", rospy.Time(), rospy.Duration(0.5))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        print(f"link6 → gripper_base 变换:")
        print(f"  位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        if abs(x) < 0.001 and abs(y) < 0.001 and abs(z) < 0.001:
            print("  ⚠️  警告：gripper_base与link6位置完全相同，可能缺少偏移！")
            
    except Exception as e:
        print(f"无法获取gripper_base变换: {e}")

if __name__ == '__main__':
    try:
        check_tf_tree()
    except rospy.ROSInterruptException:
        pass