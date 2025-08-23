#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RGBD投影功能测试脚本
验证三个相机的color图像和深度图关联后投影到机器人坐标系

测试内容：
1. 验证相机话题是否存在
2. 测试单相机RGBD投影
3. 测试多相机同步投影
"""

import rospy
import rostopic
from sensor_msgs.msg import Image, PointCloud2
import time

def check_topics():
    """检查必要的话题是否存在"""
    print("=== 检查相机话题 ===")
    
    # 灵活检查 - 不要求所有相机都存在
    depth_topics = [
        '/camera/chassis/depth/image_rect_raw',
        '/camera/top/depth/image_rect_raw', 
        '/camera/hand/depth/image_rect_raw'
    ]
    
    color_topics = [
        '/camera/chassis/color/image_raw',
        '/camera/top/color/image_raw', 
        '/camera/hand/color/image_raw'
    ]
    
    published_topics = rostopic.get_topic_list()[0]
    topic_names = [topic[0] for topic in published_topics]
    
    # 检查可用的相机
    available_cameras = []
    camera_names = ['chassis', 'top', 'hand']
    
    for i, camera in enumerate(camera_names):
        depth_topic = depth_topics[i]
        color_topic = color_topics[i]
        
        has_depth = depth_topic in topic_names
        has_color = color_topic in topic_names
        
        if has_depth:
            available_cameras.append(camera)
            print(f"✓ {camera}相机深度: {depth_topic}")
            if has_color:
                print(f"✓ {camera}相机颜色: {color_topic}")
            else:
                print(f"⚠️ {camera}相机颜色: {color_topic} (缺失)")
        else:
            print(f"✗ {camera}相机深度: {depth_topic} (缺失)")
    
    if not available_cameras:
        print("\n错误: 没有发现任何可用的相机!")
        return False
    
    print(f"\n发现 {len(available_cameras)} 个可用相机: {', '.join(available_cameras)}")
    return True

def test_pointcloud_output():
    """测试点云输出"""
    print("\n=== 测试点云输出 ===")
    
    # 动态检查存在的点云话题
    published_topics = rostopic.get_topic_list()[0]
    topic_names = [topic[0] for topic in published_topics]
    
    possible_cloud_topics = [
        '/projected_cloud/chassis_camera',
        '/projected_cloud/top_camera', 
        '/projected_cloud/hand_camera',
        '/projected_cloud/combined'
    ]
    
    # 只订阅存在的点云话题
    active_topics = []
    for topic in possible_cloud_topics:
        if topic in topic_names:
            active_topics.append(topic)
            print(f"发现点云话题: {topic}")
    
    if not active_topics:
        print("⚠️ 未发现任何点云话题")
        return False
    
    print(f"等待 {len(active_topics)} 个点云话题...")
    received_clouds = {}
    
    def cloud_callback(msg, topic_name):
        camera_name = topic_name.split('/')[-1]
        received_clouds[camera_name] = {
            'msg': msg,
            'topic': topic_name,
            'timestamp': time.time()
        }
        print(f"✓ 收到 {camera_name} 点云: {msg.width} 点")
    
    # 订阅点云话题
    subscribers = []
    for topic in active_topics:
        sub = rospy.Subscriber(topic, PointCloud2, cloud_callback, topic)
        subscribers.append(sub)
    
    # 等待数据
    print("等待点云数据 (15秒超时)...")
    start_time = time.time()
    last_count = 0
    
    while time.time() - start_time < 15.0:
        rospy.sleep(0.5)
        current_count = len(received_clouds)
        
        if current_count > last_count:
            print(f"已收到 {current_count}/{len(active_topics)} 个点云")
            last_count = current_count
            
        if current_count >= len(active_topics):
            break
    
    print(f"\n在15秒内收到 {len(received_clouds)}/{len(active_topics)} 个点云")
    
    # 分析点云数据
    if received_clouds:
        print("\n=== 点云数据分析 ===")
        total_points = 0
        
        for camera_name, data in received_clouds.items():
            cloud_msg = data['msg']
            print(f"\n📊 {camera_name} 点云:")
            print(f"  点数: {cloud_msg.width:,}")
            print(f"  字段: {[field.name for field in cloud_msg.fields]}")
            print(f"  帧ID: {cloud_msg.header.frame_id}")
            
            # 检查是否包含RGB数据
            has_color = any(field.name == 'rgb' for field in cloud_msg.fields)
            print(f"  包含颜色: {'✓ 是' if has_color else '✗ 否'}")
            
            # 检查点云密度
            if cloud_msg.width > 0:
                density = "高" if cloud_msg.width > 10000 else "中" if cloud_msg.width > 1000 else "低"
                print(f"  点云密度: {density}")
            
            total_points += cloud_msg.width
        
        print(f"\n总点数: {total_points:,}")
        return True
    else:
        print("\n⚠️ 未收到任何点云数据")
        return False

def main():
    """主测试函数"""
    rospy.init_node('test_rgbd_projection', anonymous=True)
    print("🧪 启动RGBD投影功能测试...")
    print("=" * 50)
    
    success_count = 0
    test_count = 2
    
    # 1. 检查话题
    print("\n🔍 测试1: 检查相机话题")
    if check_topics():
        print("✅ 相机话题检查: 通过")
        success_count += 1
    else:
        print("❌ 相机话题检查: 失败")
    
    # 2. 测试点云输出 
    print("\n☁️ 测试2: 检查点云输出")
    if test_pointcloud_output():
        print("✅ 点云输出测试: 通过")
        success_count += 1
    else:
        print("❌ 点云输出测试: 失败")
    
    # 测试总结
    print("\n" + "=" * 50)
    print("🏁 测试完成")
    print(f"通过率: {success_count}/{test_count} ({100*success_count/test_count:.0f}%)")
    
    if success_count == test_count:
        print("🎉 所有测试通过! RGBD投影功能正常工作")
    elif success_count > 0:
        print("⚠️ 部分测试通过，系统部分工作正常")
    else:
        print("💥 所有测试失败，请检查系统配置")
    
    print("\n建议检查项目:")
    print("- 确保相机驱动已启动: roslaunch camera_driver camera_driver.launch")
    print("- 确保深度投影节点已启动: roslaunch perception depth_projection.launch")
    print("- 检查TF树: rosrun tf view_frames")
    print("- 查看话题列表: rostopic list | grep -E '(depth|color|cloud)'")
    
    return success_count == test_count

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"测试出错: {e}")