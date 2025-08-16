#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import subprocess
import time
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
import tf2_ros
from std_msgs.msg import String

def test_world_unification():
    """测试world坐标系统一后的配置"""
    print("\n" + "="*60)
    print("测试World坐标系统一")
    print("="*60)
    
    # 初始化ROS节点
    rospy.init_node('test_world_unification', anonymous=True)
    
    # 1. 测试TF树
    print("\n1. 检查TF树...")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 等待TF树建立
    rospy.sleep(2.0)
    
    # 测试关键变换
    test_transforms = [
        ("world", "base_link"),
        ("world", "arm_base"),
        ("base_link", "link1"),
    ]
    
    tf_success = True
    for parent, child in test_transforms:
        try:
            trans = tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(1.0))
            print("  ✓ %s -> %s: 成功" % (parent, child))
        except Exception as e:
            print("  ✗ %s -> %s: 失败 - %s" % (parent, child, str(e)))
            tf_success = False
    
    # 2. 测试MoveIt配置
    print("\n2. 检查MoveIt配置...")
    try:
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        
        # 获取规划参考坐标系
        planning_frame = robot.get_planning_frame()
        print("  规划参考坐标系: %s" % planning_frame)
        
        # 检查是否为world
        if planning_frame == "world":
            print("  ✓ MoveIt正确使用world作为参考坐标系")
        else:
            print("  ✗ MoveIt使用了错误的参考坐标系: %s" % planning_frame)
        
        # 获取根链接
        root_link = robot.get_root_link()
        print("  根链接: %s" % root_link)
        
        # 列出所有链接
        all_links = robot.get_link_names()
        if "world_link" in all_links:
            print("  ⚠ 警告：world_link仍然存在于机器人模型中")
        else:
            print("  ✓ world_link已成功从机器人模型中移除")
            
    except Exception as e:
        print("  ✗ MoveIt初始化失败: %s" % str(e))
    
    # 3. 检查URDF模型
    print("\n3. 检查URDF模型...")
    try:
        # 从参数服务器获取robot_description
        robot_description = rospy.get_param("/robot_description", "")
        
        if "world_link" in robot_description:
            print("  ✗ URDF中仍包含world_link")
        else:
            print("  ✓ URDF中已不包含world_link")
            
        if '<link name="world"' in robot_description:
            print("  ✓ URDF正确使用world作为根链接")
        else:
            print("  ✗ URDF未找到world链接")
            
    except Exception as e:
        print("  ✗ 无法获取robot_description: %s" % str(e))
    
    # 4. 总结
    print("\n" + "="*60)
    print("测试总结")
    print("="*60)
    
    if tf_success and planning_frame == "world" and "world_link" not in all_links:
        print("✓ 所有测试通过！World坐标系已成功统一。")
        print("\n建议的后续步骤：")
        print("1. 运行完整的MoveIt演示确认功能正常")
        print("2. 测试机械臂运动规划")
        print("3. 检查其他依赖world_link的节点（如有）")
    else:
        print("✗ 存在问题需要解决")
        print("\n可能的问题：")
        print("1. 检查是否所有节点都已重新启动")
        print("2. 确认URDF和SRDF文件已正确更新")
        print("3. 清理并重新构建工作空间")

if __name__ == '__main__':
    try:
        test_world_unification()
    except rospy.ROSInterruptException:
        pass