#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import os

def configure_rviz_for_collision():
    """生成RViz配置指令"""
    
    print("\n" + "="*60)
    print("RViz碰撞显示配置指南")
    print("="*60)
    
    print("\n【方法1】在RViz的Motion Planning插件中手动配置：")
    print("\n1. Scene Robot设置:")
    print("   ☑ Show Robot Collision (显示碰撞网格)")
    print("   ☐ Show Robot Visual (隐藏视觉网格)")
    print("   Robot Alpha = 0.5 (半透明)")
    
    print("\n2. 在Scene Robot → Links中设置颜色:")
    print("   lifting_Link → 右键 → Set Color → 红色(255,0,0)")
    print("   lidar_Link → 右键 → Set Color → 红色(255,0,0)")
    print("   box_Link → 右键 → Set Color → 深红(200,0,0)")
    print("   base_link → 右键 → Set Color → 暗红(150,0,0)")
    
    print("\n3. Planning Request设置:")
    print("   Colliding Link Color = 红色(255,0,0)")
    
    print("\n【方法2】添加碰撞可视化标记：")
    print("\n1. 点击RViz左下角'Add'按钮")
    print("2. 选择'MarkerArray'")
    print("3. 设置Topic = /collision_visualization")
    
    print("\n【当前运行的碰撞可视化进程】")
    os.system("ps aux | grep collision | grep -v grep | grep -v rviz_collision_config")
    
    print("\n【可用的碰撞可视化话题】")
    os.system("rostopic list | grep collision")
    
    print("\n【测试碰撞检测】")
    print("1. 在Motion Planning中拖动橙色球体（交互标记）")
    print("2. 将机械臂移向红色区域")
    print("3. 观察是否自动避开或显示碰撞警告")
    
    print("\n" + "="*60)
    print("如果碰撞物体仍未显示为红色，请检查：")
    print("1. MoveIt是否正确加载了URDF/SRDF")
    print("2. TF树是否完整（使用rqt_tf_tree查看）")
    print("3. 碰撞网格是否正确加载")
    print("="*60 + "\n")

if __name__ == '__main__':
    configure_rviz_for_collision()