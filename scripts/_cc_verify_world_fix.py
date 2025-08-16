#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import time

def verify_world_fix():
    """验证world统一修改是否成功"""
    print("\n" + "="*60)
    print("验证World统一修改")
    print("="*60)
    
    # 1. 检查URDF文件
    print("\n1. 检查URDF文件...")
    urdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/urdf/mobile_manipulator2_description.urdf"
    
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    if 'link name="world"' in urdf_content:
        print("  ✓ URDF正确使用world作为根链接")
    else:
        print("  ✗ URDF未找到world链接")
        
    if 'world_link' in urdf_content:
        print("  ✗ URDF仍包含world_link引用")
    else:
        print("  ✓ URDF已移除所有world_link引用")
    
    # 2. 检查SRDF文件
    print("\n2. 检查SRDF文件...")
    srdf_path = "/home/agilex/MobileManipulator/src/robot_description/mobile_manipulator2_description/config/mobile_manipulator2_description.srdf"
    
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    
    if 'parent_frame="world" child_link="base_link"' in srdf_content:
        print("  ✓ SRDF正确定义virtual_joint从world到base_link")
    else:
        print("  ✗ SRDF的virtual_joint定义不正确")
        
    if 'world_link' in srdf_content and 'world_link已删除' not in srdf_content:
        print("  ✗ SRDF仍包含world_link引用")
    else:
        print("  ✓ SRDF已移除所有world_link引用")
    
    # 3. 检查launch文件
    print("\n3. 检查launch文件...")
    launch_path = "/home/agilex/MobileManipulator/src/arm_planner/launch/arm_planner_demo.launch"
    
    with open(launch_path, 'r') as f:
        launch_content = f.read()
    
    if 'world_tf_bridge.launch' not in launch_content:
        print("  ✓ launch文件已移除world_tf_bridge引用")
    else:
        print("  ✗ launch文件仍包含world_tf_bridge引用")
    
    # 4. 总结
    print("\n" + "="*60)
    print("验证总结")
    print("="*60)
    print("\n所有静态文件检查完成。")
    print("\n建议的后续步骤：")
    print("1. 清理工作空间: catkin clean")
    print("2. 重新构建: catkin build")
    print("3. 重新source: source devel/setup.bash")
    print("4. 重新测试: roslaunch arm_planner arm_planner_demo.launch")

if __name__ == '__main__':
    verify_world_fix()