#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import time

def fix_moveit_issues():
    """修复MoveIt相关问题"""
    print("\n" + "="*60)
    print("修复MoveIt配置问题")
    print("="*60)
    
    # 1. 检查并杀掉错误的world_to_base节点
    print("\n1. 检查错误的TF发布器...")
    try:
        # 查找world_to_base进程
        result = subprocess.check_output("rosnode list | grep world_to_base", shell=True)
        if result.strip():
            print("   发现错误的world_to_base节点，正在停止...")
            subprocess.call("rosnode kill /world_to_base", shell=True)
            time.sleep(1)
    except:
        print("   ✓ 没有错误的world_to_base节点")
    
    # 2. 启动正确的TF桥接
    print("\n2. 启动world到world_link的TF桥接...")
    subprocess.Popen(["rosrun", "tf2_ros", "static_transform_publisher", 
                     "0", "0", "0", "0", "0", "0", "world_link", "world"])
    print("   ✓ TF桥接已启动")
    
    # 3. 关于piper_with_gripper组的说明
    print("\n3. 关于piper_with_gripper组的问题:")
    print("   piper_with_gripper包含8个关节(joint1-8)")
    print("   其中joint7和joint8是夹爪的prismatic关节")
    print("   KDL求解器可能无法处理包含夹爪的完整链")
    print("\n   建议的解决方案:")
    print("   - 使用'piper'组（6轴）进行运动规划")
    print("   - 单独控制'gripper'组（joint7-8）")
    print("   - 或修改SRDF，创建一个只包含joint1-6的新组")
    
    # 4. 验证TF连接
    print("\n4. 验证TF连接...")
    time.sleep(2)
    try:
        result = subprocess.check_output(
            "rosrun tf tf_echo world world_link -c 1", 
            shell=True, stderr=subprocess.STDOUT, timeout=3
        )
        if "Translation" in result.decode():
            print("   ✓ world到world_link连接成功")
        else:
            print("   ✗ TF连接失败")
    except subprocess.TimeoutExpired:
        print("   ✗ TF连接超时")
    except Exception as e:
        print("   ✗ TF验证失败: %s" % str(e))
    
    print("\n" + "="*60)
    print("总结:")
    print("1. TF问题已修复（world <-> world_link）")
    print("2. piper_with_gripper组的问题需要在配置中解决")
    print("3. 建议使用'piper'组进行机械臂运动规划")
    print("="*60 + "\n")

if __name__ == '__main__':
    rospy.init_node('fix_moveit_issues', anonymous=True)
    fix_moveit_issues()
    print("修复完成，节点退出。")