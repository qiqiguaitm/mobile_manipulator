#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
修复joint_states时间戳警告
检测并解决多个发布者的问题
"""

import rospy
import subprocess
import sys
import time

def get_joint_state_publishers():
    """获取所有发布joint_states的节点"""
    try:
        output = subprocess.check_output(['rostopic', 'info', '/joint_states'], 
                                       stderr=subprocess.STDOUT)
        lines = output.decode().split('\n')
        
        publishers = []
        in_publishers = False
        for line in lines:
            if 'Publishers:' in line:
                in_publishers = True
                continue
            if in_publishers and line.strip().startswith('*'):
                # 提取节点名
                node_info = line.strip().split()
                if len(node_info) >= 2:
                    node_name = node_info[1].split()[0]
                    publishers.append(node_name)
            elif in_publishers and not line.strip():
                break
                
        return publishers
    except:
        return []

def check_node_type(node_name):
    """检查节点类型"""
    try:
        output = subprocess.check_output(['rosnode', 'info', node_name], 
                                       stderr=subprocess.STDOUT)
        output_str = output.decode()
        
        if 'joint_state_publisher' in output_str:
            return 'joint_state_publisher'
        elif 'piper_ctrl_single_node' in output_str:
            return 'piper_ctrl_single_node'
        else:
            return 'unknown'
    except:
        return 'unknown'

def fix_joint_state_conflicts():
    """修复joint_states冲突"""
    rospy.init_node('joint_state_fixer', anonymous=True)
    
    rospy.loginfo("="*60)
    rospy.loginfo("Joint States 冲突检测与修复")
    rospy.loginfo("="*60)
    
    # 获取所有发布者
    publishers = get_joint_state_publishers()
    
    if not publishers:
        rospy.logwarn("未找到joint_states发布者")
        return
    
    rospy.loginfo(f"\n找到 {len(publishers)} 个发布者:")
    
    # 分析每个发布者
    piper_nodes = []
    jsp_nodes = []
    other_nodes = []
    
    for pub in publishers:
        node_type = check_node_type(pub)
        rospy.loginfo(f"  - {pub} ({node_type})")
        
        if node_type == 'piper_ctrl_single_node':
            piper_nodes.append(pub)
        elif node_type == 'joint_state_publisher':
            jsp_nodes.append(pub)
        else:
            other_nodes.append(pub)
    
    # 诊断问题
    rospy.loginfo("\n诊断结果:")
    
    if len(publishers) == 1:
        rospy.loginfo("✅ 只有一个发布者，配置正确")
        return
    
    # 多个发布者的情况
    rospy.logwarn(f"⚠️  检测到 {len(publishers)} 个发布者同时发布!")
    
    # 检查是否有多个相同类型的节点
    if len(piper_nodes) > 1:
        rospy.logerr(f"❌ 错误: 有 {len(piper_nodes)} 个piper_ctrl_single_node在运行!")
        rospy.loginfo("   可能原因: 多次启动了机械臂控制器")
        
    if len(jsp_nodes) > 1:
        rospy.logerr(f"❌ 错误: 有 {len(jsp_nodes)} 个joint_state_publisher在运行!")
        
    # 检查模式冲突
    if piper_nodes and jsp_nodes:
        rospy.logerr("❌ 错误: 同时运行了实机控制器和仿真发布器!")
        rospy.loginfo("   这通常是因为launch文件配置错误")
    
    # 提供修复建议
    rospy.loginfo("\n修复建议:")
    
    if piper_nodes and jsp_nodes:
        # 实机模式下应该停止joint_state_publisher
        rospy.loginfo("1. 在实机模式下，停止joint_state_publisher:")
        for jsp in jsp_nodes:
            rospy.loginfo(f"   rosnode kill {jsp}")
            
    if len(piper_nodes) > 1:
        # 保留第一个，停止其他的
        rospy.loginfo("2. 停止多余的piper控制器:")
        for i, node in enumerate(piper_nodes[1:], 1):
            rospy.loginfo(f"   rosnode kill {node}")
    
    # 询问是否自动修复
    if len(publishers) > 1:
        rospy.loginfo("\n是否自动修复? (需要手动确认)")
        rospy.loginfo("自动修复将:")
        if piper_nodes and jsp_nodes:
            rospy.loginfo("  - 停止所有joint_state_publisher节点")
        if len(piper_nodes) > 1:
            rospy.loginfo("  - 只保留一个piper_ctrl_single_node")
        
        # 等待3秒让用户看到信息
        time.sleep(3)
        
        # 执行修复
        if piper_nodes and jsp_nodes:
            rospy.loginfo("\n正在停止joint_state_publisher...")
            for jsp in jsp_nodes:
                try:
                    subprocess.call(['rosnode', 'kill', jsp])
                    rospy.loginfo(f"  ✓ 已停止 {jsp}")
                except:
                    rospy.logerr(f"  ✗ 无法停止 {jsp}")
        
        if len(piper_nodes) > 1:
            rospy.loginfo("\n正在停止多余的piper控制器...")
            for node in piper_nodes[1:]:
                try:
                    subprocess.call(['rosnode', 'kill', node])
                    rospy.loginfo(f"  ✓ 已停止 {node}")
                except:
                    rospy.logerr(f"  ✗ 无法停止 {node}")
        
        rospy.loginfo("\n修复完成! 请检查是否还有警告信息。")
    
    rospy.loginfo("="*60)

if __name__ == '__main__':
    try:
        fix_joint_state_conflicts()
    except rospy.ROSInterruptException:
        pass