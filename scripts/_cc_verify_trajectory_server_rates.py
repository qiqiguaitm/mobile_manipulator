#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
验证 FollowJointTrajectoryServer 的频率设置
"""

import rospy
import subprocess
import os

def check_trajectory_server_rates():
    """检查轨迹服务器的频率设置"""
    print("\n=== FollowJointTrajectoryServer 频率配置检查 ===\n")
    
    file_path = '/home/agilex/MobileManipulator/src/arm_planner/scripts/follow_joint_trajectory_server.py'
    
    # 读取文件并查找Rate设置
    rate_settings = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines, 1):
            if 'Rate(' in line and not line.strip().startswith('#'):
                rate_settings.append((i, line.strip()))
    
    # 显示结果
    print(f"文件: {file_path}")
    print(f"找到 {len(rate_settings)} 个频率设置:\n")
    
    for line_num, line in rate_settings:
        # 提取频率值
        try:
            rate_value = int(line.split('Rate(')[1].split(')')[0])
            status = "✓ 正确" if rate_value == 50 else "✗ 需要修改"
            recommendation = "" if rate_value == 50 else f" (建议改为 50Hz)"
            print(f"  第{line_num}行: {line} - {status}{recommendation}")
        except:
            print(f"  第{line_num}行: {line} - 无法解析")
    
    print("\n推荐设置:")
    print("  - 轨迹执行循环: 50Hz (轨迹点插值和执行)")
    print("  - 底层控制器: 200Hz (快速响应)")
    print("\n原因:")
    print("  1. 轨迹服务器主要处理轨迹点之间的插值")
    print("  2. MoveIt生成的轨迹点通常间隔20-100ms")
    print("  3. 50Hz足以平滑执行轨迹，避免过度CPU消耗")

if __name__ == '__main__':
    check_trajectory_server_rates()