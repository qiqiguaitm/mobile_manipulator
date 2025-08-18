#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys

def main():
    rospy.init_node('launch_status_info')
    
    # 获取启动模式参数
    mode = sys.argv[1] if len(sys.argv) > 1 else "unknown"
    
    rospy.loginfo("="*50)
    rospy.loginfo("Piper机械臂系统启动成功!")
    rospy.loginfo(f"当前模式: {mode}")
    
    if mode == "fake":
        rospy.loginfo("仿真模式 - 不需要实际硬件")
    elif mode == "real":
        rospy.loginfo("实机模式 - 请确保CAN接口已连接")
    
    rospy.loginfo("提示：使用RViz进行可视化和运动规划")
    rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass