#!/usr/bin/env python3
"""
启动延时脚本 - 用于等待相机图像流稳定
修复OGRE图像加载错误的辅助工具
"""

import rospy
import time

def main():
    rospy.init_node('startup_delay', anonymous=True)
    
    delay_seconds = rospy.get_param('~delay_seconds', 3.0)
    message = rospy.get_param('~message', 'Startup delay...')
    
    rospy.loginfo(f"{message}")
    time.sleep(delay_seconds)
    rospy.loginfo(f"延时完成，继续启动...")

if __name__ == '__main__':
    main()
