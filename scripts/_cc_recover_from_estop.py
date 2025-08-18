#!/usr/bin/env python3
"""
紧急停止后的恢复脚本
"""

import rospy
from std_srvs.srv import Empty
import time

def recover_from_emergency_stop():
    rospy.init_node('emergency_stop_recovery', anonymous=True)
    
    print("="*50)
    print("紧急停止恢复程序")
    print("="*50)
    
    # 等待服务可用
    services = [
        '/reset_emergency_stop',
        '/reset_safety_system',
        '/enable_arm'
    ]
    
    available_services = []
    for service in services:
        try:
            rospy.wait_for_service(service, timeout=2.0)
            available_services.append(service)
            print(f"✓ 找到服务: {service}")
        except:
            print(f"✗ 服务不可用: {service}")
    
    if not available_services:
        print("\n错误：没有找到任何恢复服务！")
        return False
    
    # 执行恢复
    print("\n开始恢复流程...")
    
    # 1. 重置紧急停止
    if '/reset_emergency_stop' in available_services:
        try:
            reset_estop = rospy.ServiceProxy('/reset_emergency_stop', Empty)
            reset_estop()
            print("✓ 已重置紧急停止")
            time.sleep(0.5)
        except Exception as e:
            print(f"✗ 重置紧急停止失败: {e}")
    
    # 2. 重置安全系统
    if '/reset_safety_system' in available_services:
        try:
            reset_safety = rospy.ServiceProxy('/reset_safety_system', Empty)
            reset_safety()
            print("✓ 已重置安全系统")
            time.sleep(0.5)
        except Exception as e:
            print(f"✗ 重置安全系统失败: {e}")
    
    # 3. 使能机械臂
    if '/enable_arm' in available_services:
        try:
            enable_arm = rospy.ServiceProxy('/enable_arm', Empty)
            enable_arm()
            print("✓ 已使能机械臂")
        except Exception as e:
            print(f"✗ 使能机械臂失败: {e}")
    
    print("\n恢复流程完成！")
    print("提示：")
    print("- 请检查机械臂周围是否安全")
    print("- 建议先低速测试运动")
    print("- 如果问题持续，请检查具体的错误信息")
    
    return True

if __name__ == '__main__':
    try:
        recover_from_emergency_stop()
    except rospy.ROSInterruptException:
        pass