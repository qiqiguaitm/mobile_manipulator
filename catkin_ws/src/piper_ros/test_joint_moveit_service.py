#!/usr/bin/env python3

import rospy
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
import sys

def test_endpose_service():
    rospy.init_node('test_joint_moveit_service')
    
    # 等待服务可用
    service_name = '/joint_moveit_ctrl_endpose'
    print(f"等待服务 {service_name} 可用...")
    
    try:
        rospy.wait_for_service(service_name, timeout=10.0)
        print(f"服务 {service_name} 已可用")
    except rospy.ROSException:
        print(f"错误: 服务 {service_name} 在10秒内未变为可用")
        return False
    
    # 创建服务客户端
    try:
        client = rospy.ServiceProxy(service_name, JointMoveitCtrl)
        
        # 创建请求
        request = JointMoveitCtrlRequest()
        request.joint_endpose = [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
        request.max_velocity = 0.5
        request.max_acceleration = 0.5
        
        print("发送末端位姿控制请求...")
        response = client(request)
        
        if response.status:
            print("✅ 末端位姿控制成功")
            return True
        else:
            print(f"❌ 末端位姿控制失败，错误代码: {response.error_code}")
            return False
            
    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")
        return False

if __name__ == '__main__':
    try:
        success = test_endpose_service()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        print("测试被中断")
        sys.exit(1)