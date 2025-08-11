#!/usr/bin/env python3

import rospy
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
from sensor_msgs.msg import JointState
import time

def test_system_status():
    """测试系统状态和修复效果"""
    rospy.init_node('test_fixed_system')
    
    print("========== 测试修复后的系统 ==========")
    
    # 1. 检查关键话题是否存在
    print("1. 检查关键话题...")
    topics = rospy.get_published_topics()
    topic_names = [topic[0] for topic in topics]
    
    required_topics = ['/joint_states', '/joint_states_single']
    for topic in required_topics:
        if topic in topic_names:
            print(f"✅ {topic} 话题存在")
        else:
            print(f"❌ {topic} 话题不存在")
    
    # 2. 检查/joint_states话题的数据频率
    print("\n2. 检查/joint_states话题数据...")
    
    joint_state_received = False
    def joint_state_callback(msg):
        global joint_state_received
        joint_state_received = True
        print(f"✅ 接收到/joint_states数据: {len(msg.position)}个关节")
        print(f"   时间戳: {msg.header.stamp}")
        print(f"   关节名称: {msg.name}")
    
    sub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    
    # 等待3秒检查数据
    start_time = time.time()
    while time.time() - start_time < 3 and not joint_state_received:
        rospy.sleep(0.1)
    
    if not joint_state_received:
        print("❌ 3秒内未接收到/joint_states数据")
    
    sub.unregister()
    
    # 3. 测试MoveIt服务
    print("\n3. 测试MoveIt服务...")
    service_name = '/joint_moveit_ctrl_endpose'
    
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        print(f"✅ {service_name} 服务可用")
        
        # 创建服务客户端
        client = rospy.ServiceProxy(service_name, JointMoveitCtrl)
        
        # 创建测试请求
        request = JointMoveitCtrlRequest()
        request.joint_endpose = [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]  # 简单的位姿
        request.max_velocity = 0.3
        request.max_acceleration = 0.3
        
        print("发送末端位姿控制请求...")
        start_time = time.time()
        response = client(request)
        execution_time = time.time() - start_time
        
        if response.status:
            print(f"✅ 末端位姿控制成功 (耗时: {execution_time:.2f}秒)")
        else:
            print(f"❌ 末端位姿控制失败，错误代码: {response.error_code}")
            
    except rospy.ROSException:
        print(f"❌ {service_name} 服务不可用")
    
    print("\n========== 测试完成 ==========")
    print("如果看到✅标记，说明系统工作正常")
    print("如果看到❌标记，请检查相关配置")

if __name__ == '__main__':
    try:
        test_system_status()
    except rospy.ROSInterruptException:
        print("测试被中断")