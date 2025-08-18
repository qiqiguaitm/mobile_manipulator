#!/usr/bin/env python3
"""
机械臂控制问题诊断脚本
"""
import rospy
import rostopic
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from arm_planner.srv import Enable
from arm_planner.msg import PiperStatusMsg
import sys
import time

def check_topics():
    """检查关键话题是否存在"""
    print("\n=== 1. 检查ROS话题 ===")
    topics = rospy.get_published_topics()
    key_topics = ['/joint_states', '/arm_status', '/joint_ctrl', '/enable_flag']
    
    for topic in key_topics:
        found = any(t[0] == topic for t in topics)
        print(f"{topic}: {'✓ 存在' if found else '✗ 不存在'}")
    
    # 检查话题发布频率
    print("\n=== 2. 检查话题发布频率 ===")
    try:
        rate = rostopic.ROSTopicHz(10)
        rospy.Subscriber('/joint_states', JointState, rate.callback_hz)
        time.sleep(2)
        if rate.get_hz():
            print(f"/joint_states 发布频率: {rate.get_hz()[0]:.1f} Hz")
        else:
            print("/joint_states 没有数据发布")
    except:
        print("/joint_states 话题异常")

def check_arm_status():
    """检查机械臂状态"""
    print("\n=== 3. 检查机械臂状态 ===")
    try:
        msg = rospy.wait_for_message('/arm_status', PiperStatusMsg, timeout=2)
        print(f"使能状态: {msg.arm_enable}")
        print(f"错误状态: {msg.arm_error}")
        print(f"抱闸状态: {msg.arm_brake}")
        return msg.arm_enable
    except rospy.ROSException:
        print("无法获取机械臂状态")
        return False

def check_enable_service():
    """检查使能服务"""
    print("\n=== 4. 检查使能服务 ===")
    try:
        rospy.wait_for_service('/enable_srv', timeout=2)
        print("/enable_srv 服务可用")
        return True
    except:
        print("/enable_srv 服务不可用")
        return False

def test_joint_control():
    """测试关节控制"""
    print("\n=== 5. 测试关节控制 ===")
    
    # 发布测试指令
    pub = rospy.Publisher('/joint_ctrl', JointState, queue_size=1)
    rospy.sleep(0.5)  # 等待发布器就绪
    
    # 获取当前关节位置
    try:
        current = rospy.wait_for_message('/joint_states', JointState, timeout=2)
        print(f"当前关节位置: {[f'{p:.3f}' for p in current.position[:6]]}")
        
        # 发送小幅度移动指令
        test_cmd = JointState()
        test_cmd.header.stamp = rospy.Time.now()
        test_cmd.name = current.name[:6]
        test_cmd.position = list(current.position[:6])
        test_cmd.position[0] += 0.1  # joint1增加0.1弧度
        
        print(f"发送测试指令到joint1: {test_cmd.position[0]:.3f}")
        pub.publish(test_cmd)
        
        # 等待并检查是否有响应
        time.sleep(2)
        new_pos = rospy.wait_for_message('/joint_states', JointState, timeout=2)
        if abs(new_pos.position[0] - current.position[0]) > 0.01:
            print("✓ 机械臂有响应")
        else:
            print("✗ 机械臂无响应")
            
    except rospy.ROSException as e:
        print(f"测试失败: {e}")

def check_control_flow():
    """检查控制流程中的阻塞点"""
    print("\n=== 6. 检查控制流程 ===")
    
    # 检查block_ctrl_flag
    print("提示: 代码中有 block_ctrl_flag 可能阻塞控制")
    print("提示: 代码中 GetEnableFlag() 需要返回True才能控制")
    
    # 检查节点是否正常运行
    nodes = rosnode.get_node_names()
    if '/piper_ctrl_single_node' in nodes:
        print("✓ piper_ctrl_single_node 正在运行")
    else:
        print("✗ piper_ctrl_single_node 未运行")

if __name__ == "__main__":
    rospy.init_node('arm_control_tester', anonymous=True)
    
    print("=== 机械臂控制问题诊断 ===")
    
    # 执行检查
    check_topics()
    enabled = check_arm_status()
    check_enable_service()
    
    if enabled:
        test_joint_control()
    else:
        print("\n机械臂未使能，尝试使能...")
        try:
            enable_srv = rospy.ServiceProxy('/enable_srv', Enable)
            resp = enable_srv(True)
            if resp.success:
                print("使能成功")
                time.sleep(2)
                test_joint_control()
            else:
                print(f"使能失败: {resp.message}")
        except Exception as e:
            print(f"调用使能服务失败: {e}")
    
    check_control_flow()
    
    print("\n=== 诊断完成 ===")