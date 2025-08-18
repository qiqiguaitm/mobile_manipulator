#!/usr/bin/env python3
"""
直接测试机械臂控制 - Linus风格：简单、直接、无废话
"""
import rospy
from sensor_msgs.msg import JointState
from arm_planner.srv import Enable
from std_msgs.msg import Bool

def main():
    rospy.init_node('direct_arm_test')
    
    # 1. 强制使能
    print("1. 调用使能服务...")
    try:
        enable_srv = rospy.ServiceProxy('/enable_srv', Enable)
        rospy.wait_for_service('/enable_srv', timeout=5)
        resp = enable_srv(True)
        print(f"   使能响应: {resp.enable_response}")
    except Exception as e:
        print(f"   使能失败: {e}")
    
    # 2. 发布使能标志（备用方案）
    print("\n2. 发布使能标志...")
    enable_pub = rospy.Publisher('/enable_flag', Bool, queue_size=1)
    rospy.sleep(0.5)
    enable_pub.publish(Bool(data=True))
    print("   已发布enable_flag=True")
    
    # 3. 等待并检查当前位置
    print("\n3. 获取当前关节位置...")
    try:
        current = rospy.wait_for_message('/joint_states', JointState, timeout=5)
        print(f"   当前位置: {[f'{p:.3f}' for p in current.position[:6]]}")
    except:
        print("   无法获取关节状态")
        return
    
    # 4. 发送控制指令
    print("\n4. 发送控制指令...")
    cmd_pub = rospy.Publisher('/joint_ctrl', JointState, queue_size=1)
    rospy.sleep(0.5)
    
    # 构造测试指令 - 只移动第一个关节
    cmd = JointState()
    cmd.header.stamp = rospy.Time.now()
    cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    cmd.position = list(current.position[:6])
    cmd.position[0] += 0.2  # 第一个关节增加0.2弧度（约11度）
    
    print(f"   目标位置: {[f'{p:.3f}' for p in cmd.position]}")
    
    # 连续发送指令（确保接收）
    rate = rospy.Rate(10)  # 10Hz
    for i in range(20):  # 发送2秒
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        rate.sleep()
    
    # 5. 检查结果
    print("\n5. 检查执行结果...")
    rospy.sleep(1)
    try:
        final = rospy.wait_for_message('/joint_states', JointState, timeout=5)
        delta = final.position[0] - current.position[0]
        print(f"   最终位置: {[f'{p:.3f}' for p in final.position[:6]]}")
        print(f"   关节1移动量: {delta:.3f} 弧度")
        
        if abs(delta) > 0.01:
            print("\n✓ 机械臂控制正常")
        else:
            print("\n✗ 机械臂未响应控制指令")
            print("\n可能原因：")
            print("  1. piper_ctrl_single_node.py 中的 block_ctrl_flag=True")
            print("  2. piper_ctrl_single_node.py 中的 __enable_flag=False") 
            print("  3. CAN通信问题")
            print("  4. 机械臂硬件保护（急停、限位等）")
    except:
        print("   无法获取最终状态")

if __name__ == "__main__":
    main()