#!/usr/bin/env python3
"""
机械臂问题深度诊断 - 找出真正的阻塞点
"""
import rospy
import subprocess
import time

def run_diagnosis():
    print("=== 机械臂控制问题诊断 ===\n")
    
    # 1. 检查ROS系统
    print("1. 检查ROS系统状态")
    try:
        # 检查节点
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"   活跃节点数: {len(nodes)}")
            if '/piper_ctrl_single_node' in nodes:
                print("   ✓ piper_ctrl_single_node 正在运行")
            else:
                print("   ✗ piper_ctrl_single_node 未运行")
                print("   解决方案: roslaunch arm_controller start_single_piper.launch")
                return
        else:
            print("   ✗ ROS Master未运行")
            print("   解决方案: roscore")
            return
    except:
        print("   ✗ 无法连接ROS系统")
        return
    
    # 2. 检查话题
    print("\n2. 检查关键话题")
    result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        topics = result.stdout.strip().split('\n')
        critical_topics = {
            '/joint_states': '关节状态反馈',
            '/joint_ctrl': '关节控制输入',
            '/arm_status': '机械臂状态',
            '/enable_flag': '使能标志'
        }
        for topic, desc in critical_topics.items():
            if topic in topics:
                print(f"   ✓ {topic} ({desc})")
            else:
                print(f"   ✗ {topic} ({desc}) - 缺失")
    
    # 3. 检查话题数据流
    print("\n3. 检查数据流")
    # 检查joint_states是否有数据
    result = subprocess.run(['timeout', '2', 'rostopic', 'hz', '/joint_states'], 
                          capture_output=True, text=True)
    if 'average rate' in result.stdout:
        lines = result.stdout.strip().split('\n')
        for line in lines:
            if 'average rate' in line:
                print(f"   /joint_states 发布频率: {line}")
    else:
        print("   ✗ /joint_states 无数据")
    
    # 4. 检查服务
    print("\n4. 检查控制服务")
    services = ['/enable_srv', '/stop_srv', '/go_zero_srv', '/gripper_srv']
    for srv in services:
        result = subprocess.run(['rosservice', 'info', srv], 
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            print(f"   ✓ {srv} 可用")
        else:
            print(f"   ✗ {srv} 不可用")
    
    # 5. 检查参数
    print("\n5. 检查启动参数")
    params = ['/piper_ctrl_single_node/can_port', 
              '/piper_ctrl_single_node/auto_enable',
              '/piper_ctrl_single_node/gripper_exist']
    for param in params:
        result = subprocess.run(['rosparam', 'get', param], 
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            print(f"   {param}: {result.stdout.strip()}")
    
    # 6. 检查CAN接口
    print("\n6. 检查CAN接口")
    result = subprocess.run(['ip', 'link', 'show'], capture_output=True, text=True)
    if 'can0' in result.stdout:
        # 检查can0状态
        for line in result.stdout.split('\n'):
            if 'can0' in line:
                if 'UP' in line:
                    print("   ✓ can0 接口已启动")
                else:
                    print("   ✗ can0 接口未启动")
                    print("   解决方案: sudo ip link set can0 up type can bitrate 1000000")
                break
    else:
        print("   ✗ can0 接口不存在")
        print("   解决方案: sudo ip link add dev can0 type can")
    
    print("\n=== 诊断总结 ===")
    print("根据代码分析，控制失败的最可能原因：")
    print("1. 软件使能标志(__enable_flag)未设置")
    print("2. 阻塞控制标志(block_ctrl_flag)被设置") 
    print("3. CAN通信问题")
    print("\n建议操作：")
    print("1. 重启机械臂节点: roslaunch arm_planner main_demo.launch mode:=real")
    print("2. 手动调用使能: rosservice call /enable_srv 'enable_request: true'")
    print("3. 运行测试脚本: python3 scripts/_cc_direct_arm_test.py")

if __name__ == "__main__":
    run_diagnosis()