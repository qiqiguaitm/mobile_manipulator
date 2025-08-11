#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
import time
import threading

class CommandFlowTester:
    def __init__(self):
        rospy.init_node('test_command_flow')
        
        self.fake_commands_received = []
        self.real_commands_received = []
        self.robot_states_received = []
        
        # 监听MoveIt fake控制器输出
        self.fake_sub = rospy.Subscriber(
            '/move_group/fake_controller_joint_states',
            JointState,
            self.fake_command_callback
        )
        
        # 监听发送给真实机械臂的命令
        self.real_command_sub = rospy.Subscriber(
            '/joint_ctrl_commands',
            JointState,
            self.real_command_callback
        )
        
        # 监听机械臂状态反馈
        self.robot_state_sub = rospy.Subscriber(
            '/joint_states_single',
            JointState,
            self.robot_state_callback
        )
        
        print("Command Flow Tester initialized")
        print("Monitoring command flow from MoveIt to real robot...")
    
    def fake_command_callback(self, msg):
        timestamp = rospy.Time.now().to_sec()
        self.fake_commands_received.append((timestamp, msg.position[:]))
        print(f"[{timestamp:.2f}] MoveIt fake command: {[round(p, 4) for p in msg.position]}")
    
    def real_command_callback(self, msg):
        timestamp = rospy.Time.now().to_sec()
        self.real_commands_received.append((timestamp, msg.position[:]))
        print(f"[{timestamp:.2f}] Real robot command: {[round(p, 4) for p in msg.position]}")
    
    def robot_state_callback(self, msg):
        timestamp = rospy.Time.now().to_sec()
        self.robot_states_received.append((timestamp, msg.position[:]))
        if len(self.robot_states_received) % 50 == 0:  # 每50个状态打印一次
            print(f"[{timestamp:.2f}] Robot state: {[round(p, 4) for p in msg.position]}")
    
    def test_moveit_command(self):
        """测试MoveIt命令"""
        print("\n========== 测试MoveIt命令 ==========")
        
        # 等待服务可用
        service_name = '/joint_moveit_ctrl_endpose'
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            print(f"✅ {service_name} 服务可用")
        except rospy.ROSException:
            print(f"❌ {service_name} 服务不可用")
            return False
        
        # 清空之前的记录
        self.fake_commands_received.clear()
        self.real_commands_received.clear()
        
        # 发送测试命令
        client = rospy.ServiceProxy(service_name, JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_endpose = [0.15, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0]
        request.max_velocity = 0.3
        request.max_acceleration = 0.3
        
        print("发送MoveIt命令...")
        start_time = time.time()
        response = client(request)
        
        if response.status:
            print(f"✅ MoveIt命令执行成功")
        else:
            print(f"❌ MoveIt命令执行失败: {response.error_code}")
            return False
        
        # 等待10秒观察命令流
        print("等待10秒观察命令传递...")
        time.sleep(10)
        
        # 分析结果
        print(f"\n========== 命令流分析 ==========")
        print(f"MoveIt fake控制器命令数量: {len(self.fake_commands_received)}")
        print(f"发送到真实机械臂的命令数量: {len(self.real_commands_received)}")
        print(f"机械臂状态反馈数量: {len(self.robot_states_received)}")
        
        if len(self.fake_commands_received) > 0:
            print("✅ MoveIt产生了控制命令")
        else:
            print("❌ MoveIt没有产生控制命令")
        
        if len(self.real_commands_received) > 0:
            print("✅ 命令被转发到真实机械臂")
        else:
            print("❌ 命令没有被转发到真实机械臂")
        
        if len(self.robot_states_received) > 0:
            print("✅ 接收到机械臂状态反馈")
        else:
            print("❌ 没有接收到机械臂状态反馈")
        
        return len(self.fake_commands_received) > 0 and len(self.real_commands_received) > 0

def main():
    try:
        tester = CommandFlowTester()
        
        # 等待3秒让订阅者初始化
        print("等待3秒初始化...")
        time.sleep(3)
        
        # 测试命令流
        success = tester.test_moveit_command()
        
        if success:
            print("\n✅ 命令流测试成功！MoveIt命令能够到达真实机械臂")
        else:
            print("\n❌ 命令流测试失败！需要检查桥接配置")
        
    except rospy.ROSInterruptException:
        print("测试被中断")

if __name__ == '__main__':
    main()