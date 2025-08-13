#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""调试机械臂执行问题"""

import rospy
import rostopic
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from piper_msgs.srv import Enable, GetArmStatus
from piper_msgs.msg import ArmStatus
import time

class ExecutionDebugger:
    def __init__(self):
        rospy.init_node('execution_debugger', anonymous=True)
        
        self.joint_states_received = False
        self.trajectory_received = False
        self.action_goal_received = False
        self.arm_status = None
        
        # 订阅相关话题
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        rospy.Subscriber('/joint_ctrl_single', JointTrajectory, self.trajectory_cb)
        rospy.Subscriber('/arm_controller/follow_joint_trajectory/goal', 
                        FollowJointTrajectoryActionGoal, self.action_goal_cb)
        rospy.Subscriber('/arm_status', ArmStatus, self.arm_status_cb)
        
        print("\n=== Piper机械臂执行调试工具 ===\n")
        
    def joint_states_cb(self, msg):
        self.joint_states_received = True
        self.last_joint_states = msg
        
    def trajectory_cb(self, msg):
        self.trajectory_received = True
        print(f"[轨迹] 收到轨迹命令: {len(msg.points)} 个轨迹点")
        if msg.points:
            print(f"       目标位置: {[f'{p:.3f}' for p in msg.points[-1].positions[:6]]}")
        
    def action_goal_cb(self, msg):
        self.action_goal_received = True
        print(f"[Action] 收到轨迹执行请求")
        
    def arm_status_cb(self, msg):
        self.arm_status = msg
        
    def check_enable_status(self):
        """检查使能状态"""
        print("1. 检查机器人使能状态...")
        try:
            # 通过arm_status检查
            if self.arm_status:
                print(f"   ARM状态码: {self.arm_status.arm_status}")
                if self.arm_status.arm_status == 0x4C:  # 正常工作状态
                    print("   ✓ 机器人已使能（状态正常）")
                    return True
                else:
                    print(f"   ✗ 机器人状态异常: 0x{self.arm_status.arm_status:02X}")
                    
            # 尝试获取状态服务
            rospy.wait_for_service('/get_status_srv', timeout=2.0)
            get_status = rospy.ServiceProxy('/get_status_srv', GetArmStatus)
            resp = get_status()
            print(f"   状态服务返回: 0x{resp.arm_status:02X}")
            
            # 尝试使能
            print("   尝试使能机器人...")
            rospy.wait_for_service('/enable_srv', timeout=2.0)
            enable_srv = rospy.ServiceProxy('/enable_srv', Enable)
            resp = enable_srv(enable_request=True)
            if resp.enable_response:
                print("   ✓ 使能成功")
                return True
            else:
                print("   ✗ 使能失败")
                return False
                
        except Exception as e:
            print(f"   ✗ 检查使能状态失败: {e}")
            return False
            
    def check_topics(self):
        """检查话题连接"""
        print("\n2. 检查话题连接...")
        
        # 获取所有话题
        topics = rostopic.get_topic_list()[0]
        
        # 检查关键话题
        key_topics = {
            '/joint_states': '关节状态',
            '/joint_ctrl_single': '关节控制命令',
            '/arm_controller/follow_joint_trajectory/goal': '轨迹执行目标',
            '/arm_controller/follow_joint_trajectory/feedback': '轨迹执行反馈',
            '/arm_controller/follow_joint_trajectory/result': '轨迹执行结果',
            '/arm_status': '机械臂状态'
        }
        
        for topic, desc in key_topics.items():
            if any(t[0] == topic for t in topics):
                # 获取发布者和订阅者
                pubs, subs = rostopic.get_topic_type(topic)[1], rostopic.get_topic_type(topic)[2]
                print(f"   ✓ {topic} ({desc})")
                
                # 检查发布者
                pub_info = rostopic._master_get_topic_types('/')[topic]
                if pub_info:
                    publishers = rostopic._rostopic_cmd_info([topic], publishers_only=True)
                    if publishers:
                        print(f"      发布者: {publishers.strip()}")
                        
                # 检查订阅者  
                sub_info = rostopic._rostopic_cmd_info([topic], subscribers_only=True)
                if sub_info:
                    print(f"      订阅者: {sub_info.strip()}")
            else:
                print(f"   ✗ {topic} ({desc}) - 未找到")
                
    def check_data_flow(self):
        """检查数据流"""
        print("\n3. 检查数据流（等待10秒）...")
        
        start_time = time.time()
        while time.time() - start_time < 10:
            rospy.sleep(0.1)
            
        print(f"   关节状态接收: {'✓ 是' if self.joint_states_received else '✗ 否'}")
        print(f"   轨迹命令接收: {'✓ 是' if self.trajectory_received else '✗ 否'}")
        print(f"   Action目标接收: {'✓ 是' if self.action_goal_received else '✗ 否'}")
        
        if self.joint_states_received and hasattr(self, 'last_joint_states'):
            print(f"   当前关节位置: {[f'{p:.3f}' for p in self.last_joint_states.position[:6]]}")
            
    def check_nodes(self):
        """检查关键节点"""
        print("\n4. 检查关键节点...")
        
        key_nodes = [
            ('/move_group', 'MoveIt规划'),
            ('/piper_driver', '硬件驱动'),
            ('/trajectory_controller', '轨迹控制器'),
            ('/robot_state_publisher', '状态发布器')
        ]
        
        for node, desc in key_nodes:
            try:
                info = rospy.get_master().lookupNode(node)
                if info:
                    print(f"   ✓ {node} ({desc}) - 运行中")
            except:
                print(f"   ✗ {node} ({desc}) - 未运行")
                
    def test_simple_motion(self):
        """测试简单运动"""
        print("\n5. 测试直接发送轨迹命令...")
        
        try:
            # 获取当前位置
            current_joints = self.last_joint_states.position[:6] if hasattr(self, 'last_joint_states') else [0]*6
            print(f"   当前位置: {[f'{p:.3f}' for p in current_joints]}")
            
            # 创建测试轨迹
            traj_pub = rospy.Publisher('/joint_ctrl_single', JointTrajectory, queue_size=1)
            rospy.sleep(0.5)
            
            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            
            # 创建一个小幅度运动
            from trajectory_msgs.msg import JointTrajectoryPoint
            point = JointTrajectoryPoint()
            point.positions = list(current_joints)
            point.positions[0] += 0.05  # 第一个关节移动0.05弧度
            point.time_from_start = rospy.Duration(2.0)
            
            traj.points = [point]
            
            print("   发送测试轨迹（第一个关节+0.05弧度）...")
            traj_pub.publish(traj)
            
            # 等待并检查
            rospy.sleep(3.0)
            
            if hasattr(self, 'last_joint_states'):
                new_pos = self.last_joint_states.position[0]
                if abs(new_pos - current_joints[0]) > 0.01:
                    print(f"   ✓ 检测到运动: {current_joints[0]:.3f} -> {new_pos:.3f}")
                else:
                    print("   ✗ 未检测到运动")
                    
        except Exception as e:
            print(f"   ✗ 测试失败: {e}")
            
    def run(self):
        """运行所有检查"""
        # 等待系统稳定
        rospy.sleep(2.0)
        
        # 执行检查
        self.check_enable_status()
        self.check_topics()
        self.check_nodes()
        self.check_data_flow()
        self.test_simple_motion()
        
        # 诊断建议
        print("\n=== 诊断建议 ===")
        
        if not self.joint_states_received:
            print("• 未收到关节状态，检查硬件驱动是否正常")
            
        if self.action_goal_received but not self.trajectory_received:
            print("• Action目标已发送但未收到轨迹命令，检查轨迹控制器")
            
        if self.trajectory_received:
            print("• 轨迹已发送到硬件，如果机器人未动：")
            print("  - 检查机器人是否真正使能")
            print("  - 检查CAN通信是否正常: candump can0")
            print("  - 检查是否有错误状态码")
            
        print("\n按Ctrl+C退出...")
        rospy.spin()

if __name__ == '__main__':
    try:
        debugger = ExecutionDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass