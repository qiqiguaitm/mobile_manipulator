#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import time

class FakeToRealBridge:
    def __init__(self):
        rospy.init_node('fake_to_real_bridge')
        
        # 订阅MoveIt fake控制器的命令
        self.fake_command_sub = rospy.Subscriber(
            '/move_group/fake_controller_joint_states', 
            JointState, 
            self.fake_command_callback
        )
        
        # 订阅真实机械臂的状态反馈
        self.real_sub = rospy.Subscriber(
            '/joint_states_single', 
            JointState, 
            self.real_callback
        )
        
        # 为MoveIt发布当前状态到 /joint_states
        self.joint_state_pub = rospy.Publisher(
            '/joint_states', 
            JointState, 
            queue_size=10
        )
        
        # 发布命令到真实机械臂
        self.real_command_pub = rospy.Publisher(
            '/joint_ctrl_commands', 
            JointState, 
            queue_size=10
        )
        
        self.last_real_state = None
        self.last_fake_command = None
        self.bridge_active = True
        
        # 启动状态发布线程
        self.feedback_thread = threading.Thread(target=self.feedback_publisher)
        self.feedback_thread.daemon = True
        self.feedback_thread.start()
        
        rospy.loginfo("Fake to Real Bridge initialized - bridging MoveIt commands to real robot")
    
    def fake_command_callback(self, msg):
        """接收MoveIt fake控制器的命令并转发给真实机械臂"""
        rospy.loginfo(f"Received MoveIt command: {len(msg.position)} joints: {[round(p, 4) for p in msg.position]}")
        
        # 确保命令包含所有7个关节
        full_positions = [0.0] * 7
        full_velocities = [0.0] * 7
        full_efforts = [0.0] * 7
        full_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # 根据关节名称映射位置
        for i, name in enumerate(msg.name):
            if name in full_names:
                joint_idx = full_names.index(name)
                if i < len(msg.position):
                    full_positions[joint_idx] = msg.position[i]
                if msg.velocity and i < len(msg.velocity):
                    full_velocities[joint_idx] = msg.velocity[i]
                if msg.effort and i < len(msg.effort):
                    full_efforts[joint_idx] = msg.effort[i]
        
        # 创建命令消息发送给真实机械臂
        command_msg = JointState()
        command_msg.header = Header()
        command_msg.header.stamp = rospy.Time.now()
        command_msg.header.frame_id = ""
        command_msg.name = full_names
        command_msg.position = full_positions
        command_msg.velocity = full_velocities
        command_msg.effort = full_efforts
        
        # 发布到真实机械臂控制话题
        self.real_command_pub.publish(command_msg)
        self.last_fake_command = msg
        
        rospy.loginfo(f"Forwarded complete command to real robot: {[round(p, 4) for p in full_positions]}")
    
    def real_callback(self, msg):
        """接收真实机械臂的状态反馈"""
        self.last_real_state = msg
        rospy.logdebug(f"Received real robot state: {msg.position}")
    
    def feedback_publisher(self):
        """将真实机械臂状态发布到/joint_states供MoveIt使用"""
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            if self.last_real_state:
                # 创建状态消息
                feedback_msg = JointState()
                feedback_msg.header = Header()
                feedback_msg.header.stamp = rospy.Time.now()
                feedback_msg.header.frame_id = ""
                
                # 检查是否需要添加joint8（如果URDF模型中有的话）
                names = list(self.last_real_state.name)
                positions = list(self.last_real_state.position)
                velocities = list(self.last_real_state.velocity) if self.last_real_state.velocity else [0.0] * len(positions)
                efforts = list(self.last_real_state.effort) if self.last_real_state.effort else [0.0] * len(positions)
                
                # 如果只有7个关节但URDF期望8个（joint7和joint8为夹爪的两个手指）
                if len(names) == 7 and 'joint7' in names:
                    joint7_idx = names.index('joint7')
                    joint7_pos = positions[joint7_idx]
                    joint7_vel = velocities[joint7_idx] if joint7_idx < len(velocities) else 0.0
                    joint7_eff = efforts[joint7_idx] if joint7_idx < len(efforts) else 0.0
                    
                    # 添加joint8作为对称关节
                    names.append('joint8')
                    positions.append(-joint7_pos)  # 对称位置
                    velocities.append(-joint7_vel)  # 对称速度
                    efforts.append(joint7_eff)      # 相同力度
                
                feedback_msg.name = names
                feedback_msg.position = positions
                feedback_msg.velocity = velocities
                feedback_msg.effort = efforts
                
                # 发布到/joint_states供MoveIt使用
                self.joint_state_pub.publish(feedback_msg)
                
                rospy.logdebug(f"Published real state to /joint_states: {len(names)} joints")
            else:
                rospy.loginfo_throttle(5, "Waiting for real robot state...")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = FakeToRealBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass