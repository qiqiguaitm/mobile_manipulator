#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class GripperControlService:
    """夹爪控制服务节点"""
    
    def __init__(self):
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('gripper_control_service')
        
        # 尝试初始化gripper组，如果失败则使用arm组
        try:
            self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
            self.use_gripper_group = True
            rospy.loginfo("Using gripper group for control")
        except:
            self.gripper_group = None
            self.use_gripper_group = False
            rospy.logwarn("Gripper group not found, using arm group for gripper control")
        
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        # 设置速度和加速度
        if self.use_gripper_group:
            self.gripper_group.set_max_velocity_scaling_factor(0.5)
            self.gripper_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        
        # 创建服务
        self.open_service = rospy.Service('gripper/open', SetBool, self.handle_open)
        self.close_service = rospy.Service('gripper/close', SetBool, self.handle_close)
        
        # 创建订阅器，用于设置夹爪位置
        self.position_sub = rospy.Subscriber('gripper/set_position', Float32, self.handle_set_position)
        
        # 创建发布器，发布夹爪状态
        self.state_pub = rospy.Publisher('gripper/state', JointState, queue_size=10)
        
        # 定时发布夹爪状态
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_gripper_state)
        
        rospy.loginfo("Gripper control service ready")
    
    def handle_open(self, req):
        """处理打开夹爪请求"""
        try:
            if self.use_gripper_group:
                # 使用gripper组
                self.gripper_group.set_named_target("open")
                success = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
            else:
                # 使用arm组，只修改夹爪关节
                current_joints = self.arm_group.get_current_joint_values()
                target_joints = list(current_joints)
                target_joints[6] = 0.035  # joint7
                target_joints[7] = 0.035  # joint8
                
                self.arm_group.set_joint_value_target(target_joints)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
            
            if success:
                return SetBoolResponse(success=True, message="Gripper opened successfully")
            else:
                return SetBoolResponse(success=False, message="Failed to open gripper")
        
        except Exception as e:
            rospy.logerr(f"Error opening gripper: {e}")
            return SetBoolResponse(success=False, message=str(e))
    
    def handle_close(self, req):
        """处理关闭夹爪请求"""
        try:
            if self.use_gripper_group:
                # 使用gripper组
                self.gripper_group.set_named_target("close")
                success = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
            else:
                # 使用arm组，只修改夹爪关节
                current_joints = self.arm_group.get_current_joint_values()
                target_joints = list(current_joints)
                target_joints[6] = 0.0  # joint7
                target_joints[7] = 0.0  # joint8
                
                self.arm_group.set_joint_value_target(target_joints)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
            
            if success:
                return SetBoolResponse(success=True, message="Gripper closed successfully")
            else:
                return SetBoolResponse(success=False, message="Failed to close gripper")
        
        except Exception as e:
            rospy.logerr(f"Error closing gripper: {e}")
            return SetBoolResponse(success=False, message=str(e))
    
    def handle_set_position(self, msg):
        """处理设置夹爪位置请求"""
        try:
            position = max(0.0, min(0.035, msg.data))  # 限制范围
            
            if self.use_gripper_group:
                # 使用gripper组
                gripper_joints = [position, position]
                self.gripper_group.set_joint_value_target(gripper_joints)
                success = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
            else:
                # 使用arm组
                current_joints = self.arm_group.get_current_joint_values()
                target_joints = list(current_joints)
                target_joints[6] = position  # joint7
                target_joints[7] = position  # joint8
                
                self.arm_group.set_joint_value_target(target_joints)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
            
            if success:
                rospy.loginfo(f"Gripper position set to {position}")
            else:
                rospy.logerr(f"Failed to set gripper position to {position}")
        
        except Exception as e:
            rospy.logerr(f"Error setting gripper position: {e}")
    
    def publish_gripper_state(self, event):
        """发布夹爪状态"""
        try:
            state_msg = JointState()
            state_msg.header.stamp = rospy.Time.now()
            
            if self.use_gripper_group:
                gripper_values = self.gripper_group.get_current_joint_values()
                state_msg.name = ['joint7', 'joint8']
                state_msg.position = gripper_values
            else:
                arm_values = self.arm_group.get_current_joint_values()
                state_msg.name = ['joint7', 'joint8']
                state_msg.position = [arm_values[6], arm_values[7]]
            
            self.state_pub.publish(state_msg)
        
        except Exception as e:
            rospy.logerr_once(f"Error publishing gripper state: {e}")
    
    def spin(self):
        """运行服务"""
        rospy.spin()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    service = GripperControlService()
    service.spin()