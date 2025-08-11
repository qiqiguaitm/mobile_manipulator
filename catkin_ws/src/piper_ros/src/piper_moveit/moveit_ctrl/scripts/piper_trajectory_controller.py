#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import time
import math

class PiperTrajectoryController:
    def __init__(self, controller_name):
        self.controller_name = controller_name
        self.current_joint_states = None
        self.trajectory_executing = False
        
        # 订阅当前关节状态
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states_single', 
            JointState, 
            self.joint_state_callback
        )
        
        # 发布关节命令
        self.joint_command_pub = rospy.Publisher(
            '/joint_ctrl_commands', 
            JointState, 
            queue_size=10
        )
        
        # 创建动作服务器
        self.action_server = actionlib.SimpleActionServer(
            f'{controller_name}/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory,
            auto_start=False
        )
        
        self.action_server.start()
        rospy.loginfo(f"Piper Trajectory Controller '{controller_name}' initialized")
    
    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        self.current_joint_states = msg
    
    def execute_trajectory(self, goal):
        """执行轨迹"""
        rospy.loginfo(f"Executing trajectory with {len(goal.trajectory.points)} points")
        
        success = True
        feedback = FollowJointTrajectoryFeedback()
        result = FollowJointTrajectoryResult()
        
        trajectory = goal.trajectory
        joint_names = trajectory.joint_names
        
        # 验证关节名称
        expected_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        if not all(joint in expected_joints for joint in joint_names):
            rospy.logerr(f"Invalid joint names: {joint_names}")
            result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            self.action_server.set_aborted(result)
            return
        
        self.trajectory_executing = True
        start_time = rospy.Time.now()
        
        try:
            # 执行轨迹中的每个点
            for i, point in enumerate(trajectory.points):
                if self.action_server.is_preempt_requested():
                    rospy.loginfo("Trajectory execution preempted")
                    result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                    self.action_server.set_preempted(result)
                    success = False
                    break
                
                # 发送关节命令
                self.send_joint_command(joint_names, point)
                
                # 等待到达目标时间
                target_time = start_time + point.time_from_start
                current_time = rospy.Time.now()
                if target_time > current_time:
                    sleep_duration = (target_time - current_time).to_sec()
                    rospy.sleep(sleep_duration)
                
                # 发送反馈
                feedback.joint_names = joint_names
                feedback.desired = point
                if self.current_joint_states:
                    feedback.actual.positions = self.get_joint_positions(joint_names)
                    feedback.actual.time_from_start = rospy.Time.now() - start_time
                feedback.error.positions = [0.0] * len(joint_names)  # 简化处理
                
                self.action_server.publish_feedback(feedback)
                
                rospy.logdebug(f"Executed trajectory point {i+1}/{len(trajectory.points)}")
            
            if success:
                # 验证最终位置
                if self.verify_final_position(joint_names, trajectory.points[-1]):
                    result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self.action_server.set_succeeded(result)
                    rospy.loginfo("Trajectory execution completed successfully")
                else:
                    result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                    self.action_server.set_aborted(result)
                    rospy.logwarn("Trajectory execution completed but final position tolerance violated")
        
        except Exception as e:
            rospy.logerr(f"Error during trajectory execution: {str(e)}")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self.action_server.set_aborted(result)
        
        finally:
            self.trajectory_executing = False
    
    def send_joint_command(self, joint_names, trajectory_point):
        """发送关节命令"""
        command = JointState()
        command.header = Header()
        command.header.stamp = rospy.Time.now()
        
        # 创建完整的7关节命令
        full_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        full_positions = [0.0] * 7
        full_velocities = [0.0] * 7
        full_efforts = [0.0] * 7
        
        # 映射轨迹点到完整关节列表
        for i, name in enumerate(joint_names):
            if name in full_names:
                joint_idx = full_names.index(name)
                if i < len(trajectory_point.positions):
                    full_positions[joint_idx] = trajectory_point.positions[i]
                if trajectory_point.velocities and i < len(trajectory_point.velocities):
                    full_velocities[joint_idx] = trajectory_point.velocities[i]
                if trajectory_point.effort and i < len(trajectory_point.effort):
                    full_efforts[joint_idx] = trajectory_point.effort[i]
        
        command.name = full_names
        command.position = full_positions
        command.velocity = full_velocities
        command.effort = full_efforts
        
        self.joint_command_pub.publish(command)
        
        rospy.logdebug(f"Sent joint command: {[round(p, 4) for p in full_positions]}")
    
    def get_joint_positions(self, joint_names):
        """获取指定关节的当前位置"""
        if not self.current_joint_states:
            return [0.0] * len(joint_names)
        
        positions = []
        for name in joint_names:
            if name in self.current_joint_states.name:
                idx = self.current_joint_states.name.index(name)
                if idx < len(self.current_joint_states.position):
                    positions.append(self.current_joint_states.position[idx])
                else:
                    positions.append(0.0)
            else:
                positions.append(0.0)
        
        return positions
    
    def verify_final_position(self, joint_names, final_point, tolerance=0.1):
        """验证最终位置是否在容差范围内"""
        if not self.current_joint_states:
            return True  # 如果没有状态反馈，假设成功
        
        current_positions = self.get_joint_positions(joint_names)
        
        for i, (current, target) in enumerate(zip(current_positions, final_point.positions)):
            error = abs(current - target)
            if error > tolerance:
                rospy.logwarn(f"Joint {joint_names[i]} position error: {error:.4f} > {tolerance}")
                return False
        
        return True

class PiperMultiTrajectoryController:
    """管理多个轨迹控制器的主类"""
    def __init__(self):
        rospy.init_node('piper_trajectory_controller')
        
        # 创建不同的控制器
        self.controllers = {}
        
        # 机械臂控制器（6个关节）
        self.controllers['arm_controller'] = PiperTrajectoryController('arm_controller')
        
        # 夹爪控制器（1个关节）
        self.controllers['gripper_controller'] = PiperTrajectoryController('gripper_controller')
        
        # 完整机械臂控制器（7个关节）
        self.controllers['piper_controller'] = PiperTrajectoryController('piper_controller')
        
        rospy.loginfo("Piper Multi-Trajectory Controller initialized")
        rospy.loginfo("Available controllers:")
        for name in self.controllers.keys():
            rospy.loginfo(f"  - {name}/follow_joint_trajectory")

if __name__ == '__main__':
    try:
        controller = PiperMultiTrajectoryController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass