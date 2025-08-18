#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading

class FollowJointTrajectoryServer:
    def __init__(self):
        rospy.init_node('follow_joint_trajectory_server')
        
        # 创建多个action服务器以支持不同的控制器
        self.arm_server = actionlib.SimpleActionServer(
            '/arm_controller/follow_joint_trajectory',  # 注意: 是arm_controller不是arm_controllers
            FollowJointTrajectoryAction,
            self.execute_trajectory,
            False
        )
        
        self.gripper_server = actionlib.SimpleActionServer(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            self.execute_gripper_trajectory,
            False
        )
        
        self.piper_server = actionlib.SimpleActionServer(
            '/piper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            self.execute_trajectory,
            False
        )
        
        # 发布关节命令到真实机械臂
        # 使用相对话题名，可以通过remap重定向
        self.joint_pub = rospy.Publisher(
            'joint_ctrl',
            JointState,
            queue_size=10
        )
        
        # 订阅当前关节状态
        self.current_joint_states = None
        self.joint_sub = rospy.Subscriber(
            'joint_states',
            JointState,
            self.joint_states_callback
        )
        
        self.arm_server.start()
        self.gripper_server.start()
        self.piper_server.start()
        rospy.loginfo("FollowJointTrajectory servers started for arm, gripper and piper")
    
    def joint_states_callback(self, msg):
        """更新当前关节状态"""
        self.current_joint_states = msg
    
    def execute_trajectory(self, goal):
        """执行轨迹目标"""
        rospy.loginfo(f"Received trajectory goal with {len(goal.trajectory.points)} points")
        
        result = FollowJointTrajectoryResult()
        feedback = FollowJointTrajectoryFeedback()
        
        # 设置固定帧率50Hz
        rate = rospy.Rate(50)
        # 速度缩放因子 - 0.5表示慢一半
        velocity_scale = 0.5
        
        try:
            # 执行轨迹中的每个点
            for i, point in enumerate(goal.trajectory.points):
                # 检查是否被取消
                if self.arm_server.is_preempt_requested():
                    rospy.loginfo("Trajectory execution preempted")
                    self.arm_server.set_preempted()
                    return
                
                # 验证轨迹点数据
                if not goal.trajectory.joint_names or not point.positions:
                    rospy.logwarn(f"Empty trajectory data at point {i}: joint_names={len(goal.trajectory.joint_names)}, positions={len(point.positions)}")
                    continue
                    
                if len(point.positions) != len(goal.trajectory.joint_names):
                    rospy.logwarn(f"Mismatch at point {i}: {len(point.positions)} positions vs {len(goal.trajectory.joint_names)} joint names")
                    continue
                
                # 创建关节状态消息
                joint_cmd = JointState()
                joint_cmd.header = Header()
                joint_cmd.header.stamp = rospy.Time.now()
                joint_cmd.header.frame_id = ""
                
                # 使用轨迹中的关节名称
                joint_cmd.name = list(goal.trajectory.joint_names)
                joint_cmd.position = list(point.positions)
                # 应用速度缩放因子
                if point.velocities and len(point.velocities) == len(point.positions):
                    joint_cmd.velocity = [v * velocity_scale for v in point.velocities]
                else:
                    joint_cmd.velocity = [0.0] * len(point.positions)
                joint_cmd.effort = [0.0] * len(point.positions)
                
                # 发布关节命令
                self.joint_pub.publish(joint_cmd)
                
                # 每10个点输出一次调试信息
                if i % 10 == 0 or i == 0:
                    rospy.loginfo(f"Publishing to topic: {self.joint_pub.resolved_name}, positions: {[f'{p:.3f}' for p in joint_cmd.position[:6]]}")
                
                # 更新反馈
                feedback.header.stamp = rospy.Time.now()
                feedback.joint_names = goal.trajectory.joint_names
                feedback.desired = point
                if self.current_joint_states and len(self.current_joint_states.position) >= len(point.positions):
                    feedback.actual.positions = list(self.current_joint_states.position[:len(point.positions)])
                    feedback.actual.velocities = list(self.current_joint_states.velocity[:len(point.positions)]) if self.current_joint_states.velocity else [0.0] * len(point.positions)
                else:
                    feedback.actual.positions = list(point.positions)
                    feedback.actual.velocities = [0.0] * len(point.positions)
                
                self.arm_server.publish_feedback(feedback)
                
                rospy.loginfo(f"Executing point {i+1}/{len(goal.trajectory.points)}")
                
                # 使用固定50Hz帧率
                rate.sleep()
            
            # 成功完成
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            result.error_string = "Trajectory executed successfully"
            self.arm_server.set_succeeded(result)
            rospy.loginfo("Trajectory execution completed successfully")
            
        except Exception as e:
            rospy.logerr(f"Trajectory execution failed: {str(e)}")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = str(e)
            self.arm_server.set_aborted(result)
            
    def execute_gripper_trajectory(self, goal):
        """执行夹爪轨迹目标"""
        rospy.loginfo(f"Received gripper trajectory goal with {len(goal.trajectory.points)} points")
        
        result = FollowJointTrajectoryResult()
        feedback = FollowJointTrajectoryFeedback()
        
        rate = rospy.Rate(50)  # 50Hz
        
        try:
            for i, point in enumerate(goal.trajectory.points):
                if self.gripper_server.is_preempt_requested():
                    rospy.loginfo("Gripper trajectory execution preempted")
                    self.gripper_server.set_preempted()
                    return
                
                # 创建关节状态消息
                joint_cmd = JointState()
                joint_cmd.header.stamp = rospy.Time.now()
                joint_cmd.name = list(goal.trajectory.joint_names)
                joint_cmd.position = list(point.positions)
                joint_cmd.velocity = [0.0] * len(point.positions)
                joint_cmd.effort = [1.0] * len(point.positions)  # 夹爪默认力度
                
                # 发布关节命令
                self.joint_pub.publish(joint_cmd)
                
                # 更新反馈
                feedback.header.stamp = rospy.Time.now()
                feedback.joint_names = goal.trajectory.joint_names
                feedback.desired = point
                
                self.gripper_server.publish_feedback(feedback)
                rospy.loginfo(f"Executing gripper point {i+1}/{len(goal.trajectory.points)}")
                
                rate.sleep()
            
            # 成功完成
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            result.error_string = "Gripper trajectory executed successfully"
            self.gripper_server.set_succeeded(result)
            rospy.loginfo("Gripper trajectory execution completed successfully")
            
        except Exception as e:
            rospy.logerr(f"Gripper trajectory execution failed: {str(e)}")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = str(e)
            self.gripper_server.set_aborted(result)

if __name__ == '__main__':
    try:
        server = FollowJointTrajectoryServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass