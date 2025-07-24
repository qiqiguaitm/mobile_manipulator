#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from arm_planner.msg import ArmTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class ArmPlannerNode:
    def __init__(self):
        rospy.init_node('arm_planner_node')
        
        # Load topic configurations
        topics = rospy.get_param('topics', {})
        
        # Publishers for planner outputs
        trajectory_topic = topics.get('outputs', {}).get('arm_trajectory', {}).get('name', '/arm/trajectory')
        end_pose_topic = topics.get('outputs', {}).get('end_pose', {}).get('name', '/arm/end_pose')
        
        self.trajectory_pub = rospy.Publisher(trajectory_topic, ArmTrajectory, queue_size=10)
        self.end_pose_pub = rospy.Publisher(end_pose_topic, PoseStamped, queue_size=10)
        
        # Subscribers for planning inputs
        self.map_sub = rospy.Subscriber('/map/octomap', PoseStamped, self.map_callback)
        self.chassis_pose_sub = rospy.Subscriber('/chassis/pose', PoseStamped, self.chassis_pose_callback)
        self.target_pose_sub = rospy.Subscriber('/task_mgr/target_object/pose', PoseStamped, self.target_pose_callback)
        self.joint_state_sub = rospy.Subscriber('/arm/joint_states', JointState, self.joint_state_callback)
        
        # Store latest data for planning
        self.latest_target_pose = None
        self.latest_joint_state = None
        
        rospy.loginfo("ArmPlanner node initialized")

    def map_callback(self, msg):
        """Callback for octomap data"""
        # In a real implementation, this would use the map for obstacle avoidance
        rospy.logdebug("Received octomap data")

    def chassis_pose_callback(self, msg):
        """Callback for chassis pose"""
        # In a real implementation, this would use chassis pose for planning
        rospy.logdebug("Received chassis pose")

    def target_pose_callback(self, msg):
        """Callback for target object pose"""
        self.latest_target_pose = msg
        rospy.logdebug("Received target pose")
        # Trigger planning when we receive a new target
        self.plan_trajectory()

    def joint_state_callback(self, msg):
        """Callback for arm joint states"""
        self.latest_joint_state = msg
        rospy.logdebug("Received joint states")

    def plan_trajectory(self):
        """Generate a simple trajectory to the target pose"""
        if self.latest_target_pose is None:
            return
            
        rospy.loginfo("Planning trajectory to target pose")
        
        # Create ArmTrajectory message
        trajectory_msg = ArmTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = "base_link"
        
        # Define joint names (example for a 6-DOF arm)
        trajectory_msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Create trajectory points
        # For simplicity, we'll create a single point trajectory
        point = JointTrajectoryPoint()
        
        # Generate target joint positions based on target pose
        # In a real implementation, this would use inverse kinematics
        target_positions = self.calculate_target_positions(self.latest_target_pose)
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)
        point.accelerations = [0.0] * len(target_positions)
        point.time_from_start = rospy.Duration(1.0)  # 1 second to reach target
        
        trajectory_msg.points = [point]
        trajectory_msg.end_pose = self.latest_target_pose.pose
        
        # Publish the trajectory
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("Published arm trajectory with %d points", len(trajectory_msg.points))

    def calculate_target_positions(self, target_pose):
        """Simple calculation of target joint positions (placeholder for IK)"""
        # In a real implementation, this would use inverse kinematics
        # For now, we'll just generate some example positions
        return [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Periodically check if we need to plan
            if self.latest_target_pose is not None:
                # In a real implementation, this might trigger replanning
                pass
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ArmPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass