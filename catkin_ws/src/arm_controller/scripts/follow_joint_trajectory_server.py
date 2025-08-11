#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from arm_planner.msg import ArmTrajectory

class FollowJointTrajectoryServer:
    def __init__(self):
        rospy.init_node('follow_joint_trajectory_server')
        
        # Create action server
        self.server = actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory', 
                                                   FollowJointTrajectoryAction, 
                                                   self.execute_trajectory, False)
        
        # Publisher for arm controller
        self.trajectory_pub = rospy.Publisher('/arm/trajectory', ArmTrajectory, queue_size=10)
        
        # Subscribe to joint states for feedback
        self.joint_state_sub = rospy.Subscriber('/arm/joint_states', JointState, self.joint_state_callback)
        
        # Current joint state
        self.current_joint_state = None
        
        self.server.start()
        rospy.loginfo("FollowJointTrajectory action server started")
    
    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg
    
    def execute_trajectory(self, goal):
        """Execute joint trajectory"""
        rospy.loginfo("Received trajectory with %d points", len(goal.trajectory.points))
        
        # Convert FollowJointTrajectory to ArmTrajectory message
        arm_traj = ArmTrajectory()
        arm_traj.header = goal.trajectory.header
        arm_traj.joint_names = goal.trajectory.joint_names
        arm_traj.points = goal.trajectory.points
        
        # Publish trajectory to arm controller
        self.trajectory_pub.publish(arm_traj)
        
        # Wait for trajectory execution (simplified - in real implementation would monitor progress)
        if goal.trajectory.points:
            last_point = goal.trajectory.points[-1]
            total_time = last_point.time_from_start.to_sec()
            rospy.loginfo("Waiting %.2f seconds for trajectory execution", total_time)
            rospy.sleep(total_time + 1.0)  # Add 1 second buffer
        
        # Return success
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.server.set_succeeded(result)
        rospy.loginfo("Trajectory execution completed")

if __name__ == '__main__':
    try:
        server = FollowJointTrajectoryServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass