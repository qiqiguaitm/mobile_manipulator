#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from arm_planner.msg import ArmTrajectory

class ArmPlannerTester:
    def __init__(self):
        rospy.init_node('arm_planner_tester')
        
        # Wait for ROS to initialize
        rospy.sleep(1.0)
        
        # Publishers for sending test inputs
        self.target_pose_pub = rospy.Publisher('/task_mgr/target_object/pose', PoseStamped, queue_size=10)
        self.joint_state_pub = rospy.Publisher('/arm/joint_states', JointState, queue_size=10)
        self.chassis_pose_pub = rospy.Publisher('/chassis/pose', PoseStamped, queue_size=10)
        
        # Subscriber for receiving outputs
        self.trajectory_sub = rospy.Subscriber('/arm/trajectory', ArmTrajectory, self.trajectory_callback)
        
        # Store received messages
        self.received_trajectory = None
        
        # Wait for connections to establish
        rospy.sleep(2.0)
        
        rospy.loginfo("ArmPlannerTester initialized")

    def trajectory_callback(self, msg):
        """Callback for receiving trajectory output"""
        self.received_trajectory = msg
        rospy.loginfo("Received trajectory with %d points", len(msg.points))
        if len(msg.points) > 0:
            rospy.loginfo("First point positions: %s", str(msg.points[0].positions))

    def create_target_pose(self):
        """Create a test target pose message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.5
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        return pose_msg

    def create_joint_state(self):
        """Create a test joint state message"""
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return joint_msg

    def create_chassis_pose(self):
        """Create a test chassis pose message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        return pose_msg

    def run_test(self):
        """Run the test sequence"""
        rospy.loginfo("Starting arm planner test...")
        
        # Wait for connections to establish
        rospy.sleep(2.0)
        
        # Send joint states first
        rospy.loginfo("Sending joint states...")
        joint_msg = self.create_joint_state()
        self.joint_state_pub.publish(joint_msg)
        rospy.loginfo("Published joint states: %s", str(joint_msg.position))
        
        # Send chassis pose
        rospy.loginfo("Sending chassis pose...")
        chassis_msg = self.create_chassis_pose()
        self.chassis_pose_pub.publish(chassis_msg)
        rospy.loginfo("Published chassis pose: (%f, %f, %f)", 
                     chassis_msg.pose.position.x, 
                     chassis_msg.pose.position.y, 
                     chassis_msg.pose.position.z)
        
        # Wait a bit
        rospy.sleep(1.0)
        
        # Send target pose to trigger planning
        rospy.loginfo("Sending target pose...")
        target_msg = self.create_target_pose()
        self.target_pose_pub.publish(target_msg)
        rospy.loginfo("Published target pose: (%f, %f, %f)", 
                     target_msg.pose.position.x, 
                     target_msg.pose.position.y, 
                     target_msg.pose.position.z)
        
        # Wait for response
        timeout = time.time() + 15.0  # 15 second timeout
        start_time = time.time()
        while self.received_trajectory is None and time.time() < timeout:
            rospy.sleep(0.1)
        
        # Check results
        elapsed_time = time.time() - start_time
        rospy.loginfo("Waited %.2f seconds for response", elapsed_time)
        
        if self.received_trajectory is not None:
            rospy.loginfo("Test PASSED: Received trajectory from arm_planner")
            rospy.loginfo("Received trajectory with %d points", len(self.received_trajectory.points))
            if len(self.received_trajectory.points) > 0:
                rospy.loginfo("First point positions: %s", str(self.received_trajectory.points[0].positions))
            return True
        else:
            rospy.logerr("Test FAILED: No trajectory received from arm_planner")
            return False

if __name__ == '__main__':
    try:
        tester = ArmPlannerTester()
        result = tester.run_test()
        if result:
            rospy.loginfo("Test completed successfully")
        else:
            rospy.logerr("Test failed")
    except rospy.ROSInterruptException:
        pass