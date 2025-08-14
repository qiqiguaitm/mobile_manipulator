#!/usr/bin/env python3

import rospy
import time
from sensor_msgs.msg import JointState
from arm_planner.msg import ArmTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmControllerNode:
    def __init__(self):
        rospy.init_node('arm_controller_node')
        
        # Load topic configurations
        topics = rospy.get_param('topics', {})
        
        # Publishers for controller outputs
        joint_state_topic = topics.get('outputs', {}).get('joint_state', {}).get('name', '/arm/joint_states')
        self.joint_state_pub = rospy.Publisher(joint_state_topic, JointState, queue_size=10)
        
        # Publisher to send commands to Piper hardware
        self.piper_joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribers for control inputs
        trajectory_topic = topics.get('inputs', {}).get('trajectory', {}).get('name', '/arm/trajectory')
        self.trajectory_sub = rospy.Subscriber(trajectory_topic, ArmTrajectory, self.trajectory_callback)
        
        # Subscribe to hardware joint states from Piper
        self.hardware_joint_sub = rospy.Subscriber('/joint_states_single', JointState, self.hardware_joint_callback)
        
        # Current joint state from hardware
        self.hardware_joint_names = []
        self.hardware_joint_positions = []
        self.hardware_joint_velocities = []
        self.hardware_joint_efforts = []
        
        # Trajectory execution
        self.trajectory_queue = []
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_index = 0
        
        # Hardware connection status
        self.hardware_connected = False
        self.last_hardware_msg_time = None
        
        rospy.loginfo("ArmController node initialized with Piper hardware integration")

    def hardware_joint_callback(self, msg):
        """Callback for hardware joint states from Piper"""
        self.hardware_joint_names = msg.name
        self.hardware_joint_positions = msg.position
        self.hardware_joint_velocities = msg.velocity
        self.hardware_joint_efforts = msg.effort
        self.hardware_connected = True
        self.last_hardware_msg_time = rospy.Time.now()

    def trajectory_callback(self, msg):
        """Callback for arm trajectory commands"""
        rospy.loginfo("Received trajectory command with %d points", len(msg.points))
        rospy.loginfo("Joint names: %s", msg.joint_names)
        if msg.points:
            rospy.loginfo("First point positions: %s", msg.points[0].positions)
        
        # Check hardware connection
        if not self.hardware_connected:
            rospy.logwarn("Hardware not connected. Cannot execute trajectory.")
            return
        
        # Store the trajectory for execution
        self.current_trajectory = msg
        self.trajectory_start_time = rospy.Time.now()
        self.trajectory_index = 0

    def execute_trajectory(self):
        """Execute the current trajectory"""
        if self.current_trajectory is None:
            return
            
        if self.trajectory_index >= len(self.current_trajectory.points):
            rospy.loginfo("Trajectory execution completed")
            self.current_trajectory = None
            return
            
        # Get the current point in the trajectory
        point = self.current_trajectory.points[self.trajectory_index]
        
        # Calculate time since trajectory start
        elapsed_time = (rospy.Time.now() - self.trajectory_start_time).to_sec()
        
        # Check if we should move to the next point
        if elapsed_time >= point.time_from_start.to_sec():
            # Send command to Piper hardware
            self.send_joint_command(point.positions, point.velocities if point.velocities else None)
            
            rospy.loginfo("Executing trajectory point %d: positions=%s", self.trajectory_index, point.positions)
            
            # Move to next point
            self.trajectory_index += 1

    def send_joint_command(self, positions, velocities=None):
        """Send joint position command to Piper hardware"""
        if not self.hardware_connected:
            rospy.logwarn("Cannot send command: hardware not connected")
            return
        
        # Create joint state command for Piper
        cmd_msg = JointState()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.header.frame_id = "base_link"
        
        # Map from 6-DOF arm commands to 7-DOF Piper (arm + gripper)
        if len(positions) == 6:
            cmd_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
            cmd_msg.position = list(positions) + [0.0]  # Add gripper position (closed)
            if velocities:
                cmd_msg.velocity = list(velocities) + [0.0]
            else:
                cmd_msg.velocity = [0.0] * 7
        else:
            cmd_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
            cmd_msg.position = list(positions)
            cmd_msg.velocity = list(velocities) if velocities else [0.0] * len(positions)
        
        cmd_msg.effort = [0.0] * len(cmd_msg.position)
        
        # Send command to Piper hardware
        self.piper_joint_pub.publish(cmd_msg)
        rospy.logdebug("Sent joint command to Piper: %s", cmd_msg.position)

    def publish_joint_states(self):
        """Publish current joint states from hardware or default values if hardware not connected"""
        # Create joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = "base_link"
        
        if self.hardware_connected and self.hardware_joint_names:
            # Use hardware data if available
            # Filter to only arm joints (exclude gripper for arm controller)
            arm_joint_names = []
            arm_positions = []
            arm_velocities = []
            arm_efforts = []
            
            for i, name in enumerate(self.hardware_joint_names):
                if name.startswith('joint') and name != 'gripper':
                    arm_joint_names.append(name)
                    arm_positions.append(self.hardware_joint_positions[i])
                    arm_velocities.append(self.hardware_joint_velocities[i])
                    arm_efforts.append(self.hardware_joint_efforts[i])
            
            joint_state_msg.name = arm_joint_names
            joint_state_msg.position = arm_positions
            joint_state_msg.velocity = arm_velocities
            joint_state_msg.effort = arm_efforts
        else:
            # Use default values when hardware is not connected
            joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish joint states
        self.joint_state_pub.publish(joint_state_msg)

    def check_hardware_connection(self):
        """Check if hardware connection is still active"""
        if self.last_hardware_msg_time is not None:
            time_since_last_msg = (rospy.Time.now() - self.last_hardware_msg_time).to_sec()
            if time_since_last_msg > 2.0:  # 2 second timeout
                if self.hardware_connected:
                    rospy.logwarn("Hardware connection lost!")
                self.hardware_connected = False

    def run(self):
        """Main loop"""
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            # Check hardware connection
            self.check_hardware_connection()
            
            # Execute trajectory if we have one
            if self.current_trajectory is not None:
                self.execute_trajectory()
            
            # Publish joint states
            self.publish_joint_states()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ArmControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass