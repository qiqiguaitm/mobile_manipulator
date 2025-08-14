#!/usr/bin/env python3

import rospy
from task_mgr.msg import task_mgr
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import uuid

class TaskManagerNode:
    def __init__(self):
        rospy.init_node('task_mgr_node')
        
        # Load topic configurations
        topics = rospy.get_param('topics')
        
        # Publisher for task manager status
        self.task_pub = rospy.Publisher(topics['outputs']['status']['name'], task_mgr, queue_size=10)
        
        # Publisher for target object pose
        self.target_pose_pub = rospy.Publisher('/task_mgr/target_object/pose', PoseStamped, queue_size=10)
        
        # Subscribers for system state
        self.arm_state_sub = rospy.Subscriber(topics['inputs']['arm_state']['name'], JointState, self.arm_state_callback)
        self.chassis_state_sub = rospy.Subscriber(topics['inputs']['chassis_state']['name'], JointState, self.chassis_state_callback)
        self.target_list_sub = rospy.Subscriber(topics['inputs']['target_list']['name'], PoseStamped, self.target_list_callback)
        
        # Task manager message
        self.task_msg = task_mgr()
        self.task_msg.fsm_state = "idle"
        self.task_msg.task_list = ["pick_object", "place_object"]
        self.task_msg.cur_task_id = ""
        self.task_msg.cur_task_name = ""
        self.task_msg.cur_stage = ""
        self.task_msg.cur_goal = ""
        self.task_msg.target_object = ""
        
        # System state tracking
        self.arm_state = None
        self.chassis_state = None
        self.target_list = None
        
        # Task execution state
        self.current_task_index = 0
        self.task_in_progress = False
        
        rospy.loginfo("TaskManager node initialized")

    def arm_state_callback(self, msg):
        """Callback for arm joint state"""
        self.arm_state = msg
        rospy.logdebug("Received arm state with %d joints", len(msg.name) if msg.name else 0)

    def chassis_state_callback(self, msg):
        """Callback for chassis state"""
        self.chassis_state = msg
        rospy.logdebug("Received chassis state")

    def target_list_callback(self, msg):
        """Callback for target list"""
        self.target_list = msg
        rospy.logdebug("Received target list")
        # When we receive a target, start processing it
        self.process_target(msg)

    def process_target(self, target_msg):
        """Process a target and initiate task execution"""
        rospy.loginfo("Processing target for task execution")
        
        # Update task message
        self.task_msg.fsm_state = "planning"
        self.task_msg.cur_task_id = str(uuid.uuid4())
        self.task_msg.cur_task_name = "pick_object"
        self.task_msg.cur_stage = "target_acquisition"
        self.task_msg.cur_goal = "move_arm_to_target"
        self.task_msg.target_object = "object_1"
        
        # Publish updated task status
        self.task_pub.publish(self.task_msg)
        
        # Publish target pose for arm planner
        self.target_pose_pub.publish(target_msg)
        rospy.loginfo("Published target pose to arm planner")

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Update task state based on system state
            if self.arm_state and self.chassis_state:
                if self.task_msg.fsm_state == "planning":
                    self.task_msg.fsm_state = "executing"
                elif self.task_msg.fsm_state == "executing":
                    # Check if arm has reached target position
                    if self.arm_state and len(self.arm_state.position) > 0:
                        # Simple check - in reality this would be more complex
                        if abs(self.arm_state.position[0] - 0.1) < 0.01:  # Target position
                            self.task_msg.fsm_state = "completed"
                            
            # Publish current task status
            self.task_pub.publish(self.task_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TaskManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass