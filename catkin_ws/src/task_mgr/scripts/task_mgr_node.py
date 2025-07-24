#!/usr/bin/env python3

import rospy
from task_mgr.msg import task_mgr
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class TaskManagerNode:
    def __init__(self):
        rospy.init_node('task_mgr_node')
        
        # Publisher for task manager status
        self.task_pub = rospy.Publisher('/task_mgr/status', task_mgr, queue_size=10)
        
        # Subscribers for system state
        self.arm_state_sub = rospy.Subscriber('/arm/joint_state', JointState, self.arm_state_callback)
        self.chassis_state_sub = rospy.Subscriber('/chassis/chassis_state', JointState, self.chassis_state_callback)
        self.target_list_sub = rospy.Subscriber('/router/target_list', PoseStamped, self.target_list_callback)
        
        # Task manager message
        self.task_msg = task_mgr()
        self.task_msg.fsm_state = "idle"
        self.task_msg.task_list = []
        self.task_msg.cur_task_id = ""
        self.task_msg.cur_task_name = ""
        self.task_msg.cur_stage = ""
        self.task_msg.cur_goal = ""
        self.task_msg.target_object = ""
        
        rospy.loginfo("TaskManager node initialized")

    def arm_state_callback(self, msg):
        """Callback for arm joint state"""
        rospy.logdebug("Received arm state")

    def chassis_state_callback(self, msg):
        """Callback for chassis state"""
        rospy.logdebug("Received chassis state")

    def target_list_callback(self, msg):
        """Callback for target list"""
        rospy.logdebug("Received target list")

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Publish current task status
            self.task_msg.fsm_state = "running"  # In a real implementation, this would change based on actual state
            self.task_pub.publish(self.task_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = TaskManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass