#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ground Collision Manager for Mobile Manipulator
Adds virtual ground to MoveIt planning scene to prevent ground collisions
"""

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool, SetBoolResponse

class GroundCollisionManager:
    """Manager class for ground collision protection"""
    
    def __init__(self):
        # Initialize parameters
        self.ground_height = rospy.get_param('~ground_height', -0.05)  # 50mm safety margin
        self.ground_size_x = rospy.get_param('~ground_size_x', 10.0)  # 10m x 10m
        self.ground_size_y = rospy.get_param('~ground_size_y', 10.0)
        self.ground_thickness = rospy.get_param('~ground_thickness', 0.1)  # 100mm
        
        # Initialize MoveIt components
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        
        # Service to enable/disable ground
        self.enable_service = rospy.Service('~enable_ground_collision', SetBool, self.enable_ground_callback)
        
        rospy.sleep(1)  # Wait for initialization
        
        # Add ground on startup if enabled
        if rospy.get_param('~enable_on_startup', True):
            self.add_ground()
    
    def add_ground(self):
        """Add virtual ground to planning scene"""
        # Create ground pose
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = self.robot.get_planning_frame()
        ground_pose.pose.position.x = 0
        ground_pose.pose.position.y = 0
        ground_pose.pose.position.z = self.ground_height - self.ground_thickness/2
        ground_pose.pose.orientation.w = 1.0
        
        # Add ground box
        size = [self.ground_size_x, self.ground_size_y, self.ground_thickness]
        self.scene.add_box("ground", ground_pose, size=size)
        
        rospy.loginfo("Ground collision object added:")
        rospy.loginfo("  Height: %.3fm", self.ground_height)
        rospy.loginfo("  Size: %.1fm x %.1fm", self.ground_size_x, self.ground_size_y)
        
        # Verify
        rospy.sleep(0.5)
        if "ground" in self.scene.get_known_object_names():
            rospy.loginfo("Ground successfully added to planning scene")
            return True
        else:
            rospy.logerr("Failed to add ground to planning scene")
            return False
    
    def remove_ground(self):
        """Remove virtual ground from planning scene"""
        self.scene.remove_world_object("ground")
        rospy.loginfo("Ground collision object removed")
        return True
    
    def enable_ground_callback(self, req):
        """Service callback to enable/disable ground collision"""
        if req.data:
            success = self.add_ground()
        else:
            success = self.remove_ground()
        
        return SetBoolResponse(success=success, 
                              message="Ground collision %s" % ("enabled" if req.data else "disabled"))

def main():
    rospy.init_node('ground_collision_manager')
    
    try:
        manager = GroundCollisionManager()
        rospy.loginfo("Ground collision manager started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()