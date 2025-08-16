#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def verify_groups():
    rospy.init_node('verify_groups')
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    
    rospy.sleep(1)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("Verifying Group Configuration")
    rospy.loginfo("=" * 50)
    
    # Get all group names
    group_names = robot.get_group_names()
    rospy.loginfo("\nAvailable groups:")
    for group in group_names:
        rospy.loginfo("  - %s", group)
    
    # Check each group
    for group_name in group_names:
        try:
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joints = move_group.get_active_joints()
            rospy.loginfo("\n%s group joints:", group_name)
            for joint in joints:
                rospy.loginfo("  - %s", joint)
            
            # Get named targets
            targets = move_group.get_named_targets()
            if targets:
                rospy.loginfo("%s group states:", group_name)
                for target in targets:
                    rospy.loginfo("  - %s", target)
        except Exception as e:
            rospy.logerr("Failed to get info for group %s: %s", group_name, str(e))
    
    # Verify expected groups
    expected_groups = ["arm", "gripper", "piper", "chassis", "sensors"]
    rospy.loginfo("\nVerification:")
    for expected in expected_groups:
        if expected in group_names:
            rospy.loginfo("  ✓ %s group exists", expected)
        else:
            rospy.logerr("  ✗ %s group missing", expected)
    
    # Check arm vs piper
    if "arm" in group_names and "piper" in group_names:
        arm_group = moveit_commander.MoveGroupCommander("arm")
        piper_group = moveit_commander.MoveGroupCommander("piper")
        
        arm_joints = arm_group.get_active_joints()
        piper_joints = piper_group.get_active_joints()
        
        rospy.loginfo("\nGroup comparison:")
        rospy.loginfo("  arm group: %d joints (j1-j8)", len(arm_joints))
        rospy.loginfo("  piper group: %d joints (j1-j6)", len(piper_joints))
        rospy.loginfo("  gripper joints: j7, j8")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        verify_groups()
    except rospy.ROSInterruptException:
        pass