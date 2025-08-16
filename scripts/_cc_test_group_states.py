#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def test_group_states():
    rospy.init_node('test_group_states')
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("arm")
    
    rospy.sleep(1)
    
    # Get available named targets
    rospy.loginfo("=" * 50)
    rospy.loginfo("Testing Group States")
    rospy.loginfo("=" * 50)
    
    # Get robot model info
    rospy.loginfo("Robot name: %s", robot.get_robot_name())
    rospy.loginfo("Planning frame: %s", robot.get_planning_frame())
    rospy.loginfo("End effector link: %s", move_group.get_end_effector_link())
    
    # Get named targets
    named_targets = move_group.get_named_targets()
    rospy.loginfo("\nAvailable named targets (group states):")
    for target in named_targets:
        rospy.loginfo("  - %s", target)
    
    # Try to get the values for each named target
    rospy.loginfo("\nNamed target values:")
    for target in named_targets:
        try:
            move_group.set_named_target(target)
            values = move_group.get_joint_value_target()
            rospy.loginfo("\n%s:", target)
            for joint, value in values.items():
                rospy.loginfo("  %s: %.3f", joint, value)
        except Exception as e:
            rospy.logerr("Failed to get values for %s: %s", target, str(e))
    
    # Check if zero state exists
    if "zero" in named_targets:
        rospy.loginfo("\n✓ 'zero' state is available in MoveIt")
    else:
        rospy.logerr("\n✗ 'zero' state is NOT available in MoveIt!")
        rospy.loginfo("This might be why it's not showing in RViz")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_group_states()
    except rospy.ROSInterruptException:
        pass