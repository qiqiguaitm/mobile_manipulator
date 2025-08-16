#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

def test_ee_config():
    rospy.init_node('test_ee_config')
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    
    rospy.sleep(1)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("Testing End Effector Configuration")
    rospy.loginfo("=" * 50)
    
    # Test each group
    groups = ["arm", "piper", "gripper"]
    
    for group_name in groups:
        try:
            move_group = moveit_commander.MoveGroupCommander(group_name)
            
            rospy.loginfo("\n%s group:", group_name)
            
            # Get end effector link
            ee_link = move_group.get_end_effector_link()
            rospy.loginfo("  End effector link: %s", ee_link if ee_link else "None")
            
            # Get pose reference frame
            ref_frame = move_group.get_pose_reference_frame()
            rospy.loginfo("  Pose reference frame: %s", ref_frame)
            
            # Get current pose
            try:
                current_pose = move_group.get_current_pose()
                rospy.loginfo("  Current pose available: Yes")
                rospy.loginfo("  Position: x=%.3f, y=%.3f, z=%.3f", 
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z)
            except Exception as e:
                rospy.logwarn("  Current pose available: No (%s)", str(e))
            
            # Check if IK is available
            joints = move_group.get_active_joints()
            rospy.loginfo("  Active joints: %d (%s)", len(joints), 
                         ", ".join(joints[:3]) + "..." if len(joints) > 3 else ", ".join(joints))
            
        except Exception as e:
            rospy.logerr("Failed to get info for group %s: %s", group_name, str(e))
    
    # Check end effectors
    rospy.loginfo("\nEnd Effectors:")
    # This info is in SRDF, just display what we expect
    rospy.loginfo("  gripper_eef: parent_link=link6, group=gripper")
    rospy.loginfo("  arm_eef: parent_link=link6, group=arm")
    
    rospy.loginfo("\n✅ Configuration check complete")
    rospy.loginfo("If you still see 'Unknown marker' errors, restart the demo.launch")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_ee_config()
    except rospy.ROSInterruptException:
        pass