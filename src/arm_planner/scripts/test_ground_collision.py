#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test script for ground collision protection
Verifies that the robot arm cannot plan paths that would hit the ground
"""

import rospy
import moveit_commander
import sys
from std_srvs.srv import SetBool

class GroundCollisionTester:
    def __init__(self):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Wait for ground collision service
        rospy.loginfo("Waiting for ground collision service...")
        rospy.wait_for_service('/ground_collision_manager/enable_ground_collision')
        self.ground_service = rospy.ServiceProxy('/ground_collision_manager/enable_ground_collision', SetBool)
        
        rospy.sleep(2)  # Wait for scene to update
    
    def test_ground_collision(self):
        """Run ground collision tests"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("Ground Collision Protection Test")
        rospy.loginfo("=" * 50)
        
        # Check if ground exists
        objects = self.scene.get_known_object_names()
        if "ground" in objects:
            rospy.loginfo("✓ Ground object detected in planning scene")
        else:
            rospy.logwarn("✗ Ground object not found, enabling...")
            self.ground_service(True)
            rospy.sleep(1)
        
        # Get current pose
        current_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("Current end-effector Z position: %.3fm", current_pose.position.z)
        
        # Test cases
        test_cases = [
            {"name": "Safe height (30cm)", "z": 0.3, "should_succeed": True},
            {"name": "Boundary height (10cm)", "z": 0.1, "should_succeed": True},
            {"name": "Near ground (5cm)", "z": 0.05, "should_succeed": False},
            {"name": "Below ground (-5cm)", "z": -0.05, "should_succeed": False},
        ]
        
        passed = 0
        failed = 0
        
        for test in test_cases:
            rospy.loginfo("\nTest: %s", test["name"])
            
            # Set target pose
            target_pose = current_pose
            target_pose.position.z = test["z"]
            
            self.move_group.set_pose_target(target_pose)
            
            # Plan (but don't execute)
            plan = self.move_group.plan()
            success = plan[0]
            
            # Check result
            if success == test["should_succeed"]:
                rospy.loginfo("  ✓ PASS - Planning %s as expected", 
                            "succeeded" if success else "failed")
                passed += 1
            else:
                rospy.logerr("  ✗ FAIL - Expected %s but got %s",
                           "success" if test["should_succeed"] else "failure",
                           "success" if success else "failure")
                failed += 1
            
            self.move_group.clear_pose_targets()
            rospy.sleep(0.5)
        
        # Summary
        rospy.loginfo("\n" + "=" * 50)
        rospy.loginfo("Test Summary: %d passed, %d failed", passed, failed)
        if failed == 0:
            rospy.loginfo("✓ All tests passed! Ground collision protection is working.")
        else:
            rospy.logerr("✗ Some tests failed! Check ground collision setup.")
        
        return failed == 0
    
    def cleanup(self):
        """Clean up MoveIt"""
        moveit_commander.roscpp_shutdown()

def main():
    rospy.init_node('test_ground_collision')
    
    tester = GroundCollisionTester()
    try:
        success = tester.test_ground_collision()
        return 0 if success else 1
    except rospy.ROSInterruptException:
        return 1
    finally:
        tester.cleanup()

if __name__ == '__main__':
    sys.exit(main())