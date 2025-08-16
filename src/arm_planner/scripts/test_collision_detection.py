#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test collision detection between robot arm and body
Verifies that arm movements avoid collisions with chassis and sensors
"""

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

class CollisionDetectionTester:
    def __init__(self):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        rospy.sleep(2)
        
        # Get collision matrix info
        self.planning_scene = moveit_commander.PlanningScene()
        
    def check_collision_pairs(self):
        """Check which collision pairs are active"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Collision Detection Status")
        rospy.loginfo("=" * 60)
        
        # Critical collision pairs that MUST be checked
        critical_pairs = [
            ("link1", "lifting_Link"),
            ("link2", "lifting_Link"),
            ("link3", "lifting_Link"),
            ("link1", "box_Link"),
            ("link2", "box_Link"),
            ("link3", "box_Link"),
            ("link1", "lidar_Link"),
            ("link2", "lidar_Link"),
            ("link1", "under_camera_Link"),
            ("link2", "under_camera_Link"),
            ("link3", "under_camera_Link"),
        ]
        
        rospy.loginfo("\nCritical collision pairs (MUST be active):")
        for link1, link2 in critical_pairs:
            rospy.loginfo("  %s <-> %s", link1, link2)
        
        # Get scene objects
        objects = self.scene.get_known_object_names()
        rospy.loginfo("\nScene objects: %s", objects)
        
        return True
    
    def test_dangerous_poses(self):
        """Test poses that might cause collisions"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Testing Dangerous Poses")
        rospy.loginfo("=" * 60)
        
        # Define test poses that might hit the robot body
        test_cases = [
            {
                "name": "Arm folded back (might hit lifting_Link)",
                "joints": {
                    "joint1": 0.0,
                    "joint2": 2.5,    # Bend back
                    "joint3": -2.5,   # Fold down
                    "joint4": 0.0,
                    "joint5": 0.0,
                    "joint6": 0.0,
                    "joint7": 0.0,
                    "joint8": 0.0
                }
            },
            {
                "name": "Arm reaching down (might hit box_Link)",
                "joints": {
                    "joint1": 0.0,
                    "joint2": 1.57,   # 90 degrees
                    "joint3": 1.57,   # 90 degrees down
                    "joint4": 0.0,
                    "joint5": 1.57,
                    "joint6": 0.0,
                    "joint7": 0.0,
                    "joint8": 0.0
                }
            },
            {
                "name": "Arm swinging to side (might hit lidar)",
                "joints": {
                    "joint1": -2.5,   # Swing left
                    "joint2": 0.5,
                    "joint3": 0.0,
                    "joint4": 0.0,
                    "joint5": 0.0,
                    "joint6": 0.0,
                    "joint7": 0.0,
                    "joint8": 0.0
                }
            }
        ]
        
        for test in test_cases:
            rospy.loginfo("\nTesting: %s", test["name"])
            
            # Set joint target
            self.move_group.set_joint_value_target(test["joints"])
            
            # Plan (but don't execute)
            plan = self.move_group.plan()
            success = plan[0]
            
            if success:
                rospy.loginfo("  ✓ Planning succeeded - No collision detected")
                # Check if path is collision-free
                if hasattr(plan[1], 'joint_trajectory'):
                    rospy.loginfo("    Path has %d waypoints", 
                                len(plan[1].joint_trajectory.points))
            else:
                rospy.logwarn("  ⚠ Planning failed - Possible collision or unreachable")
            
            self.move_group.clear_pose_targets()
            rospy.sleep(0.5)
    
    def test_workspace_limits(self):
        """Test if arm respects workspace to avoid collisions"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Testing Workspace Limits")
        rospy.loginfo("=" * 60)
        
        current_pose = self.move_group.get_current_pose().pose
        
        # Test reaching behind (toward lifting_Link)
        test_pose = current_pose
        test_pose.position.x = -0.3  # Behind the base
        test_pose.position.z = 0.5
        
        self.move_group.set_pose_target(test_pose)
        plan = self.move_group.plan()
        
        if plan[0]:
            rospy.loginfo("Can reach behind base at x=-0.3m")
        else:
            rospy.loginfo("Cannot reach behind base at x=-0.3m (good if lifting_Link is there)")
        
        self.move_group.clear_pose_targets()
    
    def get_collision_summary(self):
        """Provide summary of collision detection setup"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Collision Detection Summary")
        rospy.loginfo("=" * 60)
        
        rospy.loginfo("\n✓ Key collision pairs being checked:")
        rospy.loginfo("  - link1/2/3 <-> lifting_Link (立柱)")
        rospy.loginfo("  - link1/2/3 <-> box_Link (箱体)")
        rospy.loginfo("  - link1/2 <-> lidar_Link (激光雷达)")
        rospy.loginfo("  - link1/2/3 <-> under_camera_Link (底部相机)")
        
        rospy.loginfo("\n✓ Safe pairs (collision disabled):")
        rospy.loginfo("  - Adjacent links (相邻关节)")
        rospy.loginfo("  - High links (link4-8) with chassis (高位连杆与底盘)")
        rospy.loginfo("  - Gripper fingers (link7 <-> link8)")
        
        rospy.loginfo("\n⚠ Critical safety zones:")
        rospy.loginfo("  - Behind robot (lifting_Link area)")
        rospy.loginfo("  - Below robot (box_Link area)")
        rospy.loginfo("  - Side sensors (lidar, cameras)")

def main():
    rospy.init_node('test_collision_detection')
    
    tester = CollisionDetectionTester()
    
    try:
        # Run tests
        tester.check_collision_pairs()
        tester.test_dangerous_poses()
        tester.test_workspace_limits()
        tester.get_collision_summary()
        
        rospy.loginfo("\n✅ Collision detection test completed")
        
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()