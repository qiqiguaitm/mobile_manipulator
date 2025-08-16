#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Verify collision detection configuration
Check if robot can avoid self-collision and ground collision
"""

import rospy
import moveit_commander
import sys
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents

class CollisionVerifier:
    def __init__(self):
        rospy.init_node('collision_verifier')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Service to get planning scene
        rospy.wait_for_service('/get_planning_scene')
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        
        rospy.sleep(2)
    
    def check_critical_collision_pairs(self):
        """Check if critical collision pairs are active"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Collision Detection Status Check")
        rospy.loginfo("=" * 60)
        
        # Get planning scene
        try:
            req = PlanningSceneComponents()
            req.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            resp = self.get_scene(req)
            acm = resp.scene.allowed_collision_matrix
            
            rospy.loginfo("\nAllowed Collision Matrix size: %d x %d", 
                         len(acm.entry_names), len(acm.entry_names))
            
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
            
            rospy.loginfo("\n🔍 Checking critical collision pairs:")
            active_count = 0
            disabled_count = 0
            
            for link1, link2 in critical_pairs:
                # Check if collision is allowed (disabled)
                is_disabled = self.check_collision_disabled(acm, link1, link2)
                
                if is_disabled:
                    rospy.logwarn("  ❌ %s <-> %s: Collision DISABLED (危险！)", link1, link2)
                    disabled_count += 1
                else:
                    rospy.loginfo("  ✅ %s <-> %s: Collision ACTIVE (正确)", link1, link2)
                    active_count += 1
            
            rospy.loginfo("\n📊 Summary:")
            rospy.loginfo("  Active collision pairs: %d", active_count)
            rospy.loginfo("  Disabled collision pairs: %d", disabled_count)
            
            if disabled_count > 0:
                rospy.logerr("\n⚠️  警告：关键碰撞对被禁用！机械臂可能撞到机身！")
                rospy.loginfo("\n💡 解决方案：")
                rospy.loginfo("  1. 修改SRDF文件，删除这些碰撞对的disable_collisions条目")
                rospy.loginfo("  2. 或者使用专门的碰撞padding配置")
            
        except Exception as e:
            rospy.logerr("Failed to get planning scene: %s", str(e))
    
    def check_collision_disabled(self, acm, link1, link2):
        """Check if collision between two links is disabled in ACM"""
        try:
            idx1 = acm.entry_names.index(link1)
            idx2 = acm.entry_names.index(link2)
            
            # ACM is symmetric, check both directions
            if idx1 < len(acm.entry_values) and idx2 < len(acm.entry_values[idx1].enabled):
                return not acm.entry_values[idx1].enabled[idx2]
            
        except (ValueError, IndexError):
            pass
        
        return False
    
    def check_ground_collision(self):
        """Check if ground collision object exists"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Ground Collision Check")
        rospy.loginfo("=" * 60)
        
        objects = self.scene.get_known_object_names()
        
        if "ground" in objects:
            rospy.loginfo("✅ Ground collision object found in scene")
        else:
            rospy.logwarn("❌ No ground collision object in scene")
            rospy.loginfo("\n💡 To add ground collision:")
            rospy.loginfo("  roslaunch arm_planner demo_with_ground.launch")
            rospy.loginfo("  或运行: rosrun arm_planner ground_collision_manager.py")
    
    def test_dangerous_poses(self):
        """Test poses that might cause collision"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Testing Dangerous Poses")
        rospy.loginfo("=" * 60)
        
        test_poses = [
            {
                "name": "机械臂向后折叠（可能撞lifting_Link）",
                "joints": [0, 2.5, -2.5, 0, 0, 0, 0, 0]
            },
            {
                "name": "机械臂向下（可能撞地面或box_Link）",
                "joints": [0, 1.57, 1.57, 0, 1.57, 0, 0, 0]
            },
            {
                "name": "机械臂侧摆（可能撞lidar）",
                "joints": [-2.5, 0.5, 0, 0, 0, 0, 0, 0]
            }
        ]
        
        for test in test_poses:
            rospy.loginfo("\n测试: %s", test["name"])
            self.move_group.set_joint_value_target(test["joints"])
            
            plan = self.move_group.plan()
            if plan[0]:
                rospy.logwarn("  ⚠️  规划成功 - 可能存在碰撞风险！")
            else:
                rospy.loginfo("  ✅ 规划失败 - 碰撞检测生效")
            
            self.move_group.clear_pose_targets()
    
    def provide_fix_instructions(self):
        """Provide instructions to fix collision detection"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("修复建议")
        rospy.loginfo("=" * 60)
        
        rospy.loginfo("\n如果碰撞检测不工作，请按以下步骤修复：")
        rospy.loginfo("\n1. 确保关键碰撞对未被禁用")
        rospy.loginfo("   检查SRDF中这些碰撞对没有disable_collisions条目：")
        rospy.loginfo("   - link1/2/3 <-> lifting_Link")
        rospy.loginfo("   - link1/2/3 <-> box_Link")
        rospy.loginfo("   - link1/2 <-> lidar_Link")
        rospy.loginfo("   - link1/2/3 <-> under_camera_Link")
        
        rospy.loginfo("\n2. 启用地面碰撞保护")
        rospy.loginfo("   rosrun arm_planner ground_collision_manager.py")
        
        rospy.loginfo("\n3. 检查碰撞padding配置")
        rospy.loginfo("   查看: collision_padding.yaml")
        
        rospy.loginfo("\n4. 重启所有节点以加载新配置")

def main():
    verifier = CollisionVerifier()
    
    try:
        verifier.check_critical_collision_pairs()
        verifier.check_ground_collision()
        verifier.test_dangerous_poses()
        verifier.provide_fix_instructions()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()