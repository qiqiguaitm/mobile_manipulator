#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

class ArmControllerWithGripper:
    """增强的机械臂控制器，支持独立的夹爪控制"""
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_controller_with_gripper', anonymous=True)
        
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        
        rospy.loginfo("ArmControllerWithGripper initialized")
    
    def open_gripper(self):
        """打开夹爪，保持其他关节不变"""
        current_joint_values = self.arm_group.get_current_joint_values()
        target_joint_values = list(current_joint_values)
        target_joint_values[6] = 0.035  # joint7
        target_joint_values[7] = 0.035  # joint8
        
        self.arm_group.set_joint_value_target(target_joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("Gripper opened successfully")
        else:
            rospy.logerr("Failed to open gripper")
        
        return success
    
    def close_gripper(self):
        """关闭夹爪，保持其他关节不变"""
        current_joint_values = self.arm_group.get_current_joint_values()
        target_joint_values = list(current_joint_values)
        target_joint_values[6] = 0.0  # joint7
        target_joint_values[7] = 0.0  # joint8
        
        self.arm_group.set_joint_value_target(target_joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("Gripper closed successfully")
        else:
            rospy.logerr("Failed to close gripper")
        
        return success
    
    def set_gripper_position(self, position):
        """设置夹爪位置（0.0 = 完全关闭, 0.035 = 完全打开）"""
        position = max(0.0, min(0.035, position))  # 限制范围
        
        current_joint_values = self.arm_group.get_current_joint_values()
        target_joint_values = list(current_joint_values)
        target_joint_values[6] = position  # joint7
        target_joint_values[7] = position  # joint8
        
        self.arm_group.set_joint_value_target(target_joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo(f"Gripper position set to {position}")
        else:
            rospy.logerr(f"Failed to set gripper position to {position}")
        
        return success
    
    def go_to_named_target(self, target_name):
        """移动到预定义的目标位置"""
        try:
            self.arm_group.set_named_target(target_name)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            if success:
                rospy.loginfo(f"Moved to {target_name} successfully")
            else:
                rospy.logerr(f"Failed to move to {target_name}")
            
            return success
        except Exception as e:
            rospy.logerr(f"Error moving to {target_name}: {e}")
            return False
    
    def go_to_pose(self, pose):
        """移动到指定的位姿"""
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("Moved to pose successfully")
        else:
            rospy.logerr("Failed to move to pose")
        
        return success
    
    def go_to_joint_values(self, joint_values):
        """移动到指定的关节角度"""
        self.arm_group.set_joint_value_target(joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            rospy.loginfo("Moved to joint values successfully")
        else:
            rospy.logerr("Failed to move to joint values")
        
        return success
    
    def get_current_joint_values(self):
        """获取当前关节角度"""
        return self.arm_group.get_current_joint_values()
    
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        return self.arm_group.get_current_pose().pose
    
    def shutdown(self):
        """关闭控制器"""
        moveit_commander.roscpp_shutdown()


def test_gripper_control():
    """测试夹爪控制功能"""
    controller = ArmControllerWithGripper()
    
    try:
        print("\n=== 测试夹爪控制 ===")
        
        # 先移动到一个非零位置
        print("\n1. 移动到ready位置...")
        controller.go_to_named_target("ready")
        rospy.sleep(1)
        
        # 测试夹爪控制
        print("\n2. 打开夹爪（保持机械臂位置）...")
        controller.open_gripper()
        rospy.sleep(1)
        
        print("\n3. 关闭夹爪（保持机械臂位置）...")
        controller.close_gripper()
        rospy.sleep(1)
        
        print("\n4. 设置夹爪到半开位置...")
        controller.set_gripper_position(0.0175)  # 半开
        rospy.sleep(1)
        
        print("\n5. 完全打开夹爪...")
        controller.open_gripper()
        
        print("\n✅ 测试完成！")
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        controller.shutdown()


if __name__ == "__main__":
    test_gripper_control()