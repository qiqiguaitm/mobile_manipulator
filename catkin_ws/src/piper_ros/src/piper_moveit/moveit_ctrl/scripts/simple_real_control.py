#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化版实机控制脚本
提供基本的实机控制功能：
- 获取当前状态
- 移动到指定关节角度
- 移动到指定笛卡尔位置
- 控制夹爪
"""

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from piper_msgs.srv import Enable
import sys
import numpy as np

class SimpleRealControl:
    def __init__(self):
        # 初始化
        rospy.init_node('simple_real_control', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        
        # MoveIt组件
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # 设置规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.2)  # 降低速度
        self.arm_group.set_max_acceleration_scaling_factor(0.2)
        self.arm_group.set_planning_time(5.0)
        
        self.current_joint_states = None
        
        # 订阅关节状态
        self.joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_callback
        )
        
        rospy.loginfo("等待系统初始化...")
        rospy.sleep(2.0)
        
    def joint_callback(self, msg):
        self.current_joint_states = msg
        
    def enable_robot(self):
        """使能机器人"""
        try:
            rospy.wait_for_service('/enable_srv', timeout=5.0)
            enable_srv = rospy.ServiceProxy('/enable_srv', Enable)
            resp = enable_srv(enable_request=True)
            return resp.enable_response
        except:
            return False
            
    def get_current_joint_positions(self):
        """获取当前关节位置"""
        if self.current_joint_states:
            return list(self.current_joint_states.position[:6])
        return None
        
    def get_current_pose(self):
        """获取当前末端位姿"""
        return self.arm_group.get_current_pose().pose
        
    def move_to_joint_positions(self, joint_positions, wait=True):
        """移动到指定关节位置
        joint_positions: 6个关节的目标位置(弧度)
        """
        if len(joint_positions) != 6:
            rospy.logerr("需要6个关节位置值")
            return False
            
        self.arm_group.set_joint_value_target(joint_positions)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        return success
        
    def move_to_pose(self, x, y, z, rx=None, ry=None, rz=None, rw=None, wait=True):
        """移动到指定位姿
        x, y, z: 目标位置(米)
        rx, ry, rz, rw: 目标姿态四元数(可选，默认保持当前姿态)
        """
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        if rx is None:
            # 保持当前姿态
            current_pose = self.get_current_pose()
            pose_goal.orientation = current_pose.orientation
        else:
            pose_goal.orientation.x = rx
            pose_goal.orientation.y = ry
            pose_goal.orientation.z = rz
            pose_goal.orientation.w = rw
            
        self.arm_group.set_pose_target(pose_goal)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        return success
        
    def control_gripper(self, position, wait=True):
        """控制夹爪
        position: 0.0(闭合) - 0.025(打开)
        """
        gripper_goal = [position, -position]
        self.gripper_group.set_joint_value_target(gripper_goal)
        success = self.gripper_group.go(wait=wait)
        self.gripper_group.stop()
        
        return success
        
    def move_to_home(self):
        """回到零位"""
        return self.move_to_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
    def print_current_state(self):
        """打印当前状态"""
        joints = self.get_current_joint_positions()
        pose = self.get_current_pose()
        
        if joints:
            print("\n当前关节位置(弧度):")
            for i, j in enumerate(joints):
                print(f"  关节{i+1}: {j:.4f} ({np.degrees(j):.2f}°)")
                
        if pose:
            print("\n当前末端位置:")
            print(f"  X: {pose.position.x:.4f} m")
            print(f"  Y: {pose.position.y:.4f} m")
            print(f"  Z: {pose.position.z:.4f} m")
            

def main():
    # 创建控制对象
    ctrl = SimpleRealControl()
    
    # 使能机器人
    print("正在使能机器人...")
    if not ctrl.enable_robot():
        print("机器人使能失败！")
        return
    print("机器人使能成功！")
    
    # 等待稳定
    rospy.sleep(2.0)
    
    # 显示当前状态
    ctrl.print_current_state()
    
    # 演示控制
    while not rospy.is_shutdown():
        print("\n========== 实机控制菜单 ==========")
        print("1. 显示当前状态")
        print("2. 移动到指定关节角度")
        print("3. 移动到指定笛卡尔位置")
        print("4. 控制夹爪")
        print("5. 回到零位")
        print("6. 预设动作演示")
        print("0. 退出")
        print("=================================")
        
        try:
            choice = input("请选择操作: ")
            
            if choice == '1':
                ctrl.print_current_state()
                
            elif choice == '2':
                print("请输入6个关节角度(度)，用空格分隔:")
                angles_str = input()
                angles_deg = [float(a) for a in angles_str.split()]
                angles_rad = [np.radians(a) for a in angles_deg]
                
                print(f"移动到: {angles_deg}")
                if ctrl.move_to_joint_positions(angles_rad):
                    print("移动成功!")
                else:
                    print("移动失败!")
                    
            elif choice == '3':
                x = float(input("X坐标(米): "))
                y = float(input("Y坐标(米): "))
                z = float(input("Z坐标(米): "))
                
                print(f"移动到: ({x}, {y}, {z})")
                if ctrl.move_to_pose(x, y, z):
                    print("移动成功!")
                else:
                    print("移动失败!")
                    
            elif choice == '4':
                pos = float(input("夹爪位置(0-0.025): "))
                if ctrl.control_gripper(pos):
                    print("夹爪控制成功!")
                else:
                    print("夹爪控制失败!")
                    
            elif choice == '5':
                print("回到零位...")
                if ctrl.move_to_home():
                    print("回零成功!")
                else:
                    print("回零失败!")
                    
            elif choice == '6':
                print("\n执行预设动作序列...")
                
                # 动作1: 准备位置
                print("1. 移动到准备位置...")
                ctrl.move_to_joint_positions([0, -30, 60, 0, 30, 0])
                rospy.sleep(1.0)
                
                # 动作2: 打开夹爪
                print("2. 打开夹爪...")
                ctrl.control_gripper(0.025)
                rospy.sleep(1.0)
                
                # 动作3: 移动到抓取位置
                print("3. 移动到抓取位置...")
                ctrl.move_to_pose(0.25, 0.1, 0.2)
                rospy.sleep(1.0)
                
                # 动作4: 关闭夹爪
                print("4. 关闭夹爪...")
                ctrl.control_gripper(0.005)
                rospy.sleep(1.0)
                
                # 动作5: 抬起
                print("5. 抬起物体...")
                current_pose = ctrl.get_current_pose()
                ctrl.move_to_pose(
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z + 0.1
                )
                rospy.sleep(1.0)
                
                # 动作6: 回到零位
                print("6. 回到零位...")
                ctrl.move_to_home()
                
                print("预设动作完成!")
                
            elif choice == '0':
                print("退出程序")
                break
                
            else:
                print("无效的选择!")
                
        except Exception as e:
            print(f"操作出错: {e}")
            
    # 关闭
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()