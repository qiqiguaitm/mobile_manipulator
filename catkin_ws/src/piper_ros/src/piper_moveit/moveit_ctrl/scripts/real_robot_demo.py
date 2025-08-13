#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实机控制演示程序
基于demo_simple.launch实现：
1. 获取实机的初始状态
2. 进行运动规划
3. 控制实机到达目标点
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import sys
import time
from std_msgs.msg import Bool
from piper_msgs.srv import Enable
import tf.transformations as tf_trans

class RealRobotDemo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('real_robot_demo', anonymous=True)
        
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建RobotCommander对象
        self.robot = moveit_commander.RobotCommander()
        
        # 创建PlanningSceneInterface对象
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建MoveGroupCommander对象
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # 设置规划参数
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(10.0)
        self.arm_group.allow_replanning(True)
        self.arm_group.set_num_planning_attempts(10)
        
        # 设置速度和加速度缩放因子
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        
        # 当前关节状态
        self.current_joint_states = None
        
        # 订阅关节状态
        self.joint_states_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_states_callback
        )
        
        # 等待关节状态数据
        rospy.loginfo("等待关节状态数据...")
        while self.current_joint_states is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("成功接收到关节状态数据")
        
    def joint_states_callback(self, msg):
        """关节状态回调函数"""
        self.current_joint_states = msg
        
    def enable_robot(self):
        """使能机器人"""
        try:
            rospy.loginfo("正在使能机器人...")
            rospy.wait_for_service('/enable_srv', timeout=5.0)
            enable_srv = rospy.ServiceProxy('/enable_srv', Enable)
            resp = enable_srv(enable_request=True)
            if resp.enable_response:
                rospy.loginfo("机器人使能成功")
                return True
            else:
                rospy.logerr("机器人使能失败")
                return False
        except Exception as e:
            rospy.logerr(f"使能服务调用失败: {e}")
            return False
            
    def get_current_state(self):
        """获取机器人当前状态"""
        rospy.loginfo("=== 获取机器人当前状态 ===")
        
        # 获取当前关节值
        if self.current_joint_states:
            joint_names = self.current_joint_states.name[:6]  # 前6个是arm关节
            joint_positions = self.current_joint_states.position[:6]
            
            rospy.loginfo("当前关节状态:")
            for name, pos in zip(joint_names, joint_positions):
                rospy.loginfo(f"  {name}: {pos:.4f} rad ({pos*180/3.14159:.2f} deg)")
        
        # 获取当前末端位姿
        current_pose = self.arm_group.get_current_pose().pose
        rospy.loginfo(f"当前末端位置:")
        rospy.loginfo(f"  x: {current_pose.position.x:.4f} m")
        rospy.loginfo(f"  y: {current_pose.position.y:.4f} m")
        rospy.loginfo(f"  z: {current_pose.position.z:.4f} m")
        
        # 转换四元数到欧拉角
        euler = tf_trans.euler_from_quaternion([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ])
        rospy.loginfo(f"当前末端姿态 (欧拉角):")
        rospy.loginfo(f"  roll:  {euler[0]*180/3.14159:.2f} deg")
        rospy.loginfo(f"  pitch: {euler[1]*180/3.14159:.2f} deg")
        rospy.loginfo(f"  yaw:   {euler[2]*180/3.14159:.2f} deg")
        
        return current_pose
        
    def plan_to_joint_goal(self, joint_goal):
        """规划到目标关节角度"""
        rospy.loginfo("=== 规划到目标关节角度 ===")
        rospy.loginfo(f"目标关节角度: {[f'{j:.4f}' for j in joint_goal]}")
        
        # 设置目标关节值
        self.arm_group.set_joint_value_target(joint_goal)
        
        # 进行规划
        rospy.loginfo("开始规划...")
        plan = self.arm_group.plan()
        
        if plan[0]:
            rospy.loginfo("规划成功!")
            return plan[1]
        else:
            rospy.logerr("规划失败!")
            return None
            
    def plan_to_pose_goal(self, pose_goal):
        """规划到目标位姿"""
        rospy.loginfo("=== 规划到目标位姿 ===")
        rospy.loginfo(f"目标位置: x={pose_goal.position.x:.4f}, y={pose_goal.position.y:.4f}, z={pose_goal.position.z:.4f}")
        
        # 设置目标位姿
        self.arm_group.set_pose_target(pose_goal)
        
        # 进行规划
        rospy.loginfo("开始规划...")
        plan = self.arm_group.plan()
        
        if plan[0]:
            rospy.loginfo("规划成功!")
            return plan[1]
        else:
            rospy.logerr("规划失败!")
            return None
            
    def execute_plan(self, plan):
        """执行规划的轨迹"""
        rospy.loginfo("=== 执行规划轨迹 ===")
        
        if plan is None:
            rospy.logerr("没有可执行的规划")
            return False
            
        rospy.loginfo("开始执行...")
        success = self.arm_group.execute(plan, wait=True)
        
        if success:
            rospy.loginfo("执行成功!")
        else:
            rospy.logerr("执行失败!")
            
        # 停止机械臂运动
        self.arm_group.stop()
        
        # 清除目标
        self.arm_group.clear_pose_targets()
        
        return success
        
    def control_gripper(self, position):
        """控制夹爪
        position: 0.0 (完全闭合) 到 0.025 (完全打开)
        """
        rospy.loginfo(f"=== 控制夹爪到位置: {position} ===")
        
        # 设置夹爪目标 (joint7和joint8相同)
        gripper_goal = [position, -position]
        self.gripper_group.set_joint_value_target(gripper_goal)
        
        # 执行
        success = self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        
        if success:
            rospy.loginfo("夹爪控制成功!")
        else:
            rospy.logerr("夹爪控制失败!")
            
        return success
        
    def demo_sequence(self):
        """演示序列"""
        rospy.loginfo("\n========== 开始实机控制演示 ==========\n")
        
        # 1. 获取并显示当前状态
        current_pose = self.get_current_state()
        rospy.sleep(1.0)
        
        # 2. 移动到预定义的关节位置
        rospy.loginfo("\n--- 演示1: 移动到预定义关节位置 ---")
        # 定义一个安全的目标关节角度 (弧度)
        joint_goal = [0.0, -0.5, 0.8, 0.0, 0.5, 0.0]  # 示例位置
        
        plan = self.plan_to_joint_goal(joint_goal)
        if plan:
            input("按Enter键执行规划的轨迹...")
            self.execute_plan(plan)
            rospy.sleep(2.0)
        
        # 3. 移动到笛卡尔空间目标位置
        rospy.loginfo("\n--- 演示2: 移动到笛卡尔空间目标 ---")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.3
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.3
        # 保持当前姿态
        pose_goal.orientation = current_pose.orientation
        
        plan = self.plan_to_pose_goal(pose_goal)
        if plan:
            input("按Enter键执行规划的轨迹...")
            self.execute_plan(plan)
            rospy.sleep(2.0)
            
        # 4. 控制夹爪
        rospy.loginfo("\n--- 演示3: 控制夹爪 ---")
        input("按Enter键打开夹爪...")
        self.control_gripper(0.025)  # 打开
        rospy.sleep(1.0)
        
        input("按Enter键关闭夹爪...")
        self.control_gripper(0.005)  # 关闭
        rospy.sleep(1.0)
        
        # 5. 回到初始位置
        rospy.loginfo("\n--- 演示4: 回到零位 ---")
        input("按Enter键回到零位...")
        zero_joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        plan = self.plan_to_joint_goal(zero_joint_goal)
        if plan:
            self.execute_plan(plan)
            
        rospy.loginfo("\n========== 演示完成 ==========\n")

def main():
    try:
        # 创建演示对象
        demo = RealRobotDemo()
        
        # 使能机器人
        if not demo.enable_robot():
            rospy.logerr("机器人使能失败，退出程序")
            return
            
        # 等待系统稳定
        rospy.sleep(2.0)
        
        # 执行演示序列
        demo.demo_sequence()
        
        # 关闭MoveIt
        moveit_commander.roscpp_shutdown()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        
if __name__ == '__main__':
    main()