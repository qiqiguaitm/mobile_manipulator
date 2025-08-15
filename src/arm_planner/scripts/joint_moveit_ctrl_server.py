#!/usr/bin/env python

import rospy
import threading
from moveit_commander import *
from arm_planner.srv import JointMoveitCtrl, JointMoveitCtrlResponse
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class JointMoveitCtrlServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_moveit_ctrl_server')

        # 初始化标志
        self._initialized = False
        self._initializing = False
        self._init_lock = threading.Lock()
        
        # 仅实例化存在的规划组
        self.arm_move_group = None
        self.gripper_move_group = None
        self.piper_move_group = None
        self.robot = None
        self.available_groups = []

        # 创建关节运动控制服务（立即提供服务，但在第一次调用时才初始化MoveIt）
        self.arm_srv = rospy.Service('joint_moveit_ctrl_arm', JointMoveitCtrl, self.handle_joint_moveit_ctrl_arm)
        self.gripper_srv = rospy.Service('joint_moveit_ctrl_gripper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_gripper)
        self.piper_srv = rospy.Service('joint_moveit_ctrl_piper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_piper)
        self.endpose_srv = rospy.Service('joint_moveit_ctrl_endpose', JointMoveitCtrl, self.handle_joint_moveit_ctrl_endpose)

        rospy.loginfo("Joint MoveIt Control Services Ready (optimized initialization mode).")
        
        # 立即在后台启动快速初始化
        init_thread = threading.Thread(target=self._fast_init_moveit)
        init_thread.daemon = True
        init_thread.start()
    
    def _fast_init_moveit(self):
        """快速初始化MoveIt组件"""
        # 等待move_group节点启动并发布服务
        self._wait_for_move_group_services()
        self._init_moveit()
    
    def _init_moveit(self):
        """优化的MoveIt初始化流程"""
        with self._init_lock:
            if self._initialized or self._initializing:
                return
            
            self._initializing = True
            
        rospy.loginfo("🚀 开始快速初始化MoveIt组件...")
        start_time = rospy.Time.now()
        
        try:
            # 初始化 MoveIt
            roscpp_initialize([])
            self.robot = RobotCommander()
            
            # 获取 MoveIt 规划组列表
            self.available_groups = self.robot.get_group_names()
            rospy.loginfo(f"Available MoveIt groups: {self.available_groups}")
            
            # 等待move_group action服务器就绪
            self._fast_wait_for_move_group()
            
            # 初始化规划组
            self._parallel_init_move_groups()
            
            self._initialized = True
            elapsed = (rospy.Time.now() - start_time).to_sec()
            rospy.loginfo(f"✅ MoveIt初始化完成！耗时: {elapsed:.2f}秒")
            
        except Exception as e:
            rospy.logerr(f"MoveIt初始化失败: {e}")
        finally:
            self._initializing = False
    
    def _fast_wait_for_move_group(self):
        """快速等待move_group服务就绪"""
        import actionlib
        from moveit_msgs.msg import MoveGroupAction
        
        rospy.loginfo("等待move_group action服务器...")
        
        # 使用渐进式超时策略
        timeout_steps = [10.0, 15.0, 20.0]  # 渐进增加等待时间
        
        for attempt, timeout in enumerate(timeout_steps):
            try:
                rospy.loginfo(f"第{attempt+1}次尝试连接move_group action服务器（超时{timeout}秒）...")
                client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
                if client.wait_for_server(timeout=rospy.Duration(timeout)):
                    rospy.loginfo("✅ move_group action服务器连接成功")
                    # 验证连接有效性
                    if self._verify_action_connection(client):
                        return
                    else:
                        rospy.logwarn("action连接验证失败，重试...")
                else:
                    rospy.logwarn(f"⚠️ 第{attempt+1}次连接超时")
                    
                if attempt < len(timeout_steps) - 1:
                    rospy.sleep(2.0)  # 重试间隔
                    
            except Exception as e:
                rospy.logwarn(f"⚠️ 第{attempt+1}次连接异常: {e}")
                rospy.sleep(2.0)
        
        raise RuntimeError("move_group action服务器连接失败，请检查move_group是否正常启动")
    
    def _wait_for_move_group_services(self):
        """等待move_group相关服务启动"""
        rospy.loginfo("等待move_group服务启动...")
        
        # 等待关键服务
        critical_services = [
            '/move_group/plan_kinematic_path',
            '/get_planning_scene'
        ]
        
        for service in critical_services:
            try:
                rospy.loginfo(f"等待服务: {service}")
                rospy.wait_for_service(service, timeout=20.0)
                rospy.loginfo(f"✅ 服务 {service} 可用")
            except rospy.ROSException:
                rospy.logwarn(f"⚠️ 服务 {service} 等待超时，继续启动")
        
        rospy.loginfo("move_group服务检查完成")
    
    def _verify_action_connection(self, client):
        """验证action连接是否可用"""
        try:
            # 简单验证：检查客户端状态
            return client.get_state() is not None
        except Exception as e:
            rospy.logwarn(f"action连接验证失败: {e}")
            return False
    
    def _parallel_init_move_groups(self):
        """优化的MoveGroup初始化"""
        def init_group(group_name):
            try:
                rospy.loginfo(f"开始初始化{group_name}规划组...")
                if group_name == "arm":
                    self.arm_move_group = MoveGroupCommander("arm")
                    rospy.loginfo("✅ 成功初始化arm规划组")
                elif group_name == "gripper":
                    # gripper组只包含简单的prismatic关节，可能没有运动学求解器配置
                    try:
                        self.gripper_move_group = MoveGroupCommander("gripper")
                        # 检查是否有活动关节
                        joints = self.gripper_move_group.get_active_joints()
                        if joints:
                            rospy.loginfo(f"✅ 成功初始化gripper规划组，活动关节: {joints}")
                        else:
                            rospy.logwarn("⚠️ gripper组没有活动关节，但组已创建")
                    except Exception as e:
                        rospy.logwarn(f"gripper组初始化失败（prismatic关节组可能不需要运动学求解器）: {e}")
                        self.gripper_move_group = None
                elif group_name == "piper":
                    self.piper_move_group = MoveGroupCommander("piper")
                    rospy.loginfo("✅ 成功初始化piper规划组")
                return True
            except Exception as e:
                rospy.logerr(f"初始化{group_name}规划组失败: {e}")
                return False
        
        # 按优先级顺序初始化
        priority_groups = ["piper", "arm", "gripper"]
        for group_name in priority_groups:
            if group_name in self.available_groups:
                success = init_group(group_name)
                if not success and group_name in ["piper", "arm"]:
                    rospy.logerr(f"关键组{group_name}初始化失败")
    
    def _ensure_initialized(self):
        """确保MoveIt已初始化"""
        if not self._initialized:
            with self._init_lock:
                if not self._initialized and not self._initializing:
                    self._init_moveit()
            
            # 快速等待初始化完成
            timeout = rospy.Time.now() + rospy.Duration(15.0)  # 减少超时时间
            while not self._initialized and rospy.Time.now() < timeout:
                rospy.sleep(0.05)  # 减少轮询间隔
            
            if not self._initialized:
                raise RuntimeError("MoveIt初始化超时")


    def handle_joint_moveit_ctrl_arm(self, request):
        rospy.loginfo("Received arm joint movement request.")

        try:
            self._ensure_initialized()
            if self.arm_move_group:
                # The arm group now includes arm joints + gripper joints (joint1-joint8)
                # Convert single gripper value to two joint values
                gripper_value = request.gripper
                arm_joint_goal = list(request.joint_states[:6]) + [gripper_value, gripper_value]
                self.arm_move_group.set_joint_value_target(arm_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                rospy.loginfo("Arm movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_gripper(self, request):
        rospy.loginfo("Received gripper joint movement request.")

        try:
            self._ensure_initialized()
            if self.gripper_move_group:
                # The gripper group has two joints (joint7 and joint8)
                # Convert single gripper value to two joint values
                # For symmetric gripper, both joints move the same amount
                gripper_value = request.gripper
                gripper_goal = [gripper_value, gripper_value]  # Both joints get same value
                self.gripper_move_group.set_joint_value_target(gripper_goal)
                self.gripper_move_group.go(wait=True)
                rospy.loginfo("Gripper movement executed successfully.")
            else:
                rospy.logerr("Gripper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_piper(self, request):
        rospy.loginfo("Received piper joint movement request.")

        try:
            self._ensure_initialized()
            if self.piper_move_group:
                # The piper group now only controls arm joints (joint1-joint6)
                piper_joint_goal = request.joint_states[:6]
                self.piper_move_group.set_joint_value_target(piper_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.piper_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.piper_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.piper_move_group.go(wait=True)
                rospy.loginfo("Piper movement executed successfully.")
            else:
                rospy.logerr("Piper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during piper movement: {str(e)}")
        
        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_endpose(self, request):
        rospy.loginfo("Received endpose movement request.")

        try:
            self._ensure_initialized()
            if self.arm_move_group:
                position = request.joint_endpose[:3]
                if len(request.joint_endpose) == 7:
                    # 四元数 [qx, qy, qz, qw]
                    quaternion = request.joint_endpose[3:]
                    rospy.loginfo("Using Quaternion for orientation: (qx, qy, qz, qw) -> %f, %f, %f, %f", *quaternion)
                else:
                    rospy.logerr("Invalid joint_endpose size. It must be 7 (Quaternion).")
                    return JointMoveitCtrlResponse(status=False, error_code=1)
                
                target_pose = Pose()
                target_pose.position.x = position[0]
                target_pose.position.y = position[1]
                target_pose.position.z = position[2]
                target_pose.orientation.x = quaternion[0]
                target_pose.orientation.y = quaternion[1]
                target_pose.orientation.z = quaternion[2]
                target_pose.orientation.w = quaternion[3]

                self.arm_move_group.set_pose_target(target_pose)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                rospy.loginfo("Endpose movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during endpose movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

if __name__ == '__main__':
    JointMoveitCtrlServer()
    rospy.spin()
