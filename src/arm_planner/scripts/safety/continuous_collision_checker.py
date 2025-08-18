#!/usr/bin/env python3
"""
连续碰撞检测器 - 使用MoveIt的碰撞检测功能
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray, Header, Bool
from geometry_msgs.msg import Point
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
import threading
import time

class ContinuousCollisionChecker:
    def __init__(self):
        rospy.init_node('continuous_collision_checker')
        
        # 等待MoveIt服务
        self.validity_service = None
        self.wait_for_moveit_services()
        
        # 安全配置
        self.load_safety_config()
        
        # 状态变量
        self.current_joint_positions = np.zeros(6)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.min_distance = float('inf')
        self.is_collision_free = True
        self.last_check_time = 0
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 订阅器和发布器
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.distance_pub = rospy.Publisher('/collision_distances', Float32MultiArray, queue_size=1)
        self.collision_pub = rospy.Publisher('/collision_detected', Bool, queue_size=1)
        self.marker_pub = rospy.Publisher('/collision_markers', MarkerArray, queue_size=1)
        
        # 检测线程
        self.running = True
        self.check_thread = threading.Thread(target=self.check_loop)
        self.check_thread.daemon = True
        self.check_thread.start()
        
        rospy.loginfo("连续碰撞检测器已启动")
        
    def wait_for_moveit_services(self):
        """等待MoveIt服务"""
        rospy.loginfo("等待MoveIt碰撞检测服务...")
        service_name = '/check_state_validity'
        
        # 多次尝试连接服务
        for i in range(5):
            try:
                rospy.wait_for_service(service_name, timeout=5.0)
                self.validity_service = rospy.ServiceProxy(service_name, GetStateValidity)
                rospy.loginfo("已连接到MoveIt碰撞检测服务")
                return
            except rospy.ROSException:
                rospy.logwarn(f"尝试 {i+1}/5: MoveIt服务未就绪，等待中...")
                rospy.sleep(2.0)
        
        rospy.logwarn("无法连接到MoveIt服务，将运行在降级模式")
        
    def load_safety_config(self):
        """加载安全配置"""
        # 默认安全距离
        self.emergency_distance = 0.01   # 10mm
        self.warning_distance = 0.03     # 30mm
        self.planning_distance = 0.05    # 50mm
        
        # 从参数服务器加载
        if rospy.has_param('/safety_config/safety_zones'):
            zones = rospy.get_param('/safety_config/safety_zones')
            self.emergency_distance = zones.get('emergency_stop', {}).get('distance', 0.01)
            self.warning_distance = zones.get('warning_zone', {}).get('distance', 0.03)
            self.planning_distance = zones.get('planning_buffer', {}).get('distance', 0.05)
        
        # 检测频率
        self.check_frequency = rospy.get_param('~check_frequency', 50.0)
        self.use_gpu = rospy.get_param('~use_gpu_acceleration', False)
        
        rospy.loginfo(f"安全配置已加载 - 紧急距离: {self.emergency_distance}m, 警告距离: {self.warning_distance}m")
        
    def joint_callback(self, msg):
        """更新关节位置"""
        with self.lock:
            if len(msg.position) >= 6:
                self.current_joint_positions = np.array(msg.position[:6])
                
    def check_state_validity(self, joint_positions):
        """使用MoveIt检查状态有效性"""
        if self.validity_service is None:
            # 降级模式：只检查关节限位
            return self.check_joint_limits(joint_positions)
            
        try:
            # 创建请求
            req = GetStateValidityRequest()
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = self.joint_names
            req.robot_state.joint_state.position = joint_positions.tolist()
            req.group_name = "arm"  # 或 "piper_with_gripper"
            
            # 调用服务
            resp = self.validity_service(req)
            
            # 解析响应
            is_valid = resp.valid
            
            # 估算距离（MoveIt不直接提供距离）
            if is_valid:
                # 如果有效，假设距离大于警告距离
                distance = self.planning_distance
            else:
                # 如果碰撞，假设距离为0
                distance = 0.0
                
            return is_valid, distance
            
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"MoveIt服务调用失败: {e}")
            return self.check_joint_limits(joint_positions)
            
    def check_joint_limits(self, joint_positions):
        """基本的关节限位检查（降级模式）"""
        # 关节限位（弧度）
        joint_limits = [
            (-3.14, 3.14),   # joint1
            (-1.57, 1.57),   # joint2
            (-1.57, 1.57),   # joint3
            (-3.14, 3.14),   # joint4
            (-1.57, 1.57),   # joint5
            (-3.14, 3.14),   # joint6
        ]
        
        is_valid = True
        min_margin = float('inf')
        
        for i, (pos, (lower, upper)) in enumerate(zip(joint_positions, joint_limits)):
            if pos < lower or pos > upper:
                is_valid = False
                min_margin = 0.0
                rospy.logwarn(f"关节{i+1}超出限位: {pos:.3f} not in [{lower:.3f}, {upper:.3f}]")
            else:
                margin = min(pos - lower, upper - pos)
                min_margin = min(min_margin, margin)
                
        # 将角度裕度转换为近似的笛卡尔距离
        distance = min_margin * 0.1  # 简单的缩放因子
        
        return is_valid, distance
        
    def publish_status(self, is_valid, distance):
        """发布碰撞检测状态"""
        # 发布距离
        distance_msg = Float32MultiArray()
        distance_msg.data = [distance]
        self.distance_pub.publish(distance_msg)
        
        # 发布碰撞状态
        collision_msg = Bool()
        collision_msg.data = not is_valid
        self.collision_pub.publish(collision_msg)
        
        # 发布可视化标记
        self.publish_visualization(is_valid, distance)
        
    def publish_visualization(self, is_valid, distance):
        """发布可视化标记"""
        marker_array = MarkerArray()
        
        # 状态指示器
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        
        # 根据距离设置颜色
        if not is_valid or distance < self.emergency_distance:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif distance < self.warning_distance:
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
        marker.color.a = 0.8
        marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        
    def check_loop(self):
        """主检测循环"""
        rate = rospy.Rate(self.check_frequency)
        
        while self.running and not rospy.is_shutdown():
            try:
                # 获取当前关节位置
                with self.lock:
                    joint_pos = self.current_joint_positions.copy()
                    
                # 执行碰撞检测
                start_time = time.time()
                is_valid, distance = self.check_state_validity(joint_pos)
                check_time = time.time() - start_time
                
                # 更新状态
                with self.lock:
                    self.is_collision_free = is_valid
                    self.min_distance = distance
                    self.last_check_time = check_time
                    
                # 发布状态
                self.publish_status(is_valid, distance)
                
                # 日志输出
                if not is_valid:
                    rospy.logwarn_throttle(1.0, f"碰撞检测: 距离={distance:.3f}m")
                elif distance < self.warning_distance:
                    rospy.loginfo_throttle(2.0, f"接近碰撞: 距离={distance:.3f}m")
                    
                # 性能监控
                if check_time > 0.01:  # 10ms
                    rospy.logdebug_throttle(5.0, f"碰撞检测耗时: {check_time*1000:.1f}ms")
                    
            except Exception as e:
                rospy.logerr_throttle(5.0, f"碰撞检测错误: {e}")
                
            rate.sleep()
            
    def shutdown(self):
        """关闭清理"""
        self.running = False
        if self.check_thread.is_alive():
            self.check_thread.join(timeout=1.0)
        rospy.loginfo("连续碰撞检测器已关闭")

if __name__ == '__main__':
    try:
        checker = ContinuousCollisionChecker()
        rospy.on_shutdown(checker.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass