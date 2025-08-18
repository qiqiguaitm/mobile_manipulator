#!/usr/bin/env python3
"""
运动平滑控制器 - 实现加速度限制和轨迹平滑
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from collections import deque
import threading

class MotionSmoother:
    def __init__(self):
        rospy.init_node('motion_smoother')
        
        # 加载安全配置
        self.load_safety_config()
        
        # 状态变量
        self.current_position = np.zeros(6)
        self.current_velocity = np.zeros(6)
        self.target_position = np.zeros(6)
        self.target_velocity = np.zeros(6)
        
        # 历史记录用于平滑
        self.position_history = deque(maxlen=10)
        self.velocity_history = deque(maxlen=10)
        
        # 订阅和发布
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.cmd_sub = rospy.Subscriber('/joint_command', JointState, self.command_callback)
        self.smooth_pub = rospy.Publisher('/smooth_joint_command', JointState, queue_size=1)
        
        # 控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.running = True
        
        # 安全状态
        self.emergency_stop = False
        self.speed_scale = 1.0
        
    def load_safety_config(self):
        """加载统一安全配置"""
        # 默认值
        self.max_vel = np.array([2.0] * 6)  # rad/s
        self.max_acc = np.array([1.0] * 6)  # rad/s²
        self.max_jerk = np.array([0.5] * 6) # rad/s³
        
        # 从参数服务器加载
        if rospy.has_param('/safety_config/motion_limits'):
            config = rospy.get_param('/safety_config/motion_limits')
            
            # 速度限制
            vel_scale = config.get('velocity', {}).get('default_scale', 0.5)
            self.max_vel *= vel_scale
            
            # 加速度限制
            if config.get('acceleration', {}).get('enabled', False):
                acc_scale = config.get('acceleration', {}).get('default_scale', 0.3)
                self.max_acc *= acc_scale
                self.emergency_decel = config.get('acceleration', {}).get('emergency_decel', 2.0)
            
            # 加加速度限制
            if config.get('jerk', {}).get('enabled', False):
                self.max_jerk *= config.get('jerk', {}).get('max_jerk', 1.0)
                self.smoothing_factor = config.get('jerk', {}).get('smoothing_factor', 0.8)
    
    def joint_callback(self, msg):
        """更新当前关节状态"""
        if len(msg.position) >= 6:
            self.current_position = np.array(msg.position[:6])
            if len(msg.velocity) >= 6:
                self.current_velocity = np.array(msg.velocity[:6])
            
            # 记录历史
            self.position_history.append(self.current_position.copy())
            self.velocity_history.append(self.current_velocity.copy())
    
    def command_callback(self, msg):
        """接收目标命令"""
        if self.emergency_stop:
            rospy.logwarn("Emergency stop active, ignoring command")
            return
            
        if len(msg.position) >= 6:
            self.target_position = np.array(msg.position[:6])
            if len(msg.velocity) >= 6:
                self.target_velocity = np.array(msg.velocity[:6])
            else:
                # 根据位置差计算目标速度
                pos_diff = self.target_position - self.current_position
                time_to_target = np.max(np.abs(pos_diff) / self.max_vel)
                if time_to_target > 0:
                    self.target_velocity = pos_diff / time_to_target
    
    def apply_acceleration_limits(self, current_vel, target_vel, dt):
        """应用加速度限制"""
        vel_diff = target_vel - current_vel
        max_vel_change = self.max_acc * dt
        
        # 限制速度变化
        vel_change = np.clip(vel_diff, -max_vel_change, max_vel_change)
        new_vel = current_vel + vel_change
        
        # 应用速度限制
        new_vel = np.clip(new_vel, -self.max_vel, self.max_vel)
        
        return new_vel
    
    def apply_jerk_limits(self, vel_history, new_vel, dt):
        """应用加加速度限制（平滑控制）"""
        if len(vel_history) < 2:
            return new_vel
            
        # 计算加速度
        current_acc = (vel_history[-1] - vel_history[-2]) / dt
        target_acc = (new_vel - vel_history[-1]) / dt
        
        # 限制加速度变化（加加速度）
        acc_diff = target_acc - current_acc
        max_acc_change = self.max_jerk * dt
        acc_change = np.clip(acc_diff, -max_acc_change, max_acc_change)
        
        # 计算平滑后的速度
        smoothed_acc = current_acc + acc_change
        smoothed_vel = vel_history[-1] + smoothed_acc * dt
        
        # 混合原始和平滑速度
        final_vel = self.smoothing_factor * smoothed_vel + (1 - self.smoothing_factor) * new_vel
        
        return final_vel
    
    def check_safety_zones(self):
        """检查安全区域并调整速度"""
        # 这里应该集成碰撞检测结果
        # 暂时使用模拟值
        min_distance = 0.1  # 假设最小距离
        
        if min_distance < 0.01:  # 紧急停止区
            self.emergency_stop = True
            self.speed_scale = 0.0
        elif min_distance < 0.03:  # 警告区
            self.speed_scale = 0.3
        elif min_distance < 0.05:  # 规划缓冲区
            self.speed_scale = 0.5
        else:
            self.speed_scale = 1.0
    
    def control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(200)  # 200Hz - 匹配控制频率
        dt = 0.005  # 1/200 = 0.005s
        
        while self.running and not rospy.is_shutdown():
            # 检查安全区域
            self.check_safety_zones()
            
            # 计算目标速度（考虑安全缩放）
            scaled_target_vel = self.target_velocity * self.speed_scale
            
            # 应用加速度限制
            new_vel = self.apply_acceleration_limits(
                self.current_velocity, scaled_target_vel, dt
            )
            
            # 应用加加速度限制（平滑）
            if len(self.velocity_history) >= 2:
                new_vel = self.apply_jerk_limits(
                    list(self.velocity_history), new_vel, dt
                )
            
            # 计算新位置
            new_pos = self.current_position + new_vel * dt
            
            # 发布平滑后的命令
            smooth_cmd = JointState()
            smooth_cmd.header.stamp = rospy.Time.now()
            smooth_cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            smooth_cmd.position = new_pos.tolist()
            smooth_cmd.velocity = new_vel.tolist()
            
            self.smooth_pub.publish(smooth_cmd)
            
            rate.sleep()
    
    def emergency_stop_callback(self):
        """紧急停止"""
        self.emergency_stop = True
        self.target_velocity = np.zeros(6)
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
        
        # 使用更大的减速度快速停止
        emergency_acc = np.array([self.emergency_decel] * 6)
        
        # 计算停止时间
        stop_time = np.max(np.abs(self.current_velocity) / emergency_acc)
        rospy.loginfo(f"Estimated stop time: {stop_time:.2f}s")
    
    def run(self):
        """启动平滑控制器"""
        self.control_thread.start()
        rospy.loginfo("Motion smoother started")
        
        # 注册紧急停止服务
        rospy.Service('/emergency_stop', Empty, 
                     lambda req: self.emergency_stop_callback())
        
        rospy.spin()
        self.running = False
        self.control_thread.join()

if __name__ == '__main__':
    try:
        smoother = MotionSmoother()
        smoother.run()
    except rospy.ROSInterruptException:
        pass