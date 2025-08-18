#!/usr/bin/env python3
"""
分级安全响应管理器 - 根据危险等级采取不同的安全措施
"""

import rospy
from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
from enum import Enum
import threading

class SafetyLevel(Enum):
    """安全等级定义"""
    NORMAL = 0      # 正常运行
    CAUTION = 1     # 注意级别
    WARNING = 2     # 警告级别
    CRITICAL = 3    # 危急级别
    EMERGENCY = 4   # 紧急停止

class SafetyResponseManager:
    def __init__(self):
        rospy.init_node('safety_response_manager')
        
        # 加载配置
        self.load_safety_config()
        
        # 状态变量
        self.current_safety_level = SafetyLevel.NORMAL
        self.min_collision_distance = float('inf')
        self.speed_scale = 1.0
        self.is_stopped = False
        
        # 安全历史记录
        self.safety_history = []
        self.event_count = {level: 0 for level in SafetyLevel}
        
        # 订阅器
        self.distance_sub = rospy.Subscriber(
            '/collision_distances', Float32MultiArray, self.distance_callback
        )
        
        # 发布器
        self.safety_level_pub = rospy.Publisher(
            '/safety_level', String, queue_size=1
        )
        self.speed_scale_pub = rospy.Publisher(
            '/speed_scale_factor', Float32MultiArray, queue_size=1
        )
        self.stop_pub = rospy.Publisher(
            '/emergency_stop_signal', Bool, queue_size=1
        )
        
        # 服务
        self.reset_srv = rospy.Service(
            '/reset_safety_system', Empty, self.reset_callback
        )
        
        # 响应线程
        self.response_thread = threading.Thread(target=self.response_loop)
        self.response_thread.daemon = True
        self.running = True
        
        rospy.loginfo("Safety Response Manager initialized")
    
    def load_safety_config(self):
        """加载安全配置"""
        # 默认配置
        self.thresholds = {
            SafetyLevel.EMERGENCY: 0.010,  # 10mm
            SafetyLevel.CRITICAL: 0.020,   # 20mm
            SafetyLevel.WARNING: 0.030,     # 30mm
            SafetyLevel.CAUTION: 0.050,     # 50mm
        }
        
        self.speed_scales = {
            SafetyLevel.NORMAL: 1.0,
            SafetyLevel.CAUTION: 0.7,
            SafetyLevel.WARNING: 0.3,
            SafetyLevel.CRITICAL: 0.1,
            SafetyLevel.EMERGENCY: 0.0,
        }
        
        # 从参数服务器加载
        if rospy.has_param('/safety_config/safety_zones'):
            zones = rospy.get_param('/safety_config/safety_zones')
            
            self.thresholds[SafetyLevel.EMERGENCY] = zones.get(
                'emergency_stop', {}).get('distance', 0.010)
            self.thresholds[SafetyLevel.WARNING] = zones.get(
                'warning_zone', {}).get('distance', 0.030)
            
            # 速度缩放
            self.speed_scales[SafetyLevel.WARNING] = zones.get(
                'warning_zone', {}).get('speed_scale', 0.3)
        
        # 响应延迟（防止抖动）
        self.level_hold_time = {
            SafetyLevel.NORMAL: 0.0,
            SafetyLevel.CAUTION: 0.5,
            SafetyLevel.WARNING: 1.0,
            SafetyLevel.CRITICAL: 2.0,
            SafetyLevel.EMERGENCY: 5.0,
        }
        
        self.last_level_change_time = rospy.Time.now()
    
    def distance_callback(self, msg):
        """处理碰撞距离数据"""
        if len(msg.data) > 0:
            self.min_collision_distance = msg.data[0]
            
            # 确定安全等级
            new_level = self.determine_safety_level(self.min_collision_distance)
            
            # 检查是否需要改变等级
            if new_level != self.current_safety_level:
                self.change_safety_level(new_level)
    
    def determine_safety_level(self, distance):
        """根据距离确定安全等级"""
        for level in [SafetyLevel.EMERGENCY, SafetyLevel.CRITICAL, 
                     SafetyLevel.WARNING, SafetyLevel.CAUTION]:
            if distance <= self.thresholds[level]:
                return level
        return SafetyLevel.NORMAL
    
    def change_safety_level(self, new_level):
        """改变安全等级"""
        current_time = rospy.Time.now()
        
        # 检查等级保持时间（防抖动）
        if self.current_safety_level != SafetyLevel.NORMAL:
            elapsed = (current_time - self.last_level_change_time).to_sec()
            required_hold = self.level_hold_time[self.current_safety_level]
            
            # 只允许升级到更高危险等级，或在保持时间后降级
            if new_level.value <= self.current_safety_level.value and elapsed < required_hold:
                return
        
        # 记录事件
        self.event_count[new_level] += 1
        self.safety_history.append({
            'time': current_time,
            'from_level': self.current_safety_level,
            'to_level': new_level,
            'distance': self.min_collision_distance
        })
        
        # 更新状态
        old_level = self.current_safety_level
        self.current_safety_level = new_level
        self.last_level_change_time = current_time
        
        # 记录日志
        rospy.logwarn(f"Safety level changed: {old_level.name} -> {new_level.name} "
                     f"(distance: {self.min_collision_distance:.3f}m)")
        
        # 触发相应的安全响应
        self.execute_safety_response(new_level)
    
    def execute_safety_response(self, level):
        """执行安全响应"""
        # 更新速度缩放
        self.speed_scale = self.speed_scales[level]
        
        # 发布速度缩放因子
        scale_msg = Float32MultiArray()
        scale_msg.data = [self.speed_scale] * 6  # 所有关节使用相同缩放
        self.speed_scale_pub.publish(scale_msg)
        
        # 发布安全等级
        level_msg = String()
        level_msg.data = level.name
        self.safety_level_pub.publish(level_msg)
        
        # 特定等级的响应
        if level == SafetyLevel.EMERGENCY:
            self.emergency_stop()
        elif level == SafetyLevel.CRITICAL:
            self.critical_response()
        elif level == SafetyLevel.WARNING:
            self.warning_response()
        elif level == SafetyLevel.CAUTION:
            self.caution_response()
        else:
            self.normal_response()
    
    def emergency_stop(self):
        """紧急停止响应"""
        rospy.logerr("EMERGENCY STOP ACTIVATED!")
        
        # 发送停止信号
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)
        self.is_stopped = True
        
        # 调用所有停止服务
        try:
            stop_service = rospy.ServiceProxy('/stop_arm', Empty)
            stop_service()
        except:
            pass
        
        # 发送零速度命令
        try:
            cmd_pub = rospy.Publisher('/joint_velocity_command', JointState, queue_size=1)
            stop_cmd = JointState()
            stop_cmd.velocity = [0.0] * 6
            cmd_pub.publish(stop_cmd)
        except:
            pass
    
    def critical_response(self):
        """危急响应"""
        rospy.logwarn("CRITICAL: Collision imminent, reducing speed to 10%")
        
        # 可以添加声音警报等
        # self.sound_alarm('critical')
    
    def warning_response(self):
        """警告响应"""
        rospy.logwarn(f"WARNING: Close to collision, speed reduced to {self.speed_scale*100:.0f}%")
        
        # 可以添加视觉警告
        # self.visual_warning('yellow')
    
    def caution_response(self):
        """注意响应"""
        rospy.loginfo(f"CAUTION: Approaching limits, speed at {self.speed_scale*100:.0f}%")
    
    def normal_response(self):
        """正常状态响应"""
        if self.is_stopped:
            rospy.loginfo("Returning to normal operation")
            self.is_stopped = False
            
            # 发送恢复信号
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_pub.publish(stop_msg)
    
    def reset_callback(self, req):
        """重置安全系统"""
        rospy.loginfo("Resetting safety system...")
        
        # 只有在非紧急状态下才允许重置
        if self.current_safety_level == SafetyLevel.EMERGENCY:
            if self.min_collision_distance > self.thresholds[SafetyLevel.CRITICAL]:
                self.current_safety_level = SafetyLevel.NORMAL
                self.is_stopped = False
                self.execute_safety_response(SafetyLevel.NORMAL)
                rospy.loginfo("Safety system reset successful")
            else:
                rospy.logerr("Cannot reset: Still in dangerous zone!")
                return False
        
        return True
    
    def response_loop(self):
        """响应循环 - 定期检查和更新"""
        rate = rospy.Rate(10)  # 10Hz
        
        while self.running and not rospy.is_shutdown():
            # 发布当前状态
            level_msg = String()
            level_msg.data = f"{self.current_safety_level.name}:{self.min_collision_distance:.3f}m"
            self.safety_level_pub.publish(level_msg)
            
            # 定期检查是否可以降级
            if self.current_safety_level != SafetyLevel.NORMAL:
                new_level = self.determine_safety_level(self.min_collision_distance)
                if new_level.value < self.current_safety_level.value:
                    self.change_safety_level(new_level)
            
            rate.sleep()
    
    def print_statistics(self):
        """打印统计信息"""
        rospy.loginfo("=== Safety System Statistics ===")
        for level in SafetyLevel:
            rospy.loginfo(f"{level.name}: {self.event_count[level]} events")
        rospy.loginfo(f"Total events: {len(self.safety_history)}")
        
        if self.safety_history:
            # 计算平均恢复时间
            recovery_times = []
            for i in range(1, len(self.safety_history)):
                if self.safety_history[i]['to_level'] == SafetyLevel.NORMAL:
                    duration = (self.safety_history[i]['time'] - 
                              self.safety_history[i-1]['time']).to_sec()
                    recovery_times.append(duration)
            
            if recovery_times:
                avg_recovery = np.mean(recovery_times)
                rospy.loginfo(f"Average recovery time: {avg_recovery:.2f}s")
    
    def run(self):
        """运行安全响应管理器"""
        self.response_thread.start()
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown)
        
        rospy.spin()
    
    def shutdown(self):
        """关闭时打印统计"""
        self.running = False
        self.print_statistics()
        self.response_thread.join()

if __name__ == '__main__':
    try:
        manager = SafetyResponseManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass