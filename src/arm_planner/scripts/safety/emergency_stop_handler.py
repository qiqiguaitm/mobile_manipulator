#!/usr/bin/env python3
"""
基础紧急停止处理器 - 提供最基本的安全保护
"""

import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class EmergencyStopHandler:
    def __init__(self):
        rospy.init_node('emergency_stop_handler')
        
        # 停止状态
        self.is_stopped = False
        
        # 发布器
        self.stop_pub = rospy.Publisher('/emergency_stop_signal', Bool, queue_size=1)
        self.joint_cmd_pub = rospy.Publisher('/joint_command', JointState, queue_size=1)
        
        # 服务
        self.stop_srv = rospy.Service('/emergency_stop', Empty, self.emergency_stop_callback)
        self.reset_srv = rospy.Service('/reset_emergency_stop', Empty, self.reset_callback)
        
        rospy.loginfo("Emergency stop handler ready")
    
    def emergency_stop_callback(self, req):
        """执行紧急停止"""
        if not self.is_stopped:
            self.is_stopped = True
            rospy.logerr("EMERGENCY STOP ACTIVATED!")
            
            # 发布停止信号
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            
            # 发送零速度命令
            stop_cmd = JointState()
            stop_cmd.velocity = [0.0] * 6
            self.joint_cmd_pub.publish(stop_cmd)
            
            # 尝试调用arm控制器的停止服务
            try:
                stop_arm = rospy.ServiceProxy('/stop_arm', Empty)
                stop_arm()
            except:
                pass
        
        return EmptyResponse()
    
    def reset_callback(self, req):
        """重置紧急停止"""
        if self.is_stopped:
            self.is_stopped = False
            rospy.loginfo("Emergency stop reset")
            
            # 发布恢复信号
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_pub.publish(stop_msg)
        
        return EmptyResponse()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        handler = EmergencyStopHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass