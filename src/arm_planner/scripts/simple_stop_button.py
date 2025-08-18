#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse, Trigger
from arm_planner.srv import GoZero
import tkinter as tk
from tkinter import ttk

class SimpleStopButton:
    """简单的机械臂停止按钮GUI"""
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('simple_stop_button', anonymous=True)
        
        # 初始化MoveIt - 添加重试机制
        self.arm_group = None
        self._init_moveit_with_retry()
        
        # 创建GUI
        self.root = tk.Tk()
        self.root.title("Piper Arm Emergency Stop")
        self.root.geometry("200x150")
        
        # 创建停止按钮
        self.stop_button = tk.Button(
            self.root,
            text="STOP",
            bg="red", 
            fg="white",
            font=("Arial", 12, "bold"),
            command=self.call_stop_srv,
            width=15,
            height=2
        )
        self.stop_button.pack(expand=True, pady=5)
        
        # 创建回零按钮
        self.go_zero_button = tk.Button(
            self.root,
            text="GO ZERO",
            bg="blue", 
            fg="white",
            font=("Arial", 10, "bold"),
            command=self.call_go_zero_srv,
            width=15,
            height=1
        )
        self.go_zero_button.pack(expand=True, pady=5)
        
        # 状态标签
        if self.arm_group is not None:
            self.status_label = tk.Label(self.root, text="Ready", fg="green")
        else:
            self.status_label = tk.Label(self.root, text="MoveGroup Not Ready", fg="orange")
        self.status_label.pack()
        
        rospy.loginfo("Simple stop button initialized")
    
    def _init_moveit_with_retry(self):
        """初始化MoveIt，带重试机制"""
        max_retries = 30  # 最多重试30次，每次1秒
        retry_count = 0
        
        while retry_count < max_retries and not rospy.is_shutdown():
            try:
                self.arm_group = moveit_commander.MoveGroupCommander("arm")
                rospy.loginfo("MoveGroup 'arm' initialized successfully")
                return
            except RuntimeError as e:
                retry_count += 1
                rospy.logwarn(f"MoveGroup 'arm' not ready, retry {retry_count}/{max_retries}: {e}")
                rospy.sleep(1.0)
        
        if self.arm_group is None:
            rospy.logerr("Failed to initialize MoveGroup 'arm' after maximum retries")
    
    def call_stop_srv(self):
        """调用stop_srv服务停止机械臂"""
        try:
            rospy.wait_for_service('stop_srv', timeout=2.0)
            stop_service = rospy.ServiceProxy('stop_srv', Trigger)
            response = stop_service()
            
             
            if response.success:
                self.status_label.config(text="STOPPED", fg="red")
                rospy.loginfo("Stop service called successfully!")
            else:
                self.status_label.config(text="STOP FAILED", fg="red")
                rospy.logerr(f"Stop service failed: {response.message}")
            
            # 2秒后恢复状态
            self.root.after(2000, self.reset_status)
            rospy.wait_for_service('reset_srv', timeout=2.0)
            reset_service = rospy.ServiceProxy('reset_srv', Trigger)
            response = reset_service()
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call stop service: {e}")
            self.status_label.config(text="SRV ERROR", fg="red")
        except rospy.ROSException as e:
            rospy.logerr(f"Stop service not available: {e}")
            self.status_label.config(text="NO SERVICE", fg="orange")
    
    def call_go_zero_srv(self):
        """调用go_zero_srv服务让机械臂回零位"""
        try:
            rospy.wait_for_service('go_zero_srv', timeout=2.0)
            go_zero_service = rospy.ServiceProxy('go_zero_srv', GoZero)
            response = go_zero_service(is_mit_mode=False)
            
            if response.status:
                self.status_label.config(text="GOING ZERO", fg="blue")
                rospy.loginfo("Go zero service called successfully!")
            else:
                self.status_label.config(text="ZERO FAILED", fg="red")
                rospy.logerr(f"Go zero service failed: code {response.code}")
            
            # 3秒后恢复状态（回零需要更多时间）
            self.root.after(3000, self.reset_status)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call go zero service: {e}")
            self.status_label.config(text="SRV ERROR", fg="red")
        except rospy.ROSException as e:
            rospy.logerr(f"Go zero service not available: {e}")
            self.status_label.config(text="NO SERVICE", fg="orange")
    
    def reset_status(self):
        """重置状态显示"""
        self.status_label.config(text="Ready", fg="green")
    
    def run(self):
        """运行GUI主循环"""
        try:
            self.root.mainloop()
        except rospy.ROSInterruptException:
            pass
        finally:
            moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        stop_button = SimpleStopButton()
        stop_button.run()
    except rospy.ROSInterruptException:
        pass