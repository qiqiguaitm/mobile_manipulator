#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse, Trigger
from arm_planner.srv import GoZero, Enable
import tkinter as tk
from tkinter import ttk

class EnhancedStopButton:
    """增强的机械臂控制按钮GUI - 包含停止、回零和使能控制"""
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('enhanced_stop_button', anonymous=True)
        
        # 初始化MoveIt - 添加重试机制
        self.arm_group = None
        self._init_moveit_with_retry()
        
        # 跟踪使能状态
        self.is_enabled = True
        
        # 创建GUI
        self.root = tk.Tk()
        self.root.title("Piper Arm Control Panel")
        self.root.geometry("250x250")
        self.root.configure(bg='#f0f0f0')
        
        # 创建主框架
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(expand=True, fill='both', padx=10, pady=10)
        
        # 创建停止按钮
        self.stop_button = tk.Button(
            main_frame,
            text="STOP",
            bg="red", 
            fg="white",
            font=("Arial", 14, "bold"),
            command=self.call_stop_srv,
            width=15,
            height=2,
            relief=tk.RAISED,
            bd=3
        )
        self.stop_button.pack(pady=5)
        
        # 创建回零按钮
        self.go_zero_button = tk.Button(
            main_frame,
            text="GO ZERO",
            bg="blue", 
            fg="white",
            font=("Arial", 12, "bold"),
            command=self.call_go_zero_srv,
            width=15,
            height=2,
            relief=tk.RAISED,
            bd=3
        )
        self.go_zero_button.pack(pady=5)
        
        # 创建使能/禁用按钮
        self.enable_button = tk.Button(
            main_frame,
            text="DISABLE",
            bg="orange", 
            fg="white",
            font=("Arial", 12, "bold"),
            command=self.toggle_enable,
            width=15,
            height=2,
            relief=tk.RAISED,
            bd=3
        )
        self.enable_button.pack(pady=5)
        
        # 状态标签框架
        status_frame = tk.Frame(main_frame, bg='#f0f0f0')
        status_frame.pack(pady=10)
        
        # 状态标签
        self.status_label = tk.Label(
            status_frame, 
            text="Ready", 
            fg="green",
            bg='#f0f0f0',
            font=("Arial", 10)
        )
        self.status_label.pack()
        
        # 使能状态标签
        self.enable_status_label = tk.Label(
            status_frame, 
            text="Arm: ENABLED", 
            fg="green",
            bg='#f0f0f0',
            font=("Arial", 10, "bold")
        )
        self.enable_status_label.pack()
        
        if self.arm_group is None:
            self.status_label.config(text="MoveGroup Not Ready", fg="orange")
        
        rospy.loginfo("Enhanced stop button initialized")
    
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
            
            # 1秒后恢复状态
            self.root.after(1000, self.reset_status)
            
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
    
    def toggle_enable(self):
        """切换机械臂使能状态"""
        # 切换目标状态
        target_enable = not self.is_enabled
        
        try:
            rospy.wait_for_service('enable_srv', timeout=2.0)
            enable_service = rospy.ServiceProxy('enable_srv', Enable)
            response = enable_service(enable_request=target_enable)
            
            if response.enable_response:
                self.is_enabled = target_enable
                if self.is_enabled:
                    self.enable_button.config(text="DISABLE", bg="orange")
                    self.enable_status_label.config(text="Arm: ENABLED", fg="green")
                    self.status_label.config(text="ENABLED", fg="green")
                    rospy.loginfo("Arm enabled successfully!")
                else:
                    self.enable_button.config(text="ENABLE", bg="green")
                    self.enable_status_label.config(text="Arm: DISABLED", fg="red")
                    self.status_label.config(text="DISABLED", fg="orange")
                    rospy.loginfo("Arm disabled successfully!")
            else:
                self.status_label.config(text="TOGGLE FAILED", fg="red")
                rospy.logerr("Failed to toggle enable state")
            
            # 2秒后恢复状态显示
            self.root.after(2000, self.reset_status)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call enable service: {e}")
            self.status_label.config(text="SRV ERROR", fg="red")
        except rospy.ROSException as e:
            rospy.logerr(f"Enable service not available: {e}")
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
        stop_button = EnhancedStopButton()
        stop_button.run()
    except rospy.ROSInterruptException:
        pass