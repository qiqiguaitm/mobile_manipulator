#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse, Trigger
from arm_planner.srv import GoZero, Enable
import tkinter as tk
from tkinter import ttk, messagebox

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
        self.root.geometry("280x320")
        self.root.configure(bg='#f0f0f0')
        self.root.resizable(False, False)
        
        # 创建主框架
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(expand=True, fill='both', padx=15, pady=15)
        
        # 统一的按钮样式
        button_width = 18
        button_height = 2
        button_font = ("Arial", 12, "bold")
        
        # 创建停止按钮 (黄色)
        self.stop_button = tk.Button(
            main_frame,
            text="STOP",
            bg="#FFD700",  # 金黄色
            fg="black",
            font=button_font,
            command=self.call_stop_srv,
            width=button_width,
            height=button_height,
            relief=tk.RAISED,
            bd=3,
            activebackground="#FFA500"  # 按下时橙色
        )
        self.stop_button.pack(pady=8)
        
        # 创建回零按钮 (蓝色)
        self.go_zero_button = tk.Button(
            main_frame,
            text="GO ZERO",
            bg="#1E90FF",  # 道奇蓝
            fg="white",
            font=button_font,
            command=self.call_go_zero_srv,
            width=button_width,
            height=button_height,
            relief=tk.RAISED,
            bd=3,
            activebackground="#0000CD"  # 按下时深蓝
        )
        self.go_zero_button.pack(pady=8)
        
        # 创建使能/禁用按钮 (根据状态变色)
        self.enable_button = tk.Button(
            main_frame,
            text="DISABLE",
            bg="#DC143C",  # 深红色
            fg="white",
            font=button_font,
            command=self.toggle_enable,
            width=button_width,
            height=button_height,
            relief=tk.RAISED,
            bd=3,
            activebackground="#8B0000"  # 按下时暗红
        )
        self.enable_button.pack(pady=8)
        
        # 分隔线
        separator = ttk.Separator(main_frame, orient='horizontal')
        separator.pack(fill='x', pady=10)
        
        # 状态标签框架
        status_frame = tk.Frame(main_frame, bg='#f0f0f0')
        status_frame.pack(pady=5)
        
        # 状态标签
        self.status_label = tk.Label(
            status_frame, 
            text="Ready", 
            fg="green",
            bg='#f0f0f0',
            font=("Arial", 11)
        )
        self.status_label.pack()
        
        # 使能状态标签
        self.enable_status_label = tk.Label(
            status_frame, 
            text="Arm: ENABLED", 
            fg="green",
            bg='#f0f0f0',
            font=("Arial", 11, "bold")
        )
        self.enable_status_label.pack(pady=(5, 0))
        
        if self.arm_group is None:
            self.status_label.config(text="MoveGroup Not Ready", fg="orange")
        
        # 添加按钮悬停效果
        self._setup_button_hover_effects()
        
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
    
    def _setup_button_hover_effects(self):
        """设置按钮悬停效果"""
        def on_enter(event, button, hover_color):
            button['bg'] = hover_color
            
        def on_leave(event, button, normal_color):
            button['bg'] = normal_color
        
        # STOP按钮悬停效果
        self.stop_button.bind("<Enter>", lambda e: on_enter(e, self.stop_button, "#FFA500"))
        self.stop_button.bind("<Leave>", lambda e: on_leave(e, self.stop_button, "#FFD700"))
        
        # GO ZERO按钮悬停效果
        self.go_zero_button.bind("<Enter>", lambda e: on_enter(e, self.go_zero_button, "#4169E1"))
        self.go_zero_button.bind("<Leave>", lambda e: on_leave(e, self.go_zero_button, "#1E90FF"))
        
        # ENABLE/DISABLE按钮悬停效果会在状态改变时动态设置
        self._update_enable_button_hover()
    
    def _update_enable_button_hover(self):
        """更新使能按钮的悬停效果"""
        # 清除旧的绑定
        self.enable_button.unbind("<Enter>")
        self.enable_button.unbind("<Leave>")
        
        if self.is_enabled:
            # DISABLE状态 - 红色系
            self.enable_button.bind("<Enter>", lambda e: self.enable_button.config(bg="#B22222"))
            self.enable_button.bind("<Leave>", lambda e: self.enable_button.config(bg="#DC143C"))
        else:
            # ENABLE状态 - 绿色系
            self.enable_button.bind("<Enter>", lambda e: self.enable_button.config(bg="#32CD32"))
            self.enable_button.bind("<Leave>", lambda e: self.enable_button.config(bg="#228B22"))
    
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
    
    def toggle_enable(self):
        """切换机械臂使能状态"""
        # 切换目标状态
        target_enable = not self.is_enabled
        
        # 如果要禁用机械臂，需要二次确认
        if self.is_enabled and not target_enable:
            result = messagebox.askyesno(
                "确认禁用", 
                "确定要禁用机械臂吗？\n\n禁用后机械臂将无法移动。",
                icon='warning'
            )
            if not result:
                return
        
        try:
            rospy.wait_for_service('enable_srv', timeout=2.0)
            enable_service = rospy.ServiceProxy('enable_srv', Enable)
            response = enable_service(enable_request=target_enable)
            
            if response.enable_response:
                self.is_enabled = target_enable
                if self.is_enabled:
                    self.enable_button.config(
                        text="DISABLE", 
                        bg="#DC143C",  # 深红色
                        activebackground="#8B0000"
                    )
                    self.enable_status_label.config(text="Arm: ENABLED", fg="green")
                    self.status_label.config(text="ENABLED", fg="green")
                    rospy.loginfo("Arm enabled successfully!")
                else:
                    self.enable_button.config(
                        text="ENABLE", 
                        bg="#228B22",  # 森林绿
                        activebackground="#006400"  # 深绿
                    )
                    self.enable_status_label.config(text="Arm: DISABLED", fg="red")
                    self.status_label.config(text="DISABLED", fg="orange")
                    rospy.loginfo("Arm disabled successfully!")
                
                # 更新悬停效果
                self._update_enable_button_hover()
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