#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QLabel
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
import rviz

class PiperStopPanel(rviz.Panel):
    """Piper机械臂停止控制面板"""
    
    def __init__(self, parent=None):
        super(PiperStopPanel, self).__init__(parent)
        
        # 初始化MoveIt（需要检查是否已经初始化）
        try:
            if not moveit_commander.roscpp_is_initialized():
                moveit_commander.roscpp_initialize(sys.argv)
            self.arm_group = moveit_commander.MoveGroupCommander("arm")
            rospy.loginfo("PiperStopPanel: MoveGroup initialized")
        except Exception as e:
            rospy.logerr(f"PiperStopPanel: Failed to initialize MoveGroup: {e}")
            self.arm_group = None
        
        # 创建UI
        layout = QVBoxLayout()
        
        # 紧急停止按钮
        self.stop_button = QPushButton("EMERGENCY STOP")
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                border: none;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #c62828;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        self.stop_button.setMinimumHeight(50)
        self.stop_button.clicked.connect(self.on_stop_clicked)
        
        # 状态标签
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("QLabel { color: green; font-size: 12px; }")
        
        # 说明标签
        info_label = QLabel("Click to immediately stop arm movement")
        info_label.setStyleSheet("QLabel { color: gray; font-size: 10px; }")
        
        layout.addWidget(self.stop_button)
        layout.addWidget(self.status_label)
        layout.addWidget(info_label)
        
        self.setLayout(layout)
        
        # 计时器用于重置状态
        self.reset_timer = QTimer()
        self.reset_timer.timeout.connect(self.reset_status)
        
    def on_stop_clicked(self):
        """停止按钮点击处理"""
        if not self.arm_group:
            self.status_label.setText("MoveGroup Error")
            self.status_label.setStyleSheet("QLabel { color: red; font-size: 12px; }")
            return
        
        try:
            # 执行停止操作
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            # 更新状态
            self.status_label.setText("STOPPED")
            self.status_label.setStyleSheet("QLabel { color: red; font-size: 12px; font-weight: bold; }")
            
            rospy.logwarn("Emergency stop executed from RViz panel!")
            
            # 1.5秒后重置状态
            self.reset_timer.start(1500)
            
        except Exception as e:
            rospy.logerr(f"Failed to stop arm: {e}")
            self.status_label.setText("Stop Failed")
            self.status_label.setStyleSheet("QLabel { color: red; font-size: 12px; }")
            
    def reset_status(self):
        """重置状态显示"""
        self.reset_timer.stop()
        self.status_label.setText("Ready")
        self.status_label.setStyleSheet("QLabel { color: green; font-size: 12px; }")