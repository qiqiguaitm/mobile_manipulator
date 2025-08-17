#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
碰撞场景管理器 - 负责添加环境碰撞对象
主要功能：
1. 添加地面碰撞平面，防止机械臂撞击地面
2. 可扩展添加其他环境障碍物（墙壁、天花板等）
"""

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
import time

class CollisionSceneManager:
    def __init__(self):
        rospy.init_node('collision_scene_manager')
        
        # 初始化planning scene接口
        self.scene = PlanningSceneInterface()
        
        # 等待scene服务就绪
        rospy.loginfo("等待Planning Scene服务就绪...")
        rospy.sleep(2.0)
        
        # 参数配置
        self.ground_height = rospy.get_param('~ground_height', 0.0)
        self.ground_size = rospy.get_param('~ground_size', 10.0)
        self.safety_margin = rospy.get_param('~safety_margin', 0.05)  # 5cm安全距离
        
        rospy.loginfo("碰撞场景管理器初始化完成")
        
    def add_ground_plane(self):
        """添加地面碰撞平面"""
        rospy.loginfo("正在添加地面碰撞平面...")
        
        # 创建地面位姿
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "world"
        ground_pose.header.stamp = rospy.Time.now()
        
        # 地面位置：略低于ground_height，考虑安全距离
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = self.ground_height - self.safety_margin - 0.005  # 地面厚度一半
        ground_pose.pose.orientation.w = 1.0
        
        # 添加地面（大平面）
        ground_name = "ground"
        ground_size = (self.ground_size, self.ground_size, 0.01)  # 10m x 10m x 1cm
        
        self.scene.add_box(ground_name, ground_pose, ground_size)
        
        # 等待场景更新
        rospy.sleep(0.5)
        
        # 验证是否添加成功
        if self._wait_for_object(ground_name):
            rospy.loginfo(f"✅ 地面碰撞平面添加成功 (高度: {self.ground_height - self.safety_margin}m)")
        else:
            rospy.logerr("❌ 地面碰撞平面添加失败！")
            
    def add_workspace_boundaries(self):
        """添加工作空间边界（可选）"""
        rospy.loginfo("添加工作空间边界...")
        
        # 工作空间参数
        workspace_x = rospy.get_param('~workspace_x', 2.0)
        workspace_y = rospy.get_param('~workspace_y', 2.0)
        workspace_z = rospy.get_param('~workspace_z', 2.5)
        
        # 墙壁厚度
        wall_thickness = 0.1
        
        # 添加四面墙壁
        walls = [
            ("wall_front", (workspace_x, 0, 0), (wall_thickness, workspace_y*2, workspace_z*2)),
            ("wall_back", (-workspace_x, 0, 0), (wall_thickness, workspace_y*2, workspace_z*2)),
            ("wall_left", (0, workspace_y, 0), (workspace_x*2, wall_thickness, workspace_z*2)),
            ("wall_right", (0, -workspace_y, 0), (workspace_x*2, wall_thickness, workspace_z*2)),
        ]
        
        for name, pos, size in walls:
            wall_pose = PoseStamped()
            wall_pose.header.frame_id = "world"
            wall_pose.pose.position.x = pos[0]
            wall_pose.pose.position.y = pos[1]
            wall_pose.pose.position.z = workspace_z/2 + self.ground_height
            wall_pose.pose.orientation.w = 1.0
            
            self.scene.add_box(name, wall_pose, size)
            rospy.sleep(0.1)
            
        rospy.loginfo("✅ 工作空间边界添加完成")
        
    def remove_all_objects(self):
        """移除所有碰撞对象"""
        rospy.loginfo("移除所有碰撞对象...")
        object_names = self.scene.get_known_object_names()
        for name in object_names:
            self.scene.remove_world_object(name)
            rospy.sleep(0.1)
        rospy.loginfo("✅ 所有碰撞对象已移除")
        
    def _wait_for_object(self, object_name, timeout=5.0):
        """等待对象添加到场景"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if object_name in self.scene.get_known_object_names():
                return True
            rospy.sleep(0.1)
        return False
        
    def print_scene_info(self):
        """打印当前场景信息"""
        rospy.loginfo("=== 当前碰撞场景信息 ===")
        objects = self.scene.get_known_object_names()
        rospy.loginfo(f"场景中的碰撞对象: {objects}")
        rospy.loginfo(f"对象数量: {len(objects)}")
        
    def run(self):
        """主运行函数"""
        # 清理旧的碰撞对象
        if rospy.get_param('~clear_scene', True):
            self.remove_all_objects()
            
        # 添加地面
        self.add_ground_plane()
        
        # 可选：添加工作空间边界
        if rospy.get_param('~add_boundaries', False):
            self.add_workspace_boundaries()
            
        # 打印场景信息
        self.print_scene_info()
        
        rospy.loginfo("碰撞场景设置完成，节点保持运行...")
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = CollisionSceneManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass