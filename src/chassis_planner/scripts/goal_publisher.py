#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher', anonymous=True)
        
        # 创建目标点发布者
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 等待发布者连接
        rospy.sleep(1.0)
        
        # 预定义的目标点列表 (x, y, yaw_degrees)
        self.goals = [
            (1.0, 0.0, 0),      # 目标点1: 前进1米
            (1.0, 1.0, 90),     # 目标点2: 右转到(1,1)
            (0.0, 1.0, 180),    # 目标点3: 左转到(0,1) 
            (0.0, 0.0, 270),    # 目标点4: 回到原点
        ]
        
        self.current_goal_index = 0
        
        rospy.loginfo("Goal Publisher initialized. Ready to publish goals.")
        rospy.loginfo("Available goals:")
        for i, (x, y, yaw) in enumerate(self.goals):
            rospy.loginfo(f"  Goal {i+1}: x={x}, y={y}, yaw={yaw}°")
    
    def create_goal_msg(self, x, y, yaw_degrees):
        """创建目标点消息"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # 将角度转换为四元数
        yaw_radians = math.radians(yaw_degrees)
        q = quaternion_from_euler(0, 0, yaw_radians)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        return goal
    
    def publish_goal(self, x, y, yaw_degrees):
        """发布目标点"""
        goal_msg = self.create_goal_msg(x, y, yaw_degrees)
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Published goal: x={x}, y={y}, yaw={yaw_degrees}°")
    
    def publish_next_goal(self):
        """发布下一个目标点"""
        if self.current_goal_index < len(self.goals):
            x, y, yaw = self.goals[self.current_goal_index]
            self.publish_goal(x, y, yaw)
            self.current_goal_index += 1
        else:
            rospy.loginfo("All goals published. Resetting to first goal.")
            self.current_goal_index = 0
    
    def publish_all_goals_sequence(self, interval=10.0):
        """按顺序发布所有目标点"""
        rospy.loginfo(f"Starting goal sequence with {interval}s intervals...")
        
        for i, (x, y, yaw) in enumerate(self.goals):
            rospy.loginfo(f"Publishing goal {i+1}/{len(self.goals)}")
            self.publish_goal(x, y, yaw)
            
            if i < len(self.goals) - 1:  # 不在最后一个目标后等待
                rospy.sleep(interval)
        
        rospy.loginfo("Goal sequence completed!")
    
    def run_interactive_mode(self):
        """交互模式"""
        rospy.loginfo("=== Interactive Goal Publisher ===")
        rospy.loginfo("Commands:")
        rospy.loginfo("  'n' - Publish next goal")
        rospy.loginfo("  'a' - Publish all goals in sequence")
        rospy.loginfo("  'c' - Publish custom goal")
        rospy.loginfo("  'r' - Reset goal index")
        rospy.loginfo("  'q' - Quit")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd == 'n':
                    self.publish_next_goal()
                elif cmd == 'a':
                    interval = float(input("Enter interval between goals (seconds, default 10): ") or "10")
                    self.publish_all_goals_sequence(interval)
                elif cmd == 'c':
                    try:
                        x = float(input("Enter x coordinate: "))
                        y = float(input("Enter y coordinate: "))
                        yaw = float(input("Enter yaw angle (degrees, default 0): ") or "0")
                        self.publish_goal(x, y, yaw)
                    except ValueError:
                        rospy.logwarn("Invalid input. Please enter numeric values.")
                elif cmd == 'r':
                    self.current_goal_index = 0
                    rospy.loginfo("Goal index reset to 0")
                elif cmd == 'q':
                    rospy.loginfo("Exiting...")
                    break
                else:
                    rospy.logwarn("Unknown command. Use 'n', 'a', 'c', 'r', or 'q'")
                    
            except KeyboardInterrupt:
                rospy.loginfo("Interrupted by user")
                break
            except EOFError:
                rospy.loginfo("EOF received, exiting...")
                break

def main():
    try:
        publisher = GoalPublisher()
        
        # 检查启动参数
        mode = rospy.get_param('~mode', 'interactive')
        
        if mode == 'interactive':
            publisher.run_interactive_mode()
        elif mode == 'sequence':
            interval = rospy.get_param('~interval', 10.0)
            publisher.publish_all_goals_sequence(interval)
        elif mode == 'single':
            x = rospy.get_param('~x', 1.0)
            y = rospy.get_param('~y', 0.0)
            yaw = rospy.get_param('~yaw', 0.0)
            publisher.publish_goal(x, y, yaw)
        else:
            rospy.logwarn(f"Unknown mode: {mode}. Using interactive mode.")
            publisher.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal publisher interrupted")

if __name__ == '__main__':
    main()