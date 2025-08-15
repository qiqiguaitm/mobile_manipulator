#!/usr/bin/env python3
"""
验证机械臂关节限制配置
检查URDF和MoveIt配置中的关节限制是否正确更新
"""

import rospy
import math
from urdf_parser_py.urdf import URDF
import yaml
import sys

class JointLimitsVerifier:
    def __init__(self):
        # 预期的关节限制参数（单位：弧度）
        self.expected_limits = {
            'joint1': {
                'min_position': -2.687,  # -154°
                'max_position': 2.687,   # +154°
                'max_velocity': 3.142    # 180°/s
            },
            'joint2': {
                'min_position': 0.0,     # 0°
                'max_position': 3.403,   # 195°
                'max_velocity': 3.403    # 195°/s
            },
            'joint3': {
                'min_position': -3.054,  # -175°
                'max_position': 0.0,     # 0°
                'max_velocity': 3.142    # 180°/s
            },
            'joint4': {
                'min_position': -1.780,  # -102°
                'max_position': 1.780,   # +102°
                'max_velocity': 3.927    # 225°/s
            },
            'joint5': {
                'min_position': -1.309,  # -75°
                'max_position': 1.309,   # +75°
                'max_velocity': 3.927    # 225°/s
            },
            'joint6': {
                'min_position': -2.094,  # -120°
                'max_position': 2.094,   # +120°
                'max_velocity': 3.927    # 225°/s
            }
        }
        
    def rad_to_deg(self, rad):
        """弧度转角度"""
        return math.degrees(rad)
        
    def check_urdf_from_param(self):
        """从ROS参数服务器检查URDF配置"""
        print("\n=== 检查URDF配置 ===")
        try:
            # 从参数服务器获取robot_description
            robot_description = rospy.get_param('robot_description')
            robot = URDF.from_xml_string(robot_description)
            
            print(f"机器人名称: {robot.name}")
            print(f"关节总数: {len(robot.joints)}")
            
            # 检查每个关节的限制
            for joint_name, expected in self.expected_limits.items():
                joint = next((j for j in robot.joints if j.name == joint_name), None)
                if joint and joint.limit:
                    print(f"\n{joint_name}:")
                    print(f"  位置范围: [{joint.limit.lower:.3f}, {joint.limit.upper:.3f}] rad")
                    print(f"            [{self.rad_to_deg(joint.limit.lower):.1f}°, {self.rad_to_deg(joint.limit.upper):.1f}°]")
                    print(f"  最大速度: {joint.limit.velocity:.3f} rad/s ({self.rad_to_deg(joint.limit.velocity):.1f}°/s)")
                    
                    # 验证是否匹配预期值
                    if abs(joint.limit.lower - expected['min_position']) > 0.001:
                        print(f"  ⚠️  最小位置不匹配! 期望: {expected['min_position']:.3f}")
                    if abs(joint.limit.upper - expected['max_position']) > 0.001:
                        print(f"  ⚠️  最大位置不匹配! 期望: {expected['max_position']:.3f}")
                    if abs(joint.limit.velocity - expected['max_velocity']) > 0.001:
                        print(f"  ⚠️  最大速度不匹配! 期望: {expected['max_velocity']:.3f}")
                else:
                    print(f"\n⚠️  找不到关节: {joint_name}")
                    
        except KeyError:
            print("⚠️  未找到robot_description参数，请先加载URDF到参数服务器")
            return False
        except Exception as e:
            print(f"❌ 检查URDF时出错: {e}")
            return False
        return True
        
    def check_moveit_config(self, config_file):
        """检查MoveIt配置文件"""
        print(f"\n=== 检查MoveIt配置: {config_file} ===")
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            print(f"默认速度缩放因子: {config.get('default_velocity_scaling_factor', 'N/A')}")
            print(f"默认加速度缩放因子: {config.get('default_acceleration_scaling_factor', 'N/A')}")
            
            joint_limits = config.get('joint_limits', {})
            
            for joint_name, expected in self.expected_limits.items():
                if joint_name in joint_limits:
                    limits = joint_limits[joint_name]
                    print(f"\n{joint_name}:")
                    
                    if 'max_position' in limits and 'min_position' in limits:
                        print(f"  位置范围: [{limits['min_position']:.3f}, {limits['max_position']:.3f}] rad")
                        print(f"            [{self.rad_to_deg(limits['min_position']):.1f}°, {self.rad_to_deg(limits['max_position']):.1f}°]")
                        
                        # 验证是否匹配预期值
                        if abs(limits['min_position'] - expected['min_position']) > 0.001:
                            print(f"  ⚠️  最小位置不匹配! 期望: {expected['min_position']:.3f}")
                        if abs(limits['max_position'] - expected['max_position']) > 0.001:
                            print(f"  ⚠️  最大位置不匹配! 期望: {expected['max_position']:.3f}")
                    
                    if 'max_velocity' in limits:
                        print(f"  最大速度: {limits['max_velocity']:.3f} rad/s ({self.rad_to_deg(limits['max_velocity']):.1f}°/s)")
                        
                        # 验证是否匹配预期值
                        if abs(limits['max_velocity'] - expected['max_velocity']) > 0.001:
                            print(f"  ⚠️  最大速度不匹配! 期望: {expected['max_velocity']:.3f}")
                else:
                    print(f"\n⚠️  配置中找不到关节: {joint_name}")
                    
        except FileNotFoundError:
            print(f"❌ 找不到配置文件: {config_file}")
            return False
        except Exception as e:
            print(f"❌ 检查MoveIt配置时出错: {e}")
            return False
        return True

def main():
    rospy.init_node('joint_limits_verifier', anonymous=True)
    
    verifier = JointLimitsVerifier()
    
    # 检查URDF（从参数服务器）
    verifier.check_urdf_from_param()
    
    # 检查MoveIt配置文件
    moveit_configs = [
        "/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_with_gripper/config/joint_limits.yaml",
        "/home/agilex/MobileManipulator/src/arm_planner/moveit_config/piper_no_gripper/config/joint_limits.yaml"
    ]
    
    for config_file in moveit_configs:
        verifier.check_moveit_config(config_file)
    
    print("\n=== 验证完成 ===")
    print("\n提示: 如果看到'未找到robot_description参数'的警告，")
    print("请先运行机械臂的launch文件来加载URDF到参数服务器。")

if __name__ == '__main__':
    main()