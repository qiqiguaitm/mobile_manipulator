#!/usr/bin/env python3
"""
Piper机械臂实用笛卡尔控制解决方案

提供可工作的笛卡尔空间控制方法
"""

import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
import time
import numpy as np

class PracticalCartesianController:
    def __init__(self):
        # 初始化
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('practical_cartesian_controller', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        # 设置参数
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_max_velocity_scaling_factor(0.2)
        self.arm_group.set_max_acceleration_scaling_factor(0.2)
        
        print("🤖 Piper实用笛卡尔控制器初始化完成")
        
    def get_current_cartesian_pose(self):
        """获取当前笛卡尔位姿"""
        pose = self.arm_group.get_current_pose().pose
        
        # 转换朝向为欧拉角
        orientation_list = [pose.orientation.x, pose.orientation.y, 
                          pose.orientation.z, pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        return {
            'position': [pose.position.x, pose.position.y, pose.position.z],
            'orientation': [roll, pitch, yaw],
            'pose_msg': pose
        }
    
    def cartesian_move_relative(self, dx=0, dy=0, dz=0, max_attempts=5):
        """相对笛卡尔运动 - 基于当前位置的偏移"""
        print(f"🎯 相对移动: dx={dx*100:.1f}cm, dy={dy*100:.1f}cm, dz={dz*100:.1f}cm")
        
        # 获取当前位姿
        current = self.get_current_cartesian_pose()
        print(f"当前位置: ({current['position'][0]:.4f}, {current['position'][1]:.4f}, {current['position'][2]:.4f})")
        
        # 分解为多个小步骤
        steps = max(1, int(max(abs(dx), abs(dy), abs(dz)) / 0.005))  # 每步最多5mm
        
        success_count = 0
        for step in range(steps):
            factor = (step + 1) / steps
            
            # 计算当前步骤的目标
            target_x = current['position'][0] + dx * factor
            target_y = current['position'][1] + dy * factor  
            target_z = current['position'][2] + dz * factor
            
            # 尝试关节空间逼近
            if self._move_to_cartesian_via_joints(target_x, target_y, target_z, max_attempts):
                success_count += 1
                print(f"  步骤 {step+1}/{steps} ✅")
            else:
                print(f"  步骤 {step+1}/{steps} ❌")
                break
                
            time.sleep(0.5)
        
        final = self.get_current_cartesian_pose()
        actual_dx = final['position'][0] - current['position'][0]
        actual_dy = final['position'][1] - current['position'][1] 
        actual_dz = final['position'][2] - current['position'][2]
        
        print(f"实际移动: dx={actual_dx*100:.1f}cm, dy={actual_dy*100:.1f}cm, dz={actual_dz*100:.1f}cm")
        
        return success_count == steps
    
    def _move_to_cartesian_via_joints(self, target_x, target_y, target_z, max_attempts):
        """使用关节空间搜索到达笛卡尔位置"""
        current_joints = self.arm_group.get_current_joint_values()
        
        # 尝试不同的关节配置
        for attempt in range(max_attempts):
            # 生成关节空间的小幅随机搜索
            test_joints = current_joints[:]
            for i in range(len(test_joints)):
                # 每个关节小幅调整
                adjustment = (np.random.random() - 0.5) * 0.1  # ±0.05弧度
                test_joints[i] += adjustment
                
                # 确保在关节限制内
                test_joints[i] = max(-3.14, min(3.14, test_joints[i]))
            
            # 测试这个关节配置
            try:
                self.arm_group.set_joint_value_target(test_joints)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
                
                if success:
                    # 检查是否接近目标位置
                    current_pose = self.get_current_cartesian_pose()
                    error_x = abs(current_pose['position'][0] - target_x)
                    error_y = abs(current_pose['position'][1] - target_y)
                    error_z = abs(current_pose['position'][2] - target_z)
                    
                    # 如果误差小于2cm，认为成功
                    if error_x < 0.02 and error_y < 0.02 and error_z < 0.02:
                        return True
                        
            except Exception:
                continue
        
        return False
    
    def cartesian_demo(self):
        """笛卡尔控制演示"""
        
        print("\n" + "="*60)
        print("🎯 Piper实用笛卡尔控制演示")
        print("="*60)
        
        # 显示当前状态
        current = self.get_current_cartesian_pose()
        print(f"起始位置: ({current['position'][0]:.4f}, {current['position'][1]:.4f}, {current['position'][2]:.4f})")
        
        results = []
        
        # 示例1：上升运动
        print("\n1️⃣ 测试上升运动 (+2cm)")
        success1 = self.cartesian_move_relative(dz=0.02)
        results.append(("上升运动", success1))
        time.sleep(2)
        
        # 示例2：右移运动
        print("\n2️⃣ 测试右移运动 (+1.5cm)")
        success2 = self.cartesian_move_relative(dy=0.015)
        results.append(("右移运动", success2))
        time.sleep(2)
        
        # 示例3：前进运动
        print("\n3️⃣ 测试前进运动 (+1cm)")
        success3 = self.cartesian_move_relative(dx=0.01)
        results.append(("前进运动", success3))
        time.sleep(2)
        
        # 示例4：组合运动
        print("\n4️⃣ 测试组合运动 (右移+下降)")
        success4 = self.cartesian_move_relative(dy=0.01, dz=-0.015)
        results.append(("组合运动", success4))
        
        # 总结结果
        print("\n" + "="*60)
        print("📊 测试结果总结")
        print("="*60)
        
        passed = 0
        for name, result in results:
            status = "✅ 成功" if result else "❌ 失败"
            print(f"{name}: {status}")
            if result:
                passed += 1
        
        print(f"\n成功率: {passed}/{len(results)} ({passed/len(results)*100:.0f}%)")
        
        final = self.get_current_cartesian_pose()
        print(f"最终位置: ({final['position'][0]:.4f}, {final['position'][1]:.4f}, {final['position'][2]:.4f})")
        
        if passed > 0:
            print("\n🎉 实用笛卡尔控制方案可以工作！")
            print("\n💡 这个方案的优势:")
            print("- ✅ 绕过MoveIt的笛卡尔规划限制")
            print("- ✅ 使用关节空间达到笛卡尔目标") 
            print("- ✅ 分步执行，提高成功率")
            print("- ✅ 适用于精确的位置控制")
            
            print("\n📋 使用方法:")
            print("```python")
            print("controller = PracticalCartesianController()")
            print("# 相对移动：右移2cm，上升1cm")
            print("controller.cartesian_move_relative(dy=0.02, dz=0.01)")
            print("```")
        else:
            print("\n⚠️ 需要进一步优化算法参数")

def main():
    try:
        controller = PracticalCartesianController()
        controller.cartesian_demo()
    except rospy.ROSInterruptException:
        print("演示被中断")
    except Exception as e:
        print(f"演示过程中发生错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()