#!/usr/bin/env python3
"""
Piper机械臂使能状态修复脚本
解决disable后无法重新enable的问题
"""

import rospy
from arm_planner.srv import Enable, EnableResponse
from std_srvs.srv import Empty, Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import sys

class PiperEnableFixer:
    def __init__(self):
        rospy.init_node('piper_enable_fixer', anonymous=True)
        
        print("="*60)
        print("Piper机械臂使能修复工具")
        print("="*60)
        
        # 订阅关节状态来检查机械臂状态
        self.joint_states = None
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        
        # 等待一下获取状态
        rospy.sleep(1.0)
        
    def joint_callback(self, msg):
        self.joint_states = msg
        
    def check_arm_status(self):
        """检查机械臂当前状态"""
        print("\n检查机械臂状态...")
        
        # 检查关节状态
        if self.joint_states is None:
            print("✗ 无法获取关节状态")
            return False
        else:
            print(f"✓ 获取到{len(self.joint_states.position)}个关节数据")
            
            # 检查速度是否为0（可能表示已禁用）
            if all(v == 0 for v in self.joint_states.velocity):
                print("⚠ 所有关节速度为0，可能已禁用")
            else:
                print("✓ 关节正在运动")
                
        return True
        
    def force_enable_sequence(self):
        """强制执行使能序列"""
        print("\n执行强制使能序列...")
        
        # 1. 先尝试停止
        print("1. 发送停止命令...")
        try:
            stop_srv = rospy.ServiceProxy('/stop_srv', Trigger)
            stop_srv.wait_for_service(timeout=2.0)
            resp = stop_srv()
            print(f"   停止响应: {resp.message}")
        except Exception as e:
            print(f"   停止失败: {e}")
            
        time.sleep(1.0)
        
        # 2. 重置紧急停止
        print("2. 重置紧急停止...")
        try:
            reset_estop = rospy.ServiceProxy('/reset_emergency_stop', Empty)
            reset_estop.wait_for_service(timeout=2.0)
            reset_estop()
            print("   ✓ 紧急停止已重置")
        except:
            print("   ⚠ 无紧急停止服务")
            
        time.sleep(0.5)
        
        # 3. 尝试使能 - 多次尝试
        print("3. 尝试使能机械臂...")
        success = False
        
        for attempt in range(3):
            print(f"\n   尝试 {attempt + 1}/3...")
            try:
                enable_srv = rospy.ServiceProxy('enable_srv', Enable)
                enable_srv.wait_for_service(timeout=2.0)
                
                # 发送使能请求
                response = enable_srv(enable_request=True)
                
                if response.enable_response:
                    print("   ✓ 使能成功！")
                    success = True
                    break
                else:
                    print("   ✗ 使能失败，等待后重试...")
                    time.sleep(2.0)
                    
            except Exception as e:
                print(f"   错误: {e}")
                time.sleep(1.0)
                
        return success
        
    def direct_enable_publish(self):
        """直接发布使能消息（备用方案）"""
        print("\n尝试直接发布使能消息...")
        
        try:
            # 创建使能消息发布器
            enable_pub = rospy.Publisher('/enable_flag', Bool, queue_size=1)
            rospy.sleep(0.5)  # 等待发布器建立
            
            # 发布使能消息
            enable_msg = Bool()
            enable_msg.data = True
            
            for _ in range(5):  # 发布多次确保接收
                enable_pub.publish(enable_msg)
                rospy.sleep(0.1)
                
            print("   ✓ 已发布使能消息")
            return True
            
        except Exception as e:
            print(f"   ✗ 发布失败: {e}")
            return False
            
    def run_recovery(self):
        """运行完整的恢复流程"""
        
        # 1. 检查状态
        if not self.check_arm_status():
            print("\n⚠ 无法获取机械臂状态，但继续尝试恢复...")
            
        # 2. 强制使能序列
        if self.force_enable_sequence():
            print("\n✅ 使能恢复成功！")
            print("\n建议：")
            print("- 检查机械臂是否响应控制命令")
            print("- 先进行小幅度测试运动")
            print("- 如果仍有问题，可能需要重启驱动节点")
            return True
            
        # 3. 如果服务方式失败，尝试直接发布
        print("\n标准使能失败，尝试备用方案...")
        if self.direct_enable_publish():
            time.sleep(2.0)
            
            # 再次尝试标准使能
            print("\n再次尝试标准使能...")
            if self.force_enable_sequence():
                print("\n✅ 使能恢复成功！")
                return True
                
        # 4. 所有方法都失败
        print("\n❌ 使能恢复失败！")
        print("\n可能的解决方案：")
        print("1. 重启机械臂驱动节点：")
        print("   rosnode kill /piper_driver")
        print("   roslaunch arm_controller start_single_piper.launch")
        print("\n2. 检查CAN通信：")
        print("   candump can0")
        print("\n3. 检查硬件急停按钮是否按下")
        print("\n4. 断电重启机械臂")
        
        return False

def main():
    try:
        fixer = PiperEnableFixer()
        
        # 询问用户
        print("\n是否立即开始修复？(y/n): ", end='')
        choice = input().strip().lower()
        
        if choice == 'y':
            fixer.run_recovery()
        else:
            print("已取消")
            
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n用户中断")

if __name__ == '__main__':
    main()