#!/usr/bin/env python3
import rospy
import moveit_commander
import sys

def test_arm_states():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_arm_states', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("\n=== 测试 arm 组的预定义状态 ===")
    print(f"当前关节值: {arm_group.get_current_joint_values()}")
    print(f"关节名称: {arm_group.get_active_joints()}")
    
    # 获取所有命名状态
    named_targets = arm_group.get_named_targets()
    print(f"\n可用的命名状态: {named_targets}")
    
    # 测试每个状态
    for state_name in ['zero', 'ready', 'gripper_open', 'gripper_close']:
        if state_name in named_targets:
            print(f"\n--- 测试 '{state_name}' 状态 ---")
            try:
                arm_group.set_named_target(state_name)
                target_values = arm_group.get_joint_value_target()
                print(f"目标关节值: {target_values}")
                
                # 尝试规划
                plan = arm_group.plan()
                if plan[0]:
                    print(f"✅ '{state_name}' 状态规划成功")
                else:
                    print(f"❌ '{state_name}' 状态规划失败")
                    
            except Exception as e:
                print(f"❌ 设置 '{state_name}' 状态时出错: {e}")
        else:
            print(f"\n⚠️  '{state_name}' 状态未定义")
    
    # 测试直接设置关节值
    print("\n\n=== 测试直接设置关节值 ===")
    test_configs = {
        "全零位置": [0, 0, 0, 0, 0, 0, 0, 0],
        "零位置但夹爪关闭": [0, 0, 0, 0, 0, 0, 0.025, -0.025],
        "ready位置": [0, 1.57, -1.57, 0, 0, 0, 0, 0]
    }
    
    for name, values in test_configs.items():
        print(f"\n--- 测试 {name}: {values} ---")
        try:
            arm_group.set_joint_value_target(values)
            plan = arm_group.plan()
            if plan[0]:
                print(f"✅ 规划成功")
            else:
                print(f"❌ 规划失败")
        except Exception as e:
            print(f"❌ 出错: {e}")
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_arm_states()
    except rospy.ROSInterruptException:
        pass