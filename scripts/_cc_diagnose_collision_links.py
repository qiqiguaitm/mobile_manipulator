#!/usr/bin/env python3
"""
诊断具体的碰撞link
"""

import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
import sys

def get_collision_info():
    """获取当前的碰撞信息"""
    try:
        # 获取规划场景
        get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        
        # 请求包含允许碰撞矩阵的完整场景
        req = moveit_msgs.srv.GetPlanningSceneRequest()
        req.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS |
            PlanningSceneComponents.ROBOT_STATE |
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.TRANSFORMS |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        
        scene = get_scene(req).scene
        
        print("=== 允许碰撞矩阵分析 ===")
        acm = scene.allowed_collision_matrix
        
        # 查找关键link
        key_links = ['link1', 'link2', 'link3', 'lifting_Link', 'box_Link', 'base_link']
        
        print(f"\n矩阵包含 {len(acm.entry_names)} 个link")
        
        # 检查关键碰撞对
        print("\n关键碰撞对状态:")
        for i, link1 in enumerate(key_links):
            for j, link2 in enumerate(key_links):
                if i >= j:
                    continue
                    
                try:
                    idx1 = acm.entry_names.index(link1)
                    idx2 = acm.entry_names.index(link2)
                    
                    # 检查是否允许碰撞
                    allowed = False
                    if idx1 < len(acm.entry_values) and idx2 < len(acm.entry_values[idx1].enabled):
                        allowed = acm.entry_values[idx1].enabled[idx2]
                    
                    status = "✓ 允许" if allowed else "✗ 检测"
                    print(f"  {link1:<15} <-> {link2:<15}: {status}")
                    
                except ValueError:
                    print(f"  {link1:<15} <-> {link2:<15}: ? 未找到")
                    
    except Exception as e:
        print(f"错误: {e}")

def test_specific_states():
    """测试特定状态"""
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('diagnose_collision_links', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    print("\n=== 测试特定关节配置 ===")
    
    # 测试配置
    test_configs = [
        ("current", None),
        ("zero", [0, 0, 0, 0, 0, 0, 0, 0]),
        ("slightly_raised", [0, 0.1, -0.1, 0, 0, 0, 0, 0]),  # 稍微抬起
        ("more_raised", [0, 0.3, -0.3, 0, 0, 0, 0, 0]),    # 更多抬起
        ("ready", [0, 1.57, -1.57, 0, 0, 0, 0, 0]),
    ]
    
    for name, joint_values in test_configs:
        print(f"\n测试 {name}:")
        
        if joint_values:
            # 设置关节值
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 
                          'joint5', 'joint6', 'joint7', 'joint8']
            joint_dict = dict(zip(joint_names, joint_values))
            
            try:
                arm_group.set_joint_value_target(joint_dict)
                
                # 检查目标是否有效
                current_state = robot.get_current_state()
                validity_req = moveit_msgs.srv.GetStateValidityRequest()
                validity_req.robot_state = current_state
                validity_req.group_name = "arm"
                
                # 更新状态到目标值
                for i, jname in enumerate(joint_names):
                    if jname in current_state.joint_state.name:
                        idx = current_state.joint_state.name.index(jname)
                        current_state.joint_state.position = list(current_state.joint_state.position)
                        current_state.joint_state.position[idx] = joint_values[i]
                
                # 检查有效性
                validity_srv = rospy.ServiceProxy('/check_state_validity', 
                                                moveit_msgs.srv.GetStateValidity)
                                                
                resp = validity_srv(validity_req)
                
                if resp.valid:
                    print(f"  ✓ 配置有效")
                else:
                    print(f"  ✗ 配置无效 - {len(resp.contacts)}个碰撞")
                    for contact in resp.contacts[:3]:
                        print(f"    - {contact.contact_body_1} <-> {contact.contact_body_2}")
                        
            except Exception as e:
                print(f"  错误: {e}")
        else:
            print("  使用当前状态")
    
    moveit_commander.roscpp_shutdown()

def main():
    print("=== 碰撞Link诊断工具 ===\n")
    
    # 先获取碰撞矩阵信息
    get_collision_info()
    
    # 然后测试特定状态
    print("\n" + "="*50 + "\n")
    test_specific_states()
    
    print("\n=== 可能的解决方案 ===")
    print("1. 检查SRDF是否正确加载")
    print("2. 重启move_group节点")
    print("3. 检查碰撞模型是否仍然过大")
    print("4. 考虑进一步减小collision_padding值")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass