#!/usr/bin/env python
# -*- coding: utf-8 -*-

def explain_collision_geometry():
    """解释为什么使用简单几何体而不是STL"""
    
    print("\n" + "="*80)
    print("碰撞几何体选择：STL vs 简单几何体")
    print("="*80)
    
    print("\n1. STL网格碰撞检测的问题：")
    print("   ❌ 原始STL文件已损坏（声称有8亿个三角形）")
    print("   ❌ 计算复杂度高：O(n×m)")
    print("   ❌ 每次规划需要检查数千次碰撞")
    print("   ❌ 可能导致规划超时")
    
    print("\n2. 简单几何体的优势：")
    print("   ✅ 计算速度快100倍以上")
    print("   ✅ 数学公式简单可靠")
    print("   ✅ 易于调试和可视化")
    print("   ✅ 提供合理的安全边距")
    
    print("\n3. 实际对比：")
    print("   lifting_Link:")
    print("     STL: 2.5KB文件，格式错误")
    print("     简单: 圆柱体 r=0.04m, h=0.56m")
    print("   ")
    print("   计算一次碰撞检测：")
    print("     STL: ~10-100ms（如果能工作）")
    print("     圆柱体: <0.1ms")
    
    print("\n4. MoveIt最佳实践：")
    print("   - 视觉模型(Visual): 使用详细STL，用于显示")
    print("   - 碰撞模型(Collision): 使用简单几何体，用于规划")
    
    print("\n5. 安全性考虑：")
    print("   碰撞模型应该：")
    print("   - 比实际物体稍大（提供安全边距）")
    print("   - 覆盖所有关键部分")
    print("   - 快速可靠地检测碰撞")
    
    print("\n" + "="*80)
    print("结论：使用简单几何体是正确选择！")
    print("="*80)

if __name__ == '__main__':
    explain_collision_geometry()