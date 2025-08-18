#!/usr/bin/env python3
"""
CAN通信测试 - 直接测试CAN通信
"""
import subprocess
import time

def test_can():
    print("=== CAN通信诊断 ===\n")
    
    # 1. 检查CAN接口
    print("1. 检查CAN接口列表")
    result = subprocess.run(['ip', 'link', 'show', 'type', 'can'], 
                          capture_output=True, text=True)
    print(result.stdout)
    
    # 2. 使用candump监听CAN数据
    print("\n2. 监听CAN数据 (5秒)...")
    print("   如果没有数据，说明机械臂未发送或CAN连接有问题")
    
    proc = subprocess.Popen(['candump', 'can0', '-n', '10'], 
                           stdout=subprocess.PIPE, 
                           stderr=subprocess.PIPE,
                           text=True)
    
    try:
        stdout, stderr = proc.communicate(timeout=5)
        if stdout:
            print("   接收到CAN数据:")
            print(stdout)
        else:
            print("   ✗ 未接收到任何CAN数据")
            print("   可能原因:")
            print("   - 机械臂未上电")
            print("   - CAN线连接错误") 
            print("   - 终端电阻问题")
            print("   - 波特率不匹配")
    except subprocess.TimeoutExpired:
        proc.kill()
        print("   ✗ 5秒内未接收到CAN数据")
    
    # 3. 测试发送CAN命令
    print("\n3. 尝试发送测试CAN帧...")
    # 发送一个查询状态的CAN帧（假设是标准的CANopen NMT命令）
    result = subprocess.run(['cansend', 'can0', '000#0100'], 
                          capture_output=True, text=True)
    if result.returncode == 0:
        print("   ✓ CAN发送成功")
    else:
        print("   ✗ CAN发送失败:", result.stderr)
    
    # 4. 检查CAN错误
    print("\n4. 检查CAN错误计数")
    result = subprocess.run(['ip', '-s', 'link', 'show', 'can0'], 
                          capture_output=True, text=True)
    lines = result.stdout.split('\n')
    for i, line in enumerate(lines):
        if 'RX:' in line and i+1 < len(lines):
            print(f"   接收: {lines[i+1].strip()}")
        elif 'TX:' in line and i+1 < len(lines):
            print(f"   发送: {lines[i+1].strip()}")
    
    print("\n=== 建议 ===")
    print("1. 检查机械臂电源是否开启")
    print("2. 检查CAN线是否正确连接（CAN_H, CAN_L）")
    print("3. 确认终端电阻（120Ω）是否正确")
    print("4. 尝试降低波特率: sudo ip link set can0 type can bitrate 500000")

if __name__ == "__main__":
    test_can()