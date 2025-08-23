#!/bin/bash
# 修复libffi冲突并启动感知节点

echo "=== 启动感知节点（修复libffi冲突）==="

# 使用系统的libffi.so.7避免版本冲突
export LD_PRELOAD=/lib/aarch64-linux-gnu/libffi.so.7

echo "Python路径: $(which python3)"
echo "Python版本: $(python3 --version)"
echo "LD_PRELOAD: $LD_PRELOAD"

# 启动感知节点
roslaunch perception perception_dino_test.launch