#!/bin/bash

# Piper机械臂CAN权限设置脚本
# 这个脚本需要在启动MoveIt之前运行

echo "设置Piper机械臂CAN权限..."

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo运行此脚本: sudo bash setup_can_permissions.sh"
    exit 1
fi

# 安装必要的工具
echo "检查并安装必要的工具..."
apt update
apt install -y can-utils ethtool

# 设置can用户组权限
groupadd -f can
usermod -a -G can $SUDO_USER

# 设置udev规则以自动处理CAN设备权限
cat > /etc/udev/rules.d/99-can.rules << 'EOF'
# CAN设备权限规则
SUBSYSTEM=="net", KERNEL=="can*", GROUP="can", MODE="0664"
EOF

# 重新加载udev规则
udevadm control --reload-rules
udevadm trigger

echo "CAN权限设置完成！"
echo "请重新登录或运行 'newgrp can' 来应用用户组权限。"