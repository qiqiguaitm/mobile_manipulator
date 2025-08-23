#!/bin/bash

# 环境准备脚本 - 为 det_online.py 配置依赖环境
# Author: Claude Code
# Date: $(date)

set -e  # 遇到错误立即退出

echo "=== 开始配置 det_online.py 运行环境 ==="

# 检查Python版本
echo "检查Python环境..."
python3 --version
which python3

# 更新pip
echo "更新pip..."
pip3 install --upgrade pip

# 安装基础科学计算包
echo "安装基础科学计算包..."
pip3 install numpy opencv-python requests

# 安装图像处理相关包
echo "安装图像处理包..."
pip3 install Pillow pycocotools

# 安装深度学习框架相关
echo "安装mmengine..."
pip3 install mmengine

# 安装可视化工具
echo "安装supervision..."
pip3 install supervision

# 安装DDS Cloud API SDK及其依赖
echo "安装DDS Cloud API SDK及其依赖..."
pip3 install flask dds-cloudapi-sdk

# 验证关键包安装
echo "=== 验证依赖包安装情况 ==="
python3 -c "
import sys
packages = [
    'cv2', 'numpy', 'requests', 'PIL', 
    'mmengine', 'supervision', 'pycocotools',
    'dds_cloudapi_sdk'
]

failed = []
for pkg in packages:
    try:
        __import__(pkg)
        print(f'✅ {pkg}')
    except ImportError as e:
        print(f'❌ {pkg}: {e}')
        failed.append(pkg)

if failed:
    print(f'\n失败的包: {failed}')
    sys.exit(1)
else:
    print('\n✅ 所有依赖包安装成功!')
"
