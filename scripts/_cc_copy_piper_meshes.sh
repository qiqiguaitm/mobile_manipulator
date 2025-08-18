#!/bin/bash

# 脚本：复制所有piper网格文件到mobile_manipulator2_description，并添加piper_前缀

set -e

# 源目录和目标目录
SRC_DIR="/home/agilex/MobileManipulator/src/robot_desc/piper_description/meshes"
DST_DIR="/home/agilex/MobileManipulator/src/robot_desc/mobile_manipulator2_description/meshes"

# 创建目标目录结构
mkdir -p "${DST_DIR}/visual/dae"
mkdir -p "${DST_DIR}/visual/stl"
mkdir -p "${DST_DIR}/collision"

echo "开始复制piper网格文件..."

# 复制DAE视觉网格文件
echo "复制DAE视觉网格..."
for file in ${SRC_DIR}/dae/*.dae; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        cp -v "$file" "${DST_DIR}/visual/dae/piper_${filename}"
    fi
done

# 复制根目录下的DAE文件
for file in ${SRC_DIR}/*.dae; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        cp -v "$file" "${DST_DIR}/visual/dae/piper_${filename}"
    fi
done

# 复制STL视觉网格文件（根目录下的STL作为视觉模型）
echo "复制STL视觉网格..."
for file in ${SRC_DIR}/*.STL; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        cp -v "$file" "${DST_DIR}/visual/stl/piper_${filename}"
    fi
done

# 复制碰撞网格文件
echo "复制碰撞网格..."
for file in ${SRC_DIR}/collision/*_collision.STL; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        # 保持_collision后缀，但添加piper_前缀
        cp -v "$file" "${DST_DIR}/collision/piper_${filename}"
    fi
done

echo "网格文件复制完成！"
echo ""
echo "文件组织结构："
echo "- 视觉网格 DAE: ${DST_DIR}/visual/dae/piper_*.dae"
echo "- 视觉网格 STL: ${DST_DIR}/visual/stl/piper_*.STL"
echo "- 碰撞网格: ${DST_DIR}/collision/piper_*_collision.STL"