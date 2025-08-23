#!/bin/bash
# 查看3D抓取信息的便捷脚本

echo "========================================="
echo "     3D Grasp Visualization Viewer      "
echo "========================================="
echo ""
echo "选择查看方式："
echo "1) 实时监控器（详细信息）"
echo "2) 原始3D数据"
echo "3) 可视化标记数据"
echo "4) 简单统计"
echo ""
read -p "请选择 (1-4): " choice

case $choice in
    1)
        echo "启动实时监控器..."
        python3 /home/agilex/MobileManipulator/scripts/_cc_monitor_3d_grasp.py
        ;;
    2)
        echo "显示原始3D数据..."
        rostopic echo /perception/hand/grasps_3d
        ;;
    3)
        echo "显示可视化标记..."
        rostopic echo /perception/grasp_3d_markers
        ;;
    4)
        echo "显示简单统计..."
        watch -n 1 "rostopic list | grep grasp | xargs -I {} sh -c 'echo -n \"{}: \"; rostopic hz {} 2>/dev/null | head -1'"
        ;;
    *)
        echo "无效选择"
        ;;
esac
