#!/bin/bash
set -x
# RealSense Camera Reset Script
# 用于解决相机USB忙和启动慢的问题

set -e  # 遇到错误时退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否有sudo权限
check_sudo() {
    if [ "$EUID" -ne 0 ]; then 
        print_warn "脚本需要sudo权限来重置USB设备"
        exec sudo "$0" "$@"
    fi
}

# 杀掉所有相关进程
kill_processes() {
    print_info "停止所有RealSense相关进程..."
    
    # ROS节点
    rosnode kill -a 2>/dev/null || true
    
    # RealSense进程
    pkill -f realsense 2>/dev/null || true
    pkill -f rs- 2>/dev/null || true
    pkill -f RealSense 2>/dev/null || true
    
    # 可能占用相机的其他程序
    pkill -f cheese 2>/dev/null || true
    pkill -f motion 2>/dev/null || true
    
    # 等待进程完全退出
    sleep 2
    
    # 检查是否还有残留进程
    if pgrep -f realsense > /dev/null; then
        print_warn "强制终止残留进程..."
        pkill -9 -f realsense 2>/dev/null || true
        sleep 1
    fi
    
    print_info "进程清理完成"
}

# 查找Intel RealSense设备
find_realsense_devices() {
    local devices=$(lsusb | grep -i "Intel" | grep -E "RealSense|8086" || true)
    if [ -z "$devices" ]; then
        print_error "未找到RealSense相机"
        return 1
    fi
    echo "$devices"
}

# 重置USB设备
reset_usb_devices() {
    print_info "重置RealSense USB设备..."
    
    # 方法1: 使用usbreset命令
    if command -v usbreset &> /dev/null; then
        while IFS= read -r line; do
            if [ ! -z "$line" ]; then
                local bus=$(echo $line | sed 's/Bus \([0-9]*\).*/\1/')
                local device=$(echo $line | sed 's/.*Device \([0-9]*\).*/\1/')
                local vendor_product=$(echo $line | grep -o '[0-9a-f]*:[0-9a-f]*')
                
                print_info "重置设备: Bus $bus Device $device ID $vendor_product"
                
                # 尝试通过设备路径重置
                if [ -e "/dev/bus/usb/$bus/$device" ]; then
                    usbreset /dev/bus/usb/$bus/$device 2>/dev/null || \
                    usbreset $vendor_product 2>/dev/null || true
                fi
            fi
        done <<< "$(find_realsense_devices)"
    else
        print_warn "usbreset命令不可用，使用备用方法..."
        
        # 方法2: 通过sysfs重置
        for device in /sys/bus/usb/devices/*; do
            if [ -e "$device/idVendor" ]; then
                vendor=$(cat "$device/idVendor" 2>/dev/null || true)
                if [ "$vendor" = "8086" ]; then
                    print_info "重置设备: $device"
                    echo 0 > "$device/authorized" 2>/dev/null || true
                    sleep 1
                    echo 1 > "$device/authorized" 2>/dev/null || true
                fi
            fi
        done
    fi
    
    print_info "USB重置完成"
}

# 重新加载内核模块
reload_kernel_modules() {
    print_info "重新加载相机内核模块..."
    
    # 卸载模块
    modprobe -r uvcvideo 2>/dev/null || true
    modprobe -r videobuf2_v4l2 2>/dev/null || true
    modprobe -r videobuf2_core 2>/dev/null || true
    
    sleep 1
    
    # 重新加载模块
    modprobe uvcvideo
    modprobe videobuf2_v4l2 2>/dev/null || true
    modprobe videobuf2_core 2>/dev/null || true
    
    print_info "内核模块重新加载完成"
}

# 优化USB设置
optimize_usb() {
    print_info "优化USB设置..."
    
    # 增加USB缓冲区大小
    echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || true
    
    # 禁用USB自动暂停
    echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true
    
    # 对Intel设备禁用电源管理
    for i in /sys/bus/usb/devices/*/idVendor; do
        if grep -q "8086" "$i" 2>/dev/null; then
            echo on > $(dirname $i)/power/control 2>/dev/null || true
        fi
    done
    
    print_info "USB优化完成"
}

# 验证相机状态
verify_camera() {
    print_info "验证相机状态..."
    
    # 检查设备是否出现
    if command -v rs-enumerate-devices &> /dev/null; then
        local device_info=$(rs-enumerate-devices 2>/dev/null | grep -c "Device info" || true)
        if [ "$device_info" -gt 0 ]; then
            print_info "✓ 检测到 $device_info 个RealSense设备"
            
            # 显示设备详细信息
            rs-enumerate-devices -s 2>/dev/null | grep -E "Serial|Firmware|Name" || true
            return 0
        else
            print_warn "rs-enumerate-devices未检测到设备"
        fi
    fi
    
    # 备用检查方法
    local video_devices=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ "$video_devices" -gt 0 ]; then
        print_info "✓ 检测到 $video_devices 个video设备"
        return 0
    else
        print_error "未检测到相机设备"
        return 1
    fi
}

# 清理临时文件
cleanup_temp_files() {
    print_info "清理临时文件..."
    rm -f /tmp/realsense* 2>/dev/null || true
    rm -f /tmp/rs* 2>/dev/null || true
}

# 主函数
main() {
    print_info "========== RealSense相机重置脚本 =========="
    print_info "开始时间: $(date)"
    
    # 检查sudo权限
    check_sudo "$@"
    
    # 执行重置步骤
    kill_processes
    cleanup_temp_files
    reset_usb_devices
    sleep 2
    reload_kernel_modules
    sleep 1
    optimize_usb
    
    # 等待设备稳定
    print_info "等待设备稳定..."
    sleep 3
    
    # 验证结果
    if verify_camera; then
        print_info "========== 相机重置成功！=========="
        print_info "现在可以启动ROS节点了："
        print_info "  roslaunch realsense2_camera rs_camera.launch"
        exit 0
    else
        print_error "相机重置可能未完全成功，请检查连接"
        print_info "尝试以下操作："
        print_info "  1. 重新插拔USB线"
        print_info "  2. 更换USB端口(使用USB 3.0)"
        print_info "  3. 再次运行此脚本"
        exit 1
    fi
}

# 捕获Ctrl+C
trap 'print_warn "脚本被中断"; exit 1' INT

# 运行主函数
main "$@"
