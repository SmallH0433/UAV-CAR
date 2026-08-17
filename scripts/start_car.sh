#!/bin/bash
# R680 小车实机链路启动脚本（由 systemd car-bringup.service 调用，也可手动执行）
#
# 要点：开机后 USB 串口设备枚举可能晚于启动（实测曾晚约 1 分钟），
# 必须先等 udev 符号链接就绪再 launch，否则雷达/电机节点会因找不到设备退出。

# 注意：不要用 set -u —— ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等）

# 启动前等待的设备（udev 规则固定名，见 config/99-car-devices.rules）
# K210 前摄当前未用（front_camera:=none），不在等待列表
DEVICES=(/dev/wheeltec /dev/wheeltec_lidar /dev/wheeltec_gps)
TIMEOUT=60  # 最长等待秒数；超时后照常启动（各节点对缺设备有重试/容错）

waited=0
while [ "$waited" -lt "$TIMEOUT" ]; do
    missing=0
    for dev in "${DEVICES[@]}"; do
        [ -e "$dev" ] || missing=1
    done
    [ "$missing" -eq 0 ] && break
    sleep 2
    waited=$((waited + 2))
done

if [ "$waited" -ge "$TIMEOUT" ]; then
    echo "[start_car] 等待设备超时（${TIMEOUT}s 内未全部就绪），仍尝试启动" >&2
else
    echo "[start_car] 设备就绪（等待 ${waited}s），启动 ROS 链路"
fi

source /opt/ros/humble/setup.bash
source /home/yahboom/CAR_ws/install/setup.bash

# UVC 免驱摄像头（仅画面采集显示，无视觉处理）：by-id 路径重启不变
CAMERA_DEVICE=/dev/v4l/by-id/usb-Generic_HD_camera_20201212000000-video-index0
CAMERA_ARGS=""
if [ -e "$CAMERA_DEVICE" ]; then
    CAMERA_ARGS="front_camera:=v4l2 camera_device:=$CAMERA_DEVICE"
else
    echo "[start_car] 未检测到摄像头，按无相机模式启动" >&2
    CAMERA_ARGS="front_camera:=none"
fi

exec ros2 launch car_sim real_bringup.launch.py \
    motor_port:=/dev/wheeltec $CAMERA_ARGS
