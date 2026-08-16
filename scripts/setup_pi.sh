#!/usr/bin/env bash
# R680 小车树莓派 4B 一键环境配置（Ubuntu Server 22.04 arm64）
# 用法：bash scripts/setup_pi.sh
set -e

echo "==> 安装 ROS 2 Humble 与依赖"
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-serial \
  python3-numpy \
  python3-opencv \
  libpcl-dev \
  ros-humble-pcl-conversions \
  libpcap-dev \
  ros-humble-tf-transformations

echo "==> 串口权限（dialout 组）"
sudo usermod -aG dialout "$USER"

echo "==> 安装 udev 设备固定命名规则"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$SCRIPT_DIR/../config/99-car-devices.rules" /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger

echo ""
echo "完成。请注销重新登录使 dialout 组生效，然后："
echo "  cd ~/CAR_ws && source /opt/ros/humble/setup.bash && colcon build"
echo "提示：udev 规则中的 VID:PID 请先用 lsusb 核对（见规则文件注释）。"
