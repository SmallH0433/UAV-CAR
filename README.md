# 无人机下位机部署分支

版本：`uav-rpi-8.4`

本分支只保存安装在无人机机载 Raspberry Pi 4B 上的代码和配置。它不包含：

- 无人车下位机或无人车树莓派代码；
- `CAR/` 项目；
- Windows、Mission Planner、Codex 等无人机上位机工具；
- 飞行日志、参数备份、虚拟环境、容器镜像和 ROS 2 构建产物。

## 部署边界

- `ov9281_debug/`：OV9281、10 cm/2 cm 双 AprilTag、标定、距离修正和网页预览。
- `air_ground_landing/`：移动平台估计、IBVS、Elastic 适配、`LANDING_TARGET`、GUIDED/LAND 执行器和 ROS 2 包。
- `config/systemd/`：视觉、MAVROS 和跟飞/降落三个用户服务。
- `config/containers/`：rootless Podman 存储配置。
- `config/boot/`：OV9281 和 Pixhawk UART 所需的启动配置片段。

当前运行逻辑：CH6 先以低位新鲜样本完成 0.4 s 重新授权，再以高位开启跟飞；CH8 在已确认 GUIDED 会话中以高电平请求降落，低位或中立取消。飞手外部切模、断连、上锁或状态过期会锁定自动会话，持续高位不能抢回 GUIDED。候选超时 1.0 s，短时视觉丢失会在 1.0 s 宽限期内保持 GUIDED 并发送零速度，水平速度上限 0.20 m/s，水平加速度上限 0.40 m/s²。健康测距或内圈标签连续 0.4 s 小于等于 0.10 m 后锁存 LAND；CH8 明确低位、CH6 明确关闭、链路丢失、上锁或飞手切换模式会解除锁存。

相机外参按“画面上为机体前、画面右为机体右”更新。大小 AprilTag 的 PnP 姿态统一转换到 BODY_FRD 并随 `LANDING_TARGET` 传递；网页预览只读显示树莓派实际发布的运动指令方向。相机服务是否拥有 MAVLink 写入权与飞控遥测是否在线分开声明。

本版本还包含 companion GUIDED 自主下降与普通 DISARM 的实验策略，但硬件配置保持关闭，仅独立 SITL 覆盖文件启用；实机打开还需要显式飞行批准。Python 控制套件 106 项通过，另有 1 项 OpenCV 图像解码测试在不含 OpenCV 的环境中跳过；这不等价于完整实飞验收。

## 目标环境

- Raspberry Pi 4B，Raspberry Pi OS Bookworm 64-bit；
- 用户名和主目录：`pi`、`/home/pi`；
- OV9281 CSI 相机；
- Pixhawk 接 `/dev/ttyAMA0`，MAVLink 2，57600 baud；
- Python 3、Picamera2、OpenCV、NumPy、Podman；
- ROS 2 Humble 和 MAVROS 运行在 Podman 镜像中。

## 恢复概要

安装系统依赖：

```bash
sudo apt update
sudo apt install -y podman uidmap slirp4netns fuse-overlayfs python3-venv python3-pip python3-opencv python3-numpy build-essential cmake
python3 -m venv --system-site-packages /home/pi/venvs/landing
/home/pi/venvs/landing/bin/pip install -r requirements-vision.txt
```

将源码放到服务约定路径：

```bash
cp -a ov9281_debug /home/pi/ov9281_debug
cp -a air_ground_landing /home/pi/air_ground_landing
mkdir -p /home/pi/.config/systemd/user /home/pi/.config/containers
cp config/systemd/*.service /home/pi/.config/systemd/user/
cp config/containers/storage.conf /home/pi/.config/containers/storage.conf
```

构建 ROS 2/MAVROS 镜像：

```bash
cd /home/pi/air_ground_landing
podman build --format docker -f Containerfile.ros2-precland -t localhost/air-ground-landing-ros2:humble .
```

启用开机服务：

```bash
sudo loginctl enable-linger pi
systemctl --user daemon-reload
systemctl --user enable ov9281-vision.service ov9281-mavros.service ov9281-landing-ros2.service
```

在首次启动正式跟飞/降落服务前，必须拆桨、保持飞控未解锁，并确认 CH6/CH8 均处于关闭位。视觉网页默认监听端口 `8765`。
