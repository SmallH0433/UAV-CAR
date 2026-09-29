# 无人机下位机部署分支

版本：`uav-rpi-8.5`

本分支只保存安装在无人机机载 Raspberry Pi 4B 上的代码和配置。它不包含：

- 无人车下位机或无人车树莓派代码；
- `CAR/` 项目；
- Windows、Mission Planner、Codex 等无人机上位机工具；
- 飞行日志、参数备份、虚拟环境、容器镜像和 ROS 2 构建产物。

## 部署边界

- `ov9281_debug/`：OV9281、10 cm/2 cm 双 AprilTag、标定、距离修正和网页预览。
- `air_ground_landing/`：移动平台估计、IBVS、`LANDING_TARGET`、唯一动作执行器、测距桥和 ROS 2 包。
- `config/systemd/`：视觉、MAVROS、测距桥、动作执行器和飞行遥测记录服务。
- `config/containers/`：rootless Podman 存储配置。
- `config/boot/`：OV9281 和 Pixhawk UART 所需的启动配置片段。
- `tools/`：开机遥测记录器及安装、离线测试脚本；不包含实际飞行日志。

当前运行逻辑：`action_executor` 是任务层到 MAVROS 的唯一动作出口，旧 `guided_executor` 不再由部署服务启动。CH6 先以低位新鲜样本完成 0.4 s 重新授权，再以高位开启跟飞；CH8 在已确认 GUIDED 会话中请求精准降落。降落过程在 GUIDED 中同时下降、居中和对齐标签方向，0.10 m 处交接原生 LAND；高于阈值丢标时最多保持 1.0 s 零速度等待重捕获。飞手外部切模、断连、上锁或状态过期会锁定自动会话，持续高位不能抢回 GUIDED。

相机外参按“画面上为机体前、画面右为机体右”更新。大小 AprilTag 的 PnP 姿态统一转换到 BODY_FRD 并随 `LANDING_TARGET` 传递；网页预览只读显示树莓派实际发布的运动指令方向。相机服务是否拥有 MAVLink 写入权与飞控遥测是否在线分开声明。

现场目标回归的 65 项单元测试通过；发布分支的完整纯 Python 控制回归为 189 项通过、1 项因当前环境缺少 OpenCV 跳过，遥测记录器另有 5 项通过。未解锁无桨测试已确认 CH6 输入和约 10 Hz 位置/速度遥测到达；当 GPS 无定位导致 EKF 不健康时，FOLLOW 被安全门正确拒绝，自动重试已限频为 1 秒。进入 GUIDED 与完整起降流程仍未实飞验收。

## 目标环境

- Raspberry Pi 4B，Raspberry Pi OS Bookworm 64-bit；
- 用户名和主目录：`pi`、`/home/pi`；
- OV9281 CSI 相机；
- Pixhawk 接 `/dev/ttyAMA0`，MAVLink 2，115200 baud；
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
systemctl --user enable ov9281-vision.service ov9281-mavros.service ov9281-range-bridge.service ov9281-landing-ros2.service
bash tools/install_pi_flight_recorder.sh
```

在首次启动正式跟飞/降落服务前，必须拆桨、保持飞控未解锁，并确认 CH6/CH8 均处于关闭位。视觉网页默认监听端口 `8765`。
