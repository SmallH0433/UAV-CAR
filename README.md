# 无人机下位机部署分支

版本：`uav-rpi-8.10`

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

当前运行逻辑：`action_executor` 是任务层到 MAVROS 的唯一动作出口，旧 `guided_executor` 不再由部署服务启动。独立的 `ekf_report_filter` 从 `/uas1/mavlink_source` 筛选飞控 `EKF_STATUS_REPORT` 并发布到 `/landing/ekf_report`，执行器只订阅这个低频主题；不健康报文仍会透传并触发安全拒绝。CH6 先以低位新鲜样本完成 0.4 s 重新授权，再以高位开启跟飞；CH8 在已确认 GUIDED 会话中请求精准降落。EKF 报文使用独立的 5 秒有效期，其他飞行状态仍为 2 秒、位姿和速度仍为 0.3 秒。降落过程在 GUIDED 中先居中并对齐标签方向：同一份新鲜观测的中心误差不超过 20 px、朝向误差不超过 4°并连续保持 0.4 s 后才允许下降，下降期间任一误差超限会立即将垂向指令清零并重新对准。新鲜测距 `<= 0.10 m` 时优先持续请求原生 LAND，不再被 Tag 姿态恢复分支阻断。飞控确认 LAND 且 ON_GROUND 后，执行器只发送普通、非强制 DISARM 作为原生自动上锁兜底，直到收到 `armed=false`。高于交接高度时，位姿/速度短暂掉帧保留 1 秒安全恢复窗口，期间不继续发送下降速度。Tag 重捕获并成功恢复 FOLLOW 后，即使 CH8 仍保持高位，也会重新允许降落请求，并在 GUIDED 心跳确认后再次进入 LAND，无需 CH8 低高切换。动作状态同时报告飞控、位姿、速度、测距年龄和 LAND 阶段。飞手外部切模、断连、上锁或状态过期会锁定自动会话，持续高位不能抢回 GUIDED。

相机外参按“画面上为机体前、画面右为机体右”更新。大小 AprilTag 的 PnP 姿态统一转换到 BODY_FRD 并随 `LANDING_TARGET` 传递；网页预览只读显示树莓派实际发布的运动指令方向。相机服务是否拥有 MAVLink 写入权与飞控遥测是否在线分开声明。

网页控制台保留浅绿色水平旋转指令箭头，并新增执行层状态卡，显示当前动作、执行状态、控制权、跟随/降落活动、模式门控与拒绝原因；超过 2 秒未更新时标记过期并隐藏旧详情。偏航箭头只有在飞控在线、处于 GUIDED、指令新鲜且未屏蔽 yaw rate 时才显示。状态桥接保留既有 `telemetry` 与 30 分钟内存历史接口，视觉页面继续禁用高负载曲线绘制。红色平移、浅绿色旋转和粉色 Tag 朝向分别表示不同量，均不是实际运动测量。

8.10 的执行层专项回归为 122 项全部通过；完整部署分支回归共 210 项，其中 209 项通过、1 项因当前环境缺少 OpenCV 跳过，无失败。覆盖对准驻留、偏离立即停止下降、丢标后重新驻留、重复帧不刷新观测年龄、FOLLOW 从实际零速度恢复限速，以及 Tag 重捕获后在 CH8 持续高位时重新进入 LAND。Python 与内嵌 JavaScript 语法、执行层网页状态、既有方向箭头和过期隐藏逻辑均通过。尚未运行本版 ROS/SITL 流程或真实带桨着陆；稳定下降、重入 LAND 和真实负载下的表现仍需现场验收。

控制台增量已通过 Python/内嵌 JavaScript 语法检查、后端 yaw-rate 掩码单测，以及左右旋和过期隐藏逻辑检查。树莓派上 MAVROS、动作执行器、视觉服务和飞行记录器均为 active；API 保留遥测并新增 `motion_command.yaw_rate_rad_s`。尚未通过电机实际旋转验证箭头方向。

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
