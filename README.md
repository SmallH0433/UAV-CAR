# UAV Pixhawk 飞控部署分支

版本：`uav-pixhawk-7.6`

本分支只保存实际写入 QAV280 无人机 Pixhawk 飞控的固件和参数。它不包含：

- 无人车代码或配置；
- 无人机机载 Raspberry Pi 的视觉、MAVROS、ROS 2、systemd 或容器内容；
- Windows、Mission Planner、Codex 等地面端工具；
- 飞行日志、图像、相机标定、参数迁移脚本和测试程序。

## 飞控基线

- 飞控：Pixhawk1（`board_id=9`，STM32F427）；
- 固件：ArduCopter 4.7.0 official；
- ArduPilot Git：`1511f271`；
- 机架：Quad/X（`FRAME_CLASS=1`、`FRAME_TYPE=1`）；
- 固件文件：`firmware/arducopter-pixhawk1-4.7.0-1511f271.apj`。

固件是 ArduPilot 上游版本，没有在本项目中修改飞控源码。对应许可见
`licenses/ARDUPILOT_GPL-3.0.txt`，上游源码为
[`ArduPilot/ardupilot@1511f271`](https://github.com/ArduPilot/ardupilot/commit/1511f271)。

## 参数文件

### `parameters/qav280-current-20260827.param`

从 2026-08-27 最新实机 DataFlash Log 59 的 `PARM` 消息提取，包含 1015 项当前参数。它包含当前这台 QAV280 的 IMU、罗盘、遥控器、动力和其他硬件标定值，只适合原机原飞控恢复或审计，不能直接复制到另一台飞机。

### `parameters/qav280-project-settings.param`

包含 108 项项目级飞控设置，包括飞行模式、CH6/CH7/CH8 角色、EKF 数据源、光流与测距、TELEM1/TELEM2、精准降落、失控保护和日志策略。该文件有意排除了 IMU、罗盘、遥控器行程、舵机、电机、电源模块等硬件专属标定，适合更换飞控后的分阶段迁移。

项目设置仍包含场地相关的 EKF 原点；在其他场地使用前必须重新核对。加载后还必须重新完成目标飞控和机体所需的全部标定。

## 恢复顺序

1. 拆除螺旋桨，备份目标飞控现有固件身份、参数和日志。
2. 仅在确认硬件为 Pixhawk1 后，通过 Mission Planner 的自定义固件功能刷入 `.apj`。
3. 更换飞控时，先恢复默认参数，再加载 `qav280-project-settings.param`，随后完成全部硬件标定。
4. 只有恢复同一台飞控时，才考虑加载 `qav280-current-20260827.param`；加载前必须逐项确认硬件没有变化。
5. 重启后核对固件版本、机架、串口、RC、EKF、光流、测距、精准降落和失控保护；保持无桨、未解锁，直至全部预检通过。

`SHA256SUMS.txt` 给出固件、参数和上游许可文件的 SHA-256，可用于下载后完整性校验。
