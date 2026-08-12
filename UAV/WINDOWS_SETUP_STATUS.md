# Windows 开发环境配置状态

日期：2026-08-05（Asia/Shanghai）

依据：[WINDOWS_DEVELOPMENT_HANDOFF.md](C:/Users/zc325/Downloads/WINDOWS_DEVELOPMENT_HANDOFF.md)

## Gate 0 盘点

- Windows 注册表识别：Windows 10 Home，DisplayVersion 25H2，Build 26200.8875。
- CPU：11th Gen Intel Core i7-11700，8 核 / 16 线程。
- 内存：约 15.7 GB。
- GPU：Intel UHD Graphics 750。
- 磁盘余量：C: 176.6 GB，D: 675.4 GB，E: 74.2 GB。
- WSL：已安装 WSL 2.7.11，`Ubuntu-22.04` 已注册并显示为 VERSION 2；用户 `zc325` 已完成首次初始化。
- 项目目录：交接归档已解压，包含 `air_ground_open_source`、`imx296_debug` 和 `docking_design`。
- 仓库审计：已在 WSL Linux 工作区执行，39 个仓库均为 `exact`。

## 已完成配置

- Windows Terminal：已通过 winget 检查并确认安装。
- `C:\Users\zc325\.wslconfig`：已创建，限制为 8 GB 内存、4 个处理器、4 GB swap。
- 未启用 `networkingMode=mirrored`：当前系统识别为 Windows 10；该模式按微软文档要求 Windows 11 22H2 或更高版本。

## 已完成配置

- WSL2 与 Ubuntu 22.04：WSL2 已启用，Ubuntu 22.04.5 LTS 已完成首次初始化。
- VS Code：1.131.0 已安装，`code.cmd` 可用。
- Windows 系统 Git：Git for Windows 2.55.0.windows.3 已安装到用户目录并加入用户 PATH。
- usbipd-win：未检测到；在确有 USB 设备并确认所有权方案前不安装/转交设备。
- ROS 2、ArduPilot SITL：已完成核心安装和 SITL Copter 编译。

## 下一步（软件环境已就绪）

Ubuntu 用户 `zc325` 已创建，并在 PowerShell 7.6.4 中验证：

```powershell
wsl --list --verbose
```

`Ubuntu-22.04` 的 `VERSION` 为 `2`。项目已解压到 `~/projects/`，并完成只读仓库审计；未运行 `git pull`、`git reset --hard`、`git clean` 或批量删除。

## 安全边界

本阶段没有刷写固件、修改真实飞控参数、解锁飞控、启动电机、移动车辆、接通高功率充电回路、转交 USB 设备或修改 Windows 防火墙。

## 2026-08-05 后续配置进度

- PowerShell：已确认使用 PowerShell 7.6.4；VS Code 默认终端已固定为该版本。
- 迁移包：`D:\Codex\UAV\air_ground_windows_handoff_20260805.tar.gz` SHA-256 校验通过，实际值为 `e5135e5bed9680ffa9c1ca667250c4bd9f1525986cbf87914e931bb5065c33d3`。
- VS Code：1.131.0 已安装，安装器 Microsoft 签名验证通过。
- VS Code 扩展：`ms-vscode-remote.remote-wsl` 0.104.3 已安装并通过扩展签名验证。
- VS Code 远程开发：`ms-vscode-remote.remote-ssh` 已安装；待提供 Jetson IP 和用户名后建立主机配置。
- Jetson SSH：已确认 `192.168.1.103:22` 可达；Windows SSH 配置已添加 `jetson-uav`（用户 `jetson`）；SSH 密钥免密登录已完成。
- Jetson Nano 兼容性：已确认 Ubuntu 18.04.6 / ARM64 / glibc 2.27；当前 VS Code Server 不满足 glibc 要求，因此改用 SSH 密钥和指定目录同步脚本，保留 JetPack 4 系统。
- Jetson 工作流：已创建 `setup_jetson_ssh_key.ps1`、`sync_to_jetson.ps1` 和 `JETSON_NANO_REMOTE_WORKFLOW.md`；脚本均已通过 PowerShell 7 语法检查。
- Jetson Nano 远程目录：已创建 `/home/jetson/uav/{bin,src,logs}`；当前识别相机为 IMX219，NVIDIA Argus/GStreamer 无界面采集测试通过，并生成 `/home/jetson/uav/logs/imx219-test.jpg`。
- Jetson Nano 限制：根分区约 57 GB、剩余约 1.5 GB；暂不安装 ROS 2 Humble、大型 CUDA/深度学习包或整套迁移源码。
- Jetson Nano MAVLink：已安装 Python 3.6 兼容的 `pymavlink 2.4.41`；MAVLink dialect 模块可导入。Nano 预装 NumPy 1.19.5 导入时触发非法指令，因此暂不使用依赖 NumPy 的 `mavutil` 路径，不改动现有机器学习栈。
- Jetson Nano MAVLink 兼容启动器：已部署 `/home/jetson/uav/bin/jetson_pymavlink_python.sh`，隔离异常 NumPy 后 `pymavlink.mavutil` 导入测试通过。
- SITL 到 Jetson 联调：WSL ArduCopter SITL 心跳已通过 TCP 读取，并经 UDP 14550 转发到 Nano；Nano 已收到 `system=1 component=0` 心跳，发送链路验证通过；仿真结束后进程已清理。
- SITL 视觉消息：`PLND_ENABLED=1`、`PLND_TYPE=1` 参数回读通过；合成 `LANDING_TARGET` 已按 BODY_FRD、`position_valid=1`、约 20 Hz 发送 101 条，距离 5.002 m；全程 `ARMED=0`、仅仿真。
- Codex CLI：0.147.0-alpha.1.2 可运行；当前 CLI 目录已加入用户 PATH。
- MCP：已注册官方 `openaiDeveloperDocs`，地址为 `https://developers.openai.com/mcp`；原有 `node_repl` 保留。
- PowerShell：已启用 Codex CLI 补全配置。
- 项目解压：已完成到 `D:\Codex\UAV\air_ground_open_source`、`imx296_debug`、`docking_design`。
- WSL：`Ubuntu-22.04` 已注册为 WSL 2，用户 `zc325` 已完成首次初始化。
- Git for Windows：2.55.0.3（64 位）已完成 SHA-256 和签名校验并安装。
- Mission Planner：已从 ArduPilot 官方地址下载并安装到 `C:\Program Files (x86)\Mission Planner`；版本文件为 `1.3.9384.38258`。
- VS Code 中文界面：`ms-ceintl.vscode-language-pack-zh-hans` 已安装，`locale.json` 已设为 `zh-cn`；重启 VS Code 后生效。
- WSL Linux 工作区：归档已解压到 `/home/zc325/projects`，包含三个项目目录。
- 仓库审计：39 个仓库全部为 `exact`；审计输出同时发现归档带入的 macOS `._*` 元数据文件。
- ROS 2：Humble 核心版、`rosdep`、`colcon`、`vcstool` 已安装；`rosdep update` 已完成；用户 `.bashrc` 已自动加载 `/opt/ros/humble/setup.bash`。
- ROS 2 示例：`ros-humble-demo-nodes-cpp`、`ros-humble-demo-nodes-py` 已安装；talker/listener 已互相收到 `Hello World` 消息。
- MAVROS：`ros-humble-mavros` 与 `ros-humble-mavros-extras` 已安装，GeographicLib 数据已部署。
- Gazebo：Gazebo Harmonic 8.14.0 已安装；ArduPilot Gazebo 插件已成功编译，包含 ArduPilot、GStreamer、相机变焦和降落伞插件。
- MAVProxy：MAVProxy 1.8.74、pymavlink 2.4.49 已安装到 `/home/zc325/.local/bin`。
- ArduPilot：固定版本 SITL 已初始化子模块并成功编译 Copter，产物为 `01_flight_stack/ardupilot/build/sitl/bin/arducopter`。
- SITL 验证：`sim_vehicle.py --help`、SITL 程序、ROS 2、MAVROS、Gazebo 插件和 MAVProxy 均已验证；无界面 ArduCopter SITL 已通过本机 TCP 收到 MAVLink 心跳；未连接真实硬件。
- IMX219 调用验证：已通过 SSH 远程调用 Jetson Nano 的 Argus/GStreamer，成功采集 1920×1080 JPEG 到 `D:\Codex\UAV\imx219-test-latest.jpg`；当前画面可见，但相机安装方向有 90° 旋转。采集脚本已支持 `IMX219_FLIP_METHOD=0..7` 软件旋转/翻转，默认值为 `0`。

## 双机器人部署边界

- 无人机：Raspberry Pi 4B + IMX296，负责本地 AprilTag、精准降落视觉和 MAVLink；Pixhawk 负责姿态、位置、电机和失效保护。
- 无人车：Jetson Nano + IMX219，作为车载 Linux 计算机，后续承担 ROS 2、AI 识别、规划、会合和车端通信；当前已完成相机调用与 MAVLink 基础联调。
- 充电控制：由独立 STM32 负责硬件保护和接触器控制，不交给 Raspberry Pi 或 Jetson。
- 本机 WSL：只承担 ROS 2、Gazebo、ArduPilot SITL 和开发验证，不代表最终安装位置。
- 之前的 `LANDING_TARGET` 仿真是协议链路测试，不改变 Jetson 的最终车载定位；无人机精准降落仍以 Pi 4B + IMX296 本地链路为第一部署方案。

## 双机器人路线执行记录（2026-08-05）

### 无人机端：Raspberry Pi + IMX296

- SSH 主机 `uavpi` 已确认是 Debian 12，IMX296 已由 libcamera 识别为 1456×1088、10-bit、约 60 fps。
- 已部署 `/home/PI/imx296_debug/` 项目脚本；`imx296_debug.py --headless` 成功采集 `/home/PI/imx296_test/imx296_headless.jpg`。
- 已在 `/home/PI/venvs/landing` 安装 `pupil-apriltags`，检测器已运行 5 秒并生成 CSV 与标注图；本次未检测到有效 `tag36h11`，现场图案被物体遮挡且不是可确认的标准标签。
- 未打开真实 MAVLink 串口、未发送 `LANDING_TARGET`、未修改 Pixhawk 参数。完成标准标签和相机标定后，再进入安全 dry-run。

### 无人车端：Jetson Nano + IMX219

- Jetson Nano 保持 Ubuntu 18.04/JetPack 4，不强行安装 ROS 2 Humble；已完成 IMX219 Argus/GStreamer 采集、轻量 pymavlink 和网络心跳基础。
- ROS 2/HUNTER 车端工作区放在 WSL `/home/zc325/ros2_ws`，已成功编译 `ugv_sdk`、`hunter_msgs`、`hunter_base`。
- WSL 当前没有 `can0`，说明尚未接入 USB-CAN/真实 HUNTER；未启动底盘节点、未发送 `/cmd_vel` 或 CAN 运动帧。

### 当前下一入口

1. 给树莓派相机放置无遮挡的标准 `tag36h11` ID 0 标签并完成棋盘格采集；
2. 生成 IMX296 标定文件，运行 `LANDING_TARGET` dry-run；
3. 确认 HUNTER SE 和 USB-CAN 型号后，只读验证 `can0` 和 `candump`；
4. 最后才进入动态会合仿真，真实运动与充电仍需人工确认。
 
### 相关会话已完成的树莓派视觉结果

- 已从项目内“配置树莓派 SSH 调试”会话核对到：IMX296 第二轮标定文件为 \`imx296_calibration_run2.yaml\`，RMS 重投影误差约 \`0.335 px\`，平均误差约 \`0.282 px\`。
- 已成功识别 \`tag36h11 / ID 0\`：有效观测约 \`43\` 条，典型距离约 \`1.30 m\`，典型位置约 \`x=0.083 m、y=0.017 m、z=1.299 m\`，Hamming 为 \`0\`，decision margin 约 \`74\`。
- 已生成 \`landing_target_dry_run.jsonl\`，共 \`43\` 个 MAVLink 2 \`LANDING_TARGET\` 干运行包；当时使用 \`MAV_FRAME_CAMERA_OPTICAL\`、\`position_valid=0\`，未连接飞控、未发送真实飞行指令。
- 当前树莓派上的最新现场测试没有检测到目标，是因为目标当前未完整放入镜头；不能覆盖此前已经成功的 43 条有效结果。
- 在相机安装到无人机并完成相机到机体 \`BODY_FRD\` 的旋转/平移测量前，仍不得把相机光学坐标直接改成 \`BODY_FRD\`，也不得连接真实 Pixhawk。
 
## Pixhawk 重新接线后诊断（2026-08-06）

- Windows 已识别 Pixhawk 为 COM4，Mission Planner 已连接。
- 当前固件显示为 ArduCopter 4.7.0，板型 fmuv3。
- 截图确认原先的 PreArm: Gyros inconsistent 已消失，说明重新接线后陀螺仪一致性问题不再出现。
- 当前剩余提示为 PreArm: Hardware safety switch，属于实体安全开关尚未解除。
- Mission Planner 地图区域的 503 Service Unavailable 只影响地图显示，不代表飞控通信故障。
- 本次只做状态读取，未解锁、未启动电机、未修改参数；继续保持拆桨。
- 安全开关打开后再次截图确认：PreArm: Hardware safety switch 已清除，新的唯一主要预解锁提示为 PreArm: RC not found。
- 当前调试顺序已确认为：陀螺仪一致性通过 -> 硬件安全开关通过 -> 遥控接收机链路待检查。
- 接入 RC 后再次截图确认：PreArm: RC not found 已清除，Mission Planner 当前显示“准备解锁”；陀螺仪、安全开关和遥控器三项基础检查均通过。
- 本阶段仍保持拆桨、未解锁、未启动电机；下一步只能进行遥控通道只读观察和无桨校准，不能直接试飞。
- 树莓派与 Pixhawk 通信检查（2026-08-06）：树莓派 IP 为 192.168.1.11，MAVLink Python 环境正常，但当前没有检测到 /dev/ttyACM* 或 /dev/ttyUSB*，说明 Pixhawk 还未物理接入树莓派。
- 已部署只读检测器 /home/PI/pi_pixhawk_heartbeat_check.py；接线后只等待 HEARTBEAT，不写参数、不解锁、不发送控制命令。
- 已关闭树莓派启动时的 serial0 Linux 控制台并完成重启，避免 UART 与系统控制台冲突；原文件已保留为 /boot/firmware/cmdline.txt.codex-backup-20260806。
- 重启后通过 /dev/serial0、57600 baud 只读监听仍未收到 Pixhawk 心跳，Pi 端软件已就绪，当前待核对 TELEM1 的 TX/RX/GND 物理线序或 Pixhawk SERIAL1 配置。
- TELEM1 重新检查结果（2026-08-06）：树莓派 /dev/serial0 在 57600 和 115200 baud 下均未收到 HEARTBEAT，原始串口读取也没有字节；当前尚未形成 Pi-Pixhawk MAVLink 链路。
- Pi-Pixhawk 通信验收（2026-08-06）：/dev/serial0、57600 baud 已收到 HEARTBEAT，system=1 component=0，10 秒内收到 11 个 HEARTBEAT；只读检查确认未写参数、未解锁、未发送控制命令。
- 当前 TELEM1 暂只观察到 HEARTBEAT，10 秒内未观察到 ATTITUDE、SYS_STATUS、RC_CHANNELS、STATUSTEXT；下一步需经负责人确认后配置遥测消息流率，再进入视觉消息联调。

## TELEM1 遥测流率配置（2026-08-06）

- 已取得负责人明确授权，仅配置 TELEM1 遥测频率；全程保持拆桨、飞控未解锁，未修改飞行模式、姿态控制、失控保护或电机参数。
- ArduCopter 4.7.0 已将旧 `SR0_* ... SR9_*` 重命名为 `MAV1_* ... MAV10_*`；由于 USB/SERIAL0 是第一个 MAVLink 端口，TELEM1/SERIAL1 是第二个 MAVLink 端口，所以旧 `SR1_*` 对应当前 `MAV2_*`。
- 配置前确认 `SERIAL1_PROTOCOL=2`（MAVLink2）、`SERIAL1_BAUD=57`（57600 baud）、`SERIAL1_OPTIONS=0`；这三项未修改。
- 配置前 6 项 `MAV2_*` 流率均为 0；原值及参数类型已备份到 `D:\Codex\UAV\telem1_mav2_stream_backup_20260806.json`。
- 已写入并收到飞控逐项确认：`MAV2_EXT_STAT=2`、`MAV2_RC_CHAN=5`、`MAV2_POSITION=3`、`MAV2_EXTRA1=10`、`MAV2_EXTRA2=3`、`MAV2_EXTRA3=3` Hz。
- 写入后 USB 重新读取 6 项数值全部一致；验证记录为 `D:\Codex\UAV\telem1_mav2_stream_verify_20260806.json`。
- 因流率参数需要重启生效，已先通过心跳确认 `ARMED=0`，再执行一次 Pixhawk 控制器重启；飞控返回 `REBOOT_ACK_RESULT=0`。未执行固件升级或刷写。
- 重启后树莓派 `/dev/serial0`、57600 baud 的 12 秒只读验收结果：`HEARTBEAT=17`、`ATTITUDE=67`、`SYS_STATUS=13`、`RC_CHANNELS=34`、`STATUSTEXT=12`，证明 TELEM1 主遥测已生效。
- 配置工具为 `D:\Codex\UAV\configure_telem1_mav2_streams.py`；Windows 隔离环境为 `D:\Codex\UAV\.venv-mavlink-windows`，包含 `pymavlink 2.4.49` 和 `pyserial 3.5`。

## Pi–Pixhawk 精准降落旁路验收（2026-08-06）

- Pi 从 `/dev/serial0` 发出 `PARAM_REQUEST_LIST` 后收到飞控参数回复，确认 Pi TX -> Pixhawk TELEM1 RX 上行链路有效；此前 `PING` 无回复不能代表线路故障。
- 一次安全检查发现飞控处于 `ARMED=1`（连续 5 个心跳 `base_mode=209`），后续发送立即暂停；人工上锁后连续 12 个心跳均为 `ARMED=0`。蜂鸣声与该次解锁状态相符。
- 精准降落参数只读结果：`PLND_ENABLED=0`、`PLND_TYPE=0`、`PLND_EST_TYPE=1`、`PLND_OPTIONS=0`；未修改这些参数。
- 已完成 43 条有效历史观测的真实串口回放，发送 78 条 `CAMERA_OPTICAL + position_valid=0` 消息；飞控保持未解锁。
- 已完成 IMX296 实时串口旁路：一致配置为 135 mm 标签、`imx296_calibration_run4_17mm.yaml` 和 `range_correction_20260806.json`。
- 一致配置 15 秒验收得到 45 条有效检测并发送 45 条消息；距离均值 0.5553 m，对应台架实测 0.555 m，图像中 tag36h11 ID 0 边框完整且标记为 VALID。
- 保守默认消息仍为 `MAV_FRAME_CAMERA_OPTICAL(27)`、`position_valid=0`。用户随后明确指定名义外参：相机中心等同机体中心、平移为零、roll/pitch 为零、机头在画面左侧，对应 `BODY_FRD=(-x_camera,-y_camera,+z_camera)`。
- 名义 BODY_FRD 已通过 4 个单元测试、43/43 离线回归、SITL 51 条、真实 TELEM1 回放 49 条和 15 秒实时 46/46 条验收；实时距离均值约 0.55522 m，frame 12、`position_valid=1`。
- 所有真实 BODY_FRD 验收均保持连续 `ARMED=0`、`PLND_ENABLED=0`、`PLND_TYPE=0`，没有模式、解锁、电机、起飞或降落命令。该名义外参只批准拆桨台架与 SITL，不批准真实飞行。
- 四点距离修正使用完整精度 `scale=0.613586842370`、`offset=-0.010204131220 m`，最大残差约 4.4 mm；0.87 m 点的新拟合结果是 39 帧旧数据的离线复算，不是新实时采集。
- 只读记录 `PLND_ORIENT=25`、`PLND_LAG≈0.02`，当前未采用、未修改。
- 已部署安全启动器 `/home/PI/uav/bin/run_landing_bench.sh`：要求连续 3 个未解锁心跳、`PLND_ENABLED=0`、`PLND_TYPE=0`、相机未被预览占用，否则拒绝运行；不会写参数、改模式、解锁或发送电机命令。默认 CAMERA_OPTICAL，只有显式设置 `FRAME_PROFILE=body-frd-nominal` 才使用名义 BODY_FRD。
- 台架配置为 `D:\Codex\UAV\config\uav_precision_landing_bench.yaml`，交接文档为 `D:\Codex\UAV\docs\PI_PIXHAWK_BENCH_HANDOFF_20260806.md`。
- 测试结束后已恢复相机预览服务（`192.168.1.11:8765` 返回 HTTP 200）和 Mission Planner；精准降落桥未设置为常驻服务。
- 已通过 Pi TELEM1 只读保存启用前 PLND 快照：相机偏移 X/Y/Z 均为 0，`PLND_YAW_ALIGN=0`、`PLND_STRICT=1`、高度范围 0.75–8 m、最大水平距离 2.5 m；未修改任何参数。
- PLND 启用方案记录在 `D:\Codex\UAV\config\precision_landing_activation_plan_20260806.yaml`；该行所述待授权阶段随后已完成，当前权威状态见下方“真实 PLND 启用与未解锁验收”。
- 已核对 ArduPilot 官方 fmuv3 stable 4.7.0 特性清单，包含 `AC_PRECLAND_ENABLED` 与 `AC_PRECLAND_MAVLINK_ENABLED`，确认当前板型固件编译进了 MAVLink 精准降落功能。
- 最终高频 BODY_FRD 真实台架已完成：`30 fps / 4 threads / quad_decimate=3.0`，15 秒识别并发送 179/179 条，消息时间戳实际频率 12.68 Hz；距离均值 0.556631 m，X/Y/Z 均值 0.075942/-0.014375/0.551239 m，frame 12、`position_valid=1`、Hamming 全为 0。全程连续 `ARMED=0`、`PLND_ENABLED=0`、`PLND_TYPE=0`，无控制命令；测试后预览恢复 HTTP 200、无 landing bridge 常驻。

## 真实 PLND 启用与未解锁验收（2026-08-06）

- 用户明确授权仅在拆桨、未解锁台架启用精准降落；安全门曾检测到连续 `ARMED=1` 并拒绝全部操作，人工上锁后才继续。
- 启用前 PLND 完整参数备份：`D:\Codex\UAV\config\precision_landing_before_activation_20260806.json`。
- 已按顺序设置并回读：`PLND_ENABLED=1` -> 未解锁重启 -> `PLND_TYPE=1`；没有修改模式、解锁状态、电机、失效策略或其他 PLND 参数。
- 重启前的 `PreArm: Gyros not calibrated` 在机体完全静止重启后消失；12 秒采样中陀螺仪健康 `25/25`、加速度计健康 `25/25`，确认不需要六面重新校准。
- 最终 PLND 1/1 实时台架使用 BODY_FRD/frame 12/`position_valid=1`，15 秒检测并发送 177/177 条，实际 12.52 Hz，Hamming 全 0；距离均值 0.369594 m。
- 证据：`D:\Codex\UAV\imx296_debug\landing_target_body_frd_plnd_enabled_final_20260806.jsonl/.jpg`。测试后再次确认连续 `ARMED=0`、PLND 1/1，预览 HTTP 200，无 bridge 常驻。
- 当前仅完成拆桨未解锁台架；名义零外参尚未物理测量，真实解锁、飞行、Land/RTL 精准降落均未授权。

## 无桨真实命令链路台架（2026-08-06）

- 用户明确确认螺旋桨已拆除、电机区域清空并授权短时解锁；未使用强制解锁值 `21196`。
- 第一次解锁被 `PreArm: Hardware safety switch` 安全拒绝；人工解除实体安全开关后，第二次正常解锁由真实飞控 heartbeat 确认进入 `ARMED=1`，约 2 秒内自动上锁。
- 解锁窗口采集到 5 组四路 PWM 输出，均为 1100 us；仅证明飞控输出链路已工作，不宣称物理电机一定旋转。
- 上锁状态下 TAKEOFF 返回 ACK 4，没有起飞；LAND 返回 ACK 0并短暂进入 mode 9，随后恢复 STABILIZE/mode 0。
- 最终脚本连续 5 次、独立复核连续 8 次确认 `ARMED=0、custom_mode=0`；PLND 回读仍为 1/1，相机预览保持运行。
- 证据：`D:\Codex\UAV\config\pixhawk_command_bench_20260806.json`。没有在解锁状态发送 TAKEOFF/LAND，没有真实飞行或装桨测试授权。

## 无桨逐电机 Motor Test（2026-08-06）

- 用户再次确认动力电池已连接、ESC 上电提示音完成、电机周围净空且螺旋桨已拆除。
- 全程只使用 `MAV_CMD_DO_MOTOR_TEST`，没有发送解锁、模式、起飞、降落或执行器覆盖命令，也未使用强制解锁值 `21196`。
- 5% 测试时 Motor 1 命令被接受并输出 1050 us；该值低于当前 `MOT_SPIN_MIN=0.15`，因此不作为稳定旋转测试值。
- 随后按当前飞控最低旋转设置，以 15% 油门、每个 1 秒依次测试 Motor 1–4，四次 ACK 均为 0，并分别收到 `starting motor test` / `finished motor test`。
- 输出映射实测为：Motor 1 -> SERVO1=1150 us，Motor 2 -> SERVO4=1150 us，Motor 3 -> SERVO2=1150 us，Motor 4 -> SERVO3=1150 us；其他三路在对应窗口保持 1000 us。
- ArduPilot 在 Motor Test 窗口内短暂报告 armed 标志；每个窗口结束后均连续 3 次确认 `ARMED=0`，最终又连续 8 次独立确认 `ARMED=0`。
- 最终无测试/精准降落桥进程常驻，相机预览 HTTP 200。电气输出链路已确认；电机是否实际旋转及方向仍需现场人员目视确认。
- 证据：`D:\Codex\UAV\config\pixhawk_motor_test_spin_20260806_222803.json`；执行脚本：`D:\Codex\UAV\pi_pixhawk_motor_test.py`。

## 移动 AprilTag 跟随第一批离线实现（2026-08-07）

- 用户确认硬件：QSDZ/u-blox NEO-M9N GNSS、MicoAir MTF-01P 光流+短距测距、Pixhawk 电源模块、Pi 4B + 下视 IMX296。
- 已完成纯离线模块：目标滤波、水平跟随控制、安全状态机和 GUIDED 速度消息编码；旧的未解锁精准降落桥保持不变。
- 10 项单元测试全部通过，覆盖目标连续捕获、位置跳变拒绝、方向、速度/加速度限幅、目标丢失 HOLD、RC 重启许可、模式接管及 MAVLink 超速拒绝。
- 10 秒/10 Hz 动态回放共 101 条，命令速度峰值 0.153184 m/s，最大合成水平误差 0.175756 m；1 秒遮挡触发 PREDICT_DECEL/HOLD。
- 回放未打开真实串口，未发送控制命令。证据：`D:\Codex\UAV\output\apriltag_follow_synthetic_20260807.jsonl` 和 `apriltag_follow_synthetic_20260807_summary.json`。
- 当前硬件仍未核验：Pi `192.168.1.11` SSH 超时；待上电后只读确认 GPS、OPTICAL_FLOW、DISTANCE_SENSOR 和 BATTERY_STATUS。
- 真实飞行仍被以下条件阻塞：`BATT_MONITOR=0`、相机外参未实测、0.87 m以上距离修正未标定、GPS/光流/测距数据尚未确认进入 EKF。

## 移动 AprilTag 跟随 SITL 闭环（2026-08-07）

- 已新增 `D:\Codex\UAV\uav_sitl_apriltag_follow_test.py` 和 `D:\Codex\UAV\run_sitl_apriltag_follow_test.sh`；端点硬编码为本机 `udpin:127.0.0.1:14550`，拒绝连接真实设备。
- 使用 WSL 中已编译的 ArduCopter SITL V4.6.3，并由 MAVProxy 提供本地 UDP 转发；保留正常 IMU/GPS/EKF/位置预检，未关闭预检、未强制解锁。
- 仿真完成正常解锁、3 m 指令起飞、32 秒/10 Hz 移动目标跟随、1.2 秒丢标、RC 许可循环、BRAKE 模式接管、恢复 GUIDED、LAND 和自动上锁。
- 共记录 321 个周期：ACQUIRE 4、FOLLOW_XY 265、PREDICT_DECEL 5、HOLD 23、DISABLED 4、PILOT_OVERRIDE 20；无 ABORT。
- 命令速度峰值 `0.200000 m/s`，FOLLOW_XY 最大水平误差 `0.290473 m`，满足当前限速 `0.20 m/s` 和误差 `<0.30 m` 的验收门限。
- 综合结果 `passed=true`、`landed_and_disarmed=true`、`physical_vehicle_connected=false`，真实 Pixhawk 指令为 0。
- 证据：`D:\Codex\UAV\output\apriltag_follow_sitl_closed_loop_20260807.jsonl` 与 `D:\Codex\UAV\output\apriltag_follow_sitl_closed_loop_20260807_summary.json`。
- 下一阶段是拆桨台架“命令预览”：只读真实相机/Pixhawk并记录拟发送速度，禁止发送 `SET_POSITION_TARGET_LOCAL_NED`；Pi 当前离线时无需用户操作。

## MTF-01P / GPS / 电池只读硬件审计（2026-08-07）

- 用户确认无人机拆桨、水平放置于纹理地面、Pi/Pixhawk 上电且未解锁，MTF-01P 接 Pixhawk TELEM2；全程未写参数、未改模式、未解锁、未发送运动或电机命令。
- Pi `192.168.1.11` SSH 恢复正常；`/dev/serial0 -> /dev/ttyAMA0`，审计前后均无串口桥接进程占用。
- 30 秒真实 MAVLink 监听得到 30/30 个未解锁心跳；MTF 数据已进入飞控：`DISTANCE_SENSOR=90`、`OPTICAL_FLOW=90`，均约 3 Hz；测距约 0.19 m、向下 orientation 25、光流 quality 约 106。
- 20 秒静态高频统计得到 1605 条光流和 1520 条测距；quality 均值 100.08/255，静止补偿速度均值 X/Y 为 0.00012/0.00025 m/s，标准差 0.00854/0.01213 m/s，存在约 ±0.20 m/s 瞬时尖峰；独立测距 0.18–0.20 m、均值 0.18666 m。
- `OPTICAL_FLOW.ground_distance` 大多为 -1，但独立 `DISTANCE_SENSOR` 连续有效；后续高度数据必须使用并验证 `DISTANCE_SENSOR`/EKF range，不依赖该扩展字段。
- TELEM2 参数符合基础接入：`SERIAL2_PROTOCOL=1`、`SERIAL2_BAUD=115`、`FLOW_TYPE=5`、`RNGFND1_TYPE=10`、`RNGFND1_MIN=0.01 m`、`RNGFND1_MAX=8 m`、`RNGFND1_ORIENT=25`。
- 消息源审计发现 MTF 原始消息为 `sys=1/comp=88`，飞控为 `sys=1/comp=1`；`SERIAL2_OPTIONS=0` 时约 80 Hz 原始光流/测距会转发到 TELEM1。ArduPilot 4.5+ 官方建议使用 MicoAssistant 将 MTF `mav_id` 改为非 1（如 200），并设置对应 `SERIAL2_OPTIONS=1024`；本次未执行。
- GPS 当前 `fix_type=1`、0 星、经纬度为 0；室内状态下没有定位。`BATT_MONITOR=0`、电压 0、电流和余量未知，电池监测仍是飞行阻塞项。
- EKF3 已启用，但 `EK3_SRC1_POSXY=3`、`EK3_SRC1_VELXY=3` 仍使用 GPS；`EKF_STATUS_REPORT.flags=231`，无水平绝对/相对位置并处于 constant-position 状态。MTF 有数据但尚未作为当前水平速度源，不能据此进入 Loiter/GUIDED 跟随飞行。
- 配置快照：`D:\Codex\UAV\config\mtf01_telem2_audit_20260807.yaml`；证据位于 `D:\Codex\UAV\output\follow_hardware_audit_20260807.txt`、`sensor_parameter_audit_20260807.json`、`mtf_static_audit_20260807.json`、`mtf_message_source_audit_20260807.json`。

### 安全开关解除后复测

- 用户随后说明前次保险未打开，并在保持拆桨、未解锁状态下解除安全开关；35 秒复测得到 35/35 个 `ARMED=0` 心跳。
- `PreArm: Hardware safety switch` 已消失，当前只观察到 `PreArm: RC not found`；`SYS_STATUS.onboard_control_sensors_enabled` 从 1348476399 增至 1348509167，差值 32768，对应 motor outputs enabled 位，确认安全开关状态变化已被飞控识别。
- MTF-01P 数据持续正常：约 80 Hz 原始光流与测距仍来自 `1/88`，独立测距 0.17–0.20 m、均值 0.18599 m；光流质量均值 100.56/255。
- 10 秒静态复测 Y 方向仍出现最大约 0.401 m/s 的孤立尖峰，因此解除安全开关没有解决光流异常值，仍不能直接启用光流位置控制飞行。
- GPS 仍为 `fix_type=1`、0 星；电池监测仍无数据；EKF flags 仍为 231，无水平位置。保险解除只消除了硬件安全开关预检，不会自动配置 GPS、光流融合、电池监测或 RC。
- 全程 `PARAMETER_WRITE=0`、`ARM_COMMAND=0`、`MODE_CHANGE=0`。证据：`D:\Codex\UAV\output\follow_hardware_audit_safety_open_20260807.txt` 与 `mtf_static_audit_safety_open_20260807.json`。

## AprilTag 跟随拆桨命令预览（2026-08-07）

- 用户明确暂不进行起飞测试，并确认设备上电、标签可见；本阶段只计算和记录“拟发送速度”。
- 新增 `imx296_debug/follow_command_preview.py`：真实 IMX296 + tag36h11 ID0 + 名义 BODY_FRD 外参 + 真实 Pixhawk 接收遥测；串口代码只有接收，不调用 MAVLink send、参数、模式、解锁或电机接口。
- 启动前连续 5 个真实心跳确认未解锁；20.69 秒运行期间累计 26 个未解锁心跳、0 个解锁心跳。
- 351/351 帧检测有效、0 拒绝，实际约 16.97 Hz；距离均值 0.735861 m、标准差 0.000068 m。
- 标签相对机体均值 X=+0.042019 m（前）、Y=-0.004734 m（左），均在 0.05 m 死区内；拟发送速度峰值仅 0.000071 m/s，符合“标签居中时命令归零”的静态验收。
- 离线编码 351 个候选 `SET_POSITION_TARGET_LOCAL_NED` 包用于格式审计，真实发送 0；汇总明确记录 `mavlink_packets_transmitted=0`。
- 测试后已恢复 `http://192.168.1.11:8765/` 相机预览，主页和 snapshot 均 HTTP 200；无命令预览进程常驻。
- 静态居中项已通过；后续动态结果见下一节。
- 配置与证据：`D:\Codex\UAV\config\apriltag_follow_bench_preview_20260807.yaml`、`D:\Codex\UAV\output\follow_command_preview_20260807.jsonl`、summary JSON 和标注图。

### 动态方向与丢标验收

- 两次动态预览合计 100.97 秒、1747 帧、1406 次有效检测、0 次质量拒绝；共 112 个真实未解锁心跳、0 个已解锁心跳。
- 标签向机头方向移动时，目标 BODY_FRD X 最大 +0.113914 m，拟前向速度最大 +0.047155 m/s；向机体右侧移动时，目标 BODY_FRD Y 最大 +0.099909 m，拟右向速度最大 +0.079681 m/s。方向符号均正确，拟水平速度峰值 0.099271 m/s，低于 0.20 m/s 上限。
- 3 个遮挡区段均进入减速/保持，最长目标年龄 7.758 秒，末端速度为零。审计发现旧实现刚超过 0.70 秒边界时有 3 帧限加速度残余，最高 0.030277 m/s；现已让 `ACQUIRE`/`PREVIEW_HOLD` 绕过限加速度器、强制严格零速度并重置控制器。
- 修正后的代码在树莓派通过 15 项测试；Windows 与树莓派脚本 SHA-256 均为 `31c09cc9d15a03c6f0169deae439b92bca44750853ee35f2f1a3d1008348eb94`。动态日志形成于修正前，未冒充修正后的实时复测。
- 1747 个候选 MAVLink 包仅离线编码，真实发送 0；未写参数、未改模式、未解锁、未触发电机/起飞/降落。阶段 3 完成，但仍未批准飞行使用。
- 相机预览已恢复：主页与 `/snapshot.jpg` 均 HTTP 200；只有 `camera_stream.py` 常驻，无命令预览或降落桥进程常驻。
- 证据：`D:\Codex\UAV\output\follow_command_preview_dynamic_20260807.*`、`follow_command_preview_dynamic_right_loss_20260807.*` 与 `follow_command_preview_dynamic_analysis_20260807.json`。

### CH7 树莓派跟随许可开关（2026-08-07）

- AT9S Pro 姿态选择确认使用 `CH5 + SwC`；用户将 `CH7` 从旋钮 `VrC` 改为独立拨杆 `SwD`。
- Mission Planner 实测 SwD 上拨 `Radio 7=1000`、下拨 `Radio 7=2000`；冻结为上拨关闭、下拨许可。飞控 `RC7_OPTION=0` 保持不变。
- 新增失效关闭门 `imx296_debug/rc_follow_gate.py`：>=1800 才许可，<=1200关闭，中间值、无样本和超过0.5秒的旧样本全部关闭。
- 接收/预览工具已接入CH7，关闭时速度严格归零；状态机、预览零速和RC门共20项Pi端测试通过，仍无真实MAVLink控制发送。
- 部署后完成真实只读回归：初始上拨60/60样本为1000且门关闭；下拨60/60样本为2000且门许可；最终上拨40/40样本为1000且门关闭。三段共32个真实心跳全部未解锁，无状态告警。
- 最终SwD保持上拨1000，Windows/Pi SHA-256一致，无控制进程常驻。证据：`D:\Codex\UAV\config\rc_follow_authorization_20260807.json`。当前仅批准拆桨、未解锁、只收不发使用。
- 集成相机/RC循环已完成：上拨居中279/279帧和上拨偏移275/275帧均为`RC_DISABLED`且拟速度严格为0；下拨居中280/280帧有效、偏移277/277帧有效，偏移时拟速度方向正确且峰值0.028815 m/s。四组共84个真实飞控心跳全部未解锁，真实控制发送0。
- 测试结束后SwD仍为上拨1000，预览服务PID 2222已恢复，主页和`/snapshot.jpg`均HTTP 200；无命令预览或降落桥常驻。证据已复制至`D:\Codex\UAV\output\follow_rc_gate_*_20260807*`。
