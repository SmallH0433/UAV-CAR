# Raspberry Pi–Pixhawk 精准降落台架交接

日期：2026-08-06  
范围：拆桨、未解锁、精准降落功能关闭的真实硬件台架。

## 已验收结果

- Raspberry Pi 4B 的 `/dev/serial0` 对应 `ttyAMA0`，GPIO14/15 为 TXD0/RXD0。
- Pixhawk TELEM1 为 MAVLink2、57600 baud，Pi 可接收心跳、姿态、系统状态和 RC 通道。
- Pi 发起 `PARAM_REQUEST_LIST` 后收到飞控参数，证明 Pi TX -> Pixhawk RX 上行有效。
- ArduCopter 4.7 将旧 `SR1_*` 重命名为 TELEM1 对应的 `MAV2_*`。
- 已配置 `MAV2_EXT_STAT=2`、`MAV2_RC_CHAN=5`、`MAV2_POSITION=3`、`MAV2_EXTRA1=10`、`MAV2_EXTRA2=3`、`MAV2_EXTRA3=3` Hz。
- 一致视觉组合为：IMX296 1456x1088、`imx296_calibration_run4_17mm.yaml`、tag36h11 ID 0、实测黑色有效边 0.135 m、`range_correction_20260806.json`。
- run4 标定板实测约 11.9x17 cm、10x7 格，即单格 17 mm；这解释了 `run4_17mm` 的标定尺度。
- 四点距离修正为 `scale=0.613586842370`、`offset=-0.010204131220 m`，最大绝对残差约 4.4 mm。
- 四点 raw->true 为：0.379759896->0.220 m、0.914040000->0.555 m、1.113300000->0.675 m、1.440467167->0.870 m。
- 0.87 m 点的新拟合结果来自 39 帧旧数据的离线复算，不是四点参数更新后的新实时重采样；不能将其表述为实时验收。
- 15 秒实时验收得到 45 条有效观测和 45 条 `LANDING_TARGET`；距离均值 0.5553 m，对应台架实测 0.555 m。
- 飞控始终保持 `PLND_ENABLED=0`、`PLND_TYPE=0`；消息采用 `MAV_FRAME_CAMERA_OPTICAL(27)`、`position_valid=0`。
- 只读还观察到 `PLND_ORIENT=25`、`PLND_LAG≈0.02`；当前仅记录，不采用也不修改。
- 用户随后明确指定名义外参：相机中心等同机体/Pixhawk 参考中心、平移为零、roll/pitch 为零，机头位于画面左侧；采用 `BODY_FRD=(-x_camera,-y_camera,+z_camera)`。
- 名义 BODY_FRD 链路已通过 4 个单元测试、43/43 离线回归、SITL 发送 51 条、真实 TELEM1 回放 49 条和 15 秒实时 46/46 条验收。
- 实时 BODY_FRD 验收使用 frame 12、`position_valid=1`，距离均值约 0.55522 m；全程 `ARMED=0`、`PLND_ENABLED=0`、`PLND_TYPE=0`，没有发送控制命令。

## 安全启动命令

Pi 上运行：

```bash
DURATION_S=15 /home/PI/uav/bin/run_landing_bench.sh
```

默认运行保守的 CAMERA_OPTICAL 配置。若要重复已验收的名义 BODY_FRD 拆桨台架，可使用：

```bash
FRAME_PROFILE=body-frd-nominal DURATION_S=15 /home/PI/uav/bin/run_landing_bench.sh
```

真实飞控已启用 PLND 后，只允许显式使用已验收配置：

```bash
FRAME_PROFILE=body-frd-plnd-enabled DURATION_S=15 /home/PI/uav/bin/run_landing_bench.sh
```

启动器会依次：

1. 连续读取 3 个飞控心跳并要求全部未解锁；
2. 根据显式 profile 只读确认 PLND 为 `0/0` 或 `1/1`，不匹配即拒绝；
3. 如果相机预览进程占用 IMX296，则拒绝运行；
4. 仅发送 `LANDING_TARGET` 传感器消息；
5. 保存 JSONL 和标注图到 `/home/PI/uav/logs/`。

启动器不会写参数、改变模式、解锁、上锁或发送电机命令。

## 主要文件

- `config/uav_precision_landing_bench.yaml`：冻结的台架配置与安全门。
- `imx296_debug/landing_target_serial_bridge.py`：实时检测和串口消息桥。
- `imx296_debug/landing_observer.py`：AprilTag 检测、质量筛选和位姿估计。
- `imx296_debug/mavlink_landing_target.py`：MAVLink 消息编码。
- `imx296_debug/imx296_calibration_run4_17mm.yaml`：当前相机内参。
- `imx296_debug/range_correction_20260806.json`：四距离点线性修正。
- `imx296_debug/imx296_body_extrinsics_20260806.json`：用户指定的名义零平移、零倾角外参。
- `imx296_debug/landing_target_live_consistent_20260806.jsonl`：15 秒实时验收日志。
- `imx296_debug/landing_target_live_consistent_20260806.jpg`：最新标注图。
- `imx296_debug/landing_target_body_frd_live_nominal_20260806.jsonl`：真实串口名义 BODY_FRD 实时验收日志。
- `imx296_debug/landing_target_body_frd_live_nominal_20260806.jpg`：名义 BODY_FRD 实时验收标注图。
- `config/precision_landing_activation_plan_20260806.yaml`：已执行的 PLND 参数启用、复核与回滚方案。
- `config/precision_landing_before_activation_20260806.json`：真实飞控 PLND 启用前备份。
- `imx296_debug/landing_target_body_frd_plnd_enabled_final_20260806.jsonl/.jpg`：PLND 1/1 下最终未解锁验收证据。
- `telem1_mav2_stream_backup_20260806.json`：TELEM1 流率修改前备份。

## 当前禁止跨越的门槛

- 名义 `BODY_FRD + position_valid=1` 只允许在拆桨、连续确认未解锁的真实台架，或在 SITL 中使用；启动器会根据 profile 强制匹配 PLND 0/0 或 1/1。
- 名义外参是用户指定的零平移/零倾角工程假设，不是物理测量；因此尚不批准真实飞行、自动降落或装桨测试。
- 真实串口脚本仅在读取到真实飞控 system 1/component 1 未解锁，并确认 PLND 状态与显式 profile 完全一致时允许发送；否则立即拒绝。
- `PLND_ENABLED=1`、`PLND_TYPE=1` 已获用户明确授权并完成未解锁台架验证；这不构成解锁、飞行或自动降落授权。
- 禁止装桨台架测试、自动解锁、起飞或真实降落。
- 相机预览和视觉桥不能同时占用 IMX296。
- `camera_stream.py` 已保留 RGB/BGR 通道修复，避免预览肤色发蓝；不要恢复旧的重复通道交换代码。

## 后续可选的物理复核

需要在机体保持水平、机头方向明确时记录：

- 物理复核图像上方是否确实对应机体右侧；
- 相机光心相对飞控/机体参考点的前后偏移，单位 mm，向前为正；
- 左右偏移，单位 mm，向右为正；
- 上下偏移，单位 mm，向下为正；
- 相机是否垂直向下，以及估计的 roll/pitch/yaw 安装误差。

这些数据可用于将“名义外参”升级为“实测外参”。当前用户选择先按零偏移、零倾角继续，因此它不是拆桨台架的阻塞项，但仍是批准真实飞行前的风险复核项。

## 2026-08-07 电池与基础传感器补充验收

- 用户通过标签确认动力电池为 6S LiPo、22.2 V、6000 mAh。
- TELEM1 安全门确认真实飞控 `ARMED=0` 后，已写入并回读：`BATT_CAPACITY=6000`、`BATT_LOW_VOLT=21.6`、`BATT_CRT_VOLT=21.0`、低电压动作为 RTL、严重低电压动作为 Land、`BATT_ARM_VOLT=22.2`。
- 未启用 mAh 容量保护，未修改电压/电流倍率；正式飞行前必须用充电器或万用表核对电压，并通过已知负载校准电流。
- 20 秒复测得到电池 22.372 V、0.14 A、fault bitmask 0；20/20 心跳未解锁，无状态告警。测距 0.77 m，光流地距 0.785 m、质量 110。
- GPS 为 `fix_type=1`、0 星，仍是当前明确的飞行阻塞项。证据见 `config/pixhawk_battery_6s_6000_20260807.json`。

## 用户选择的零外参假设

用户于 2026-08-06 明确选择跳过物理外参复核，并在后续离线/SITL 阶段采用：

- 相机光心相对机体参考点平移：`X=0、Y=0、Z=0 m`；
- 安装 roll/pitch：`0°/0°`；
- 机头位于画面左侧，对应离线映射 `BODY_FRD=(-x_camera,-y_camera,+z_camera)`。

这些值属于用户指定的工程假设，不是实测外参。它们已用于离线、SITL、PLND 关闭台架和 PLND 1/1 未解锁台架；不等于批准真实飞行。

## PLND 启用前只读快照

2026-08-06 通过 Pi TELEM1 逐项只读确认，真实飞控 system 1/component 1 保持 `ARMED=0`：

- `PLND_ENABLED=0`、`PLND_TYPE=0`、`PLND_EST_TYPE=1`、`PLND_OPTIONS=0`；
- `PLND_ORIENT=25`、`PLND_LAG=0.02`、`PLND_YAW_ALIGN=0`；
- `PLND_CAM_POS_X/Y/Z=0/0/0 m`；
- `PLND_LAND_OFS_X/Y=0/0 m`；
- `PLND_STRICT=1`、`PLND_ALT_MIN=0.75 m`、`PLND_ALT_MAX=8 m`、`PLND_XY_DIST_MAX=2.5 m`。

此次快照未修改任何参数，作为启用前基线保留。启用方案随后获用户明确授权并执行，状态现为 `applied_disarmed_bench_validated`。

官方 fmuv3 stable 4.7.0 的 `features.txt` 同时包含 `AC_PRECLAND_ENABLED` 和 `AC_PRECLAND_MAVLINK_ENABLED`，因此当前板型/固件具备 MAVLink 精准降落后端；启用顺序仍按官方建议先 `PLND_ENABLED=1` 并重启，再选择 `PLND_TYPE=1`。

## 检测性能配置

全分辨率未标注原始帧的离线基准显示：原配置 `2 threads / quad_decimate=1.0` 约 `5.21 Hz`；`4 threads / quad_decimate=2.0` 约 `11.11 Hz`；最终配置 `30 fps / 4 threads / quad_decimate=3.0` 离线约 `16.55 Hz`。修复实时循环中“处理时间后再固定 sleep”的调度问题后，真实 BODY_FRD 台架 15 秒得到 179/179 条，消息时间戳跨度 14.036 秒，实际 `12.68 Hz`。距离均值 `0.556631 m`，X/Y/Z 均值为 `0.075942/-0.014375/0.551239 m`，Hamming 全为 0，平均 decision margin 约 69.70。证据为 `imx296_debug/landing_target_body_frd_final_10hz_20260806.jsonl/.jpg`。

## PLND 启用与最终未解锁验收

- 用户明确授权在拆桨、未解锁台架启用 PLND；写入前曾检测到真实飞控 `ARMED=1`，安全门拒绝所有操作，人工上锁后才继续。
- 启用前完整备份为 `config/precision_landing_before_activation_20260806.json`。
- 按官方顺序写入 `PLND_ENABLED=1`，未解锁重启，再写入 `PLND_TYPE=1`；两项均回读一致。
- 重启前存在 `PreArm: Gyros not calibrated`；保持机体静止重启后，陀螺仪健康 `25/25`、加速度计健康 `25/25`，无需六面校准。
- 最终 PLND 1/1 实时台架 15 秒检测并发送 177/177 条，时间跨度 14.060488 秒，实际 `12.517346 Hz`。
- frame 12、`position_valid=1`、Hamming 全 0；距离均值/范围 `0.369593991 / 0.369584097–0.369609837 m`，X/Y/Z 均值 `0.025391880 / -0.007223798 / 0.368649952 m`。
- 全程未解锁，没有模式、解锁、电机、起飞或降落命令；测试后再次确认 `ARMED=0`、PLND 1/1，预览恢复 HTTP 200。

## 无桨真实命令链路台架

- 用户明确确认螺旋桨已拆除、电机周围人员/工具/线缆已清空，并授权短时无桨解锁。
- 第一次正常解锁被 `PreArm: Hardware safety switch` 拒绝；没有绕过检查，也没有使用 force 值 `21196`。人工解除安全开关后才重试。
- 第二次正常解锁虽未观察到 COMMAND_ACK，但真实飞控 heartbeat 明确进入 `ARMED=1`；最多约 2 秒后发送正常上锁，`DISARM_ACK=0`。
- 解锁窗口采集到 5 组四路输出，均为 `[1100,1100,1100,1100] us`；这证明飞控输出链路进入解锁状态，不等同于确认各电机物理旋转。
- `MAV_CMD_NAV_TAKEOFF` 仅在上锁状态发送并返回 ACK 4（失败/拒绝），没有起飞动作。
- `MAV_CMD_NAV_LAND` 仅在上锁状态发送并返回 ACK 0，模式短暂进入 LAND（custom mode 9）后由脚本恢复 STABILIZE（mode 0）。
- 脚本结束后连续 5 次、独立复核连续 8 次真实飞控心跳均为 `DISARMED + STABILIZE`；PLND 仍为 1/1。
- 完整证据：`config/pixhawk_command_bench_20260806.json`。没有在解锁状态发送起飞或降落命令，真实飞行仍未授权。

## 无桨逐电机 Motor Test

- 用户确认动力电池已连接、ESC 上电完成、螺旋桨已拆除且电机周围净空。
- 仅发送 `MAV_CMD_DO_MOTOR_TEST`；没有发送解锁、模式、起飞、降落或 actuator override，也没有使用 `21196`。
- 5%/1050 us 的 Motor 1 预试命令被接受，但低于当前 `MOT_SPIN_MIN=15%`。
- 正式测试按 15%、每个 1 秒依次完成 Motor 1–4，四路 ACK 均为 0，飞控均报告测试开始和结束。
- 实测输出对应关系：Motor 1 -> SERVO1、Motor 2 -> SERVO4、Motor 3 -> SERVO2、Motor 4 -> SERVO3；活动通道峰值均为 1150 us，非活动通道为 1000 us。
- Motor Test 执行窗口内飞控会短暂报告 armed 标志；每路结束后连续 3 次确认未解锁，全部结束后另行连续 8 次确认 `ARMED=0`。
- 证据：`config/pixhawk_motor_test_spin_20260806_222803.json`；脚本：`pi_pixhawk_motor_test.py`。电气输出已验证，物理旋转和方向必须由现场人员目视确认后记录。

## 2026-08-07 CH7 单开关跟随模式仲裁

- 新增 `imx296_debug/follow_mode_manager.py`，把 CH7/SwD 定义为唯一的正常跟随授权：1000 为关闭，2000 为申请跟随。
- CH7 打开后，仅在已解锁、传感器/目标/高度/电池等前置条件有效且进入模式为 `ALT_HOLD`、`LOITER` 或 `POSHOLD` 时，才请求 `GUIDED`；必须收到 3 个独立飞控心跳确认 `GUIDED` 后才能放行速度设定值。
- CH7 关闭时先停止跟随，再恢复进入跟随前的模式。系统不自动解锁、不自动起飞、不自动降落。
- 飞行模式拨杆使飞控离开 `GUIDED` 时，判定为人工接管：立即禁止跟随且不与飞手争抢模式；CH7 必须先回 1000，再到 2000 才能重新申请。
- 横滚/俯仰/偏航任一摇杆相对中位持续偏离至少 150 us、达到 0.20 s 时，也触发人工接管，恢复进入前模式并锁存。油门不参与该检测，避免不同飞行模式下中位定义不一致。
- GUIDED 申请超时、前置条件丢失或从未批准的模式申请跟随时均失败关闭，不发送运动速度。
- 单元测试 `23/23` 通过。真实 ArduCopter SITL 验证通过：自动 GUIDED、CH7 关闭恢复、摇杆接管锁存、模式拨杆接管不争抢、CH7 循环后重新启用、最终 LAND 并上锁；证据为 `output/follow_mode_manager_sitl_20260807_summary.json` 和对应 JSONL。
- 上述结果验证了模式仲裁模块和 SITL 集成。当前 Pi 的 `follow_command_preview.py` 仍明确为接收/预览工具，不发送运动命令；在真实试飞前，必须使用单独的飞行运行器接入本模块并完成只读启动检查，不能把 preview 工具误当成飞行控制程序。

## 2026-08-07 常开相机与跟随条件监控

- Pi 已启用用户服务 `apriltag-follow-monitor.service`，持续打开 IMX296 并提供 `http://192.168.1.11:8765/` 预览和 `/status.json` 状态。
- 当前部署是明确的 `MONITOR_ONLY_CONTROL_LOCKED`：MAVLink 只接收，配置 `control_enabled=false`，不发送模式、速度、解锁、电机、起飞、降落命令。
- 部署后真实只读状态为：未解锁、STABILIZE、CH7=1000、EKF flags=367、电池约 22.90 V、MTF 约 0.16 m、光流质量约 95–104、EKF 原点无效、当前未捕获标签。
- 只有 LOITER/GUIDED、CH7 遥测新鲜、EKF/电池/光流/测距健康、有效 EKF 原点、高度 0.55–0.85 m、标签已连续捕获且新鲜时，监控状态才会报告 `ready_for_follow_request=true`。报告 true 也只是传感器条件满足，不代表当前只读服务会发送控制。
- 当前禁止直接开始 AprilTag 跟随试飞。必须先完成光流方向/尺度的低空飞行校准与 Loiter 定点验收，并取得近似场地坐标设置 EKF 原点；之后才能单独审核和部署真实飞行运行器。

室内 EKF 原点已随后设置完成：用户场地坐标 `22.1350000, 113.5447222`、参考海拔 0 m；`AHRS_OPTIONS=24` 使 ArduCopter 4.7 记录并在无 GPS 启动时恢复原点。安全重启后飞控实际发布 `22.1350000, 113.5447296`，误差约 0.8 m，`origin_valid=true`。这只清除了 GUIDED 所需的原点阻塞，不代表 GPS 获得定位或光流首飞校准完成。
