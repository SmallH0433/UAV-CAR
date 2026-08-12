# 空地协同项目模块化分工计划

版本：2026-08-05
用途：将项目拆成可以并行分发给不同同学的独立任务包。

## 一、统一架构

~~~text
无人机：
IMX296 -> Raspberry Pi 4B -> AprilTag/位姿 -> LANDING_TARGET -> Pixhawk

无人车：
IMX219/其他传感器 -> Jetson -> ROS 2/AI/规划 -> CAN -> HUNTER

开发与仿真：
Windows + WSL Ubuntu 22.04 -> ROS 2/Humble/Gazebo/ArduPilot SITL

充电安全：
STM32/安全控制器 -> 预充/接触器/保护 -> 充电器/BMS
~~~

原则：

- Pi 的视觉结果只提供观测，Pixhawk 负责飞行安全和姿态控制。
- Jetson 属于无人车端，不把当前 Jetson Nano 当作无人机视觉主机。
- WSL 是开发和仿真环境，不是最终车载或机载部署位置。
- STM32 独立负责充电硬件联锁，Linux 进程不能成为唯一保护。
- 所有真实飞行、车辆运动、CAN 控制和充电输出都必须由总负责人单独批准。

## 二、模块分工

| 模块 | 建议负责人 | 平台 | 当前状态 | 可并行性 |
|---|---|---|---|---|
| M0 架构、接口和安全 | 总负责人/架构同学 | Windows/WSL | 已有初版，持续维护 | 所有人依赖 |
| M1 无人机视觉 | 同学 A | Raspberry Pi + IMX296 | 名义零外参 BODY_FRD 已完成真实拆桨台架；待 PLND 启用授权与飞行前复核 | 可与 M3/M5 并行 |
| M2 飞控与 MAVLink | 同学 B | Pixhawk + WSL SITL | SITL、TELEM1 双向台架和遥测流率已验证 | 依赖 M1 接口 |
| M3 无人车底盘与 Jetson | 同学 C | Jetson/WSL + HUNTER | ROS 2 驱动已编译；CAN 实车待硬件 | 可与 M1/M2 并行 |
| M4 空地协调器 | 同学 D | WSL ROS 2 | 尚未形成首版节点 | 依赖 M1、M3、M5 |
| M5 仿真、回放和测试 | 同学 E | WSL SITL/Gazebo | 心跳、合成 LANDING_TARGET 已完成 | 可持续并行 |
| M6 对接与充电安全 | 同学 F | STM32/机械/电气 | 只做方案和接口，暂不接高功率 | 可与软件并行 |
| M7 集成与质量验收 | 总负责人+各模块 | 全部 | 尚未开始 | 依赖 M1-M6 |

## 三、模块任务卡

### M0：架构、接口和安全

交付：

- 维护 docs/SYSTEM_ARCHITECTURE.md、docs/ADAPTATION_MATRIX.md 和本文件；
- 维护统一坐标、时间戳、日志字段和状态机；
- 审核所有模块是否越过安全边界；
- 维护版本、设备清单和验收记录。

验收：每个模块都有输入、输出、依赖、负责人、验收命令和回滚方法。

### M1：无人机本地视觉

平台：Raspberry Pi 4B + IMX296。

任务：

- 维护 Picamera2/libcamera 采集；
- 维护相机标定文件和标定数据；
- 使用 tag36h11 检测固定 ID；
- 输出相机光学坐标中的目标观测；
- 维护相机到机体 BODY_FRD 外参；当前采用用户指定的零平移、零 roll/pitch 名义值，真实飞行前再做物理复核；
- 在真实飞控接入前完成 dry-run 和回放测试。

已完成基线：

- IMX296 可出图；
- tag36h11 / ID 0 已成功识别；
- 已产生 43 条有效观测和 43 个 LANDING_TARGET dry-run 包。
- 135 mm 标签、run4_17mm 标定和距离修正的一致组合已完成实时台架验收；15 秒得到 45 条有效观测，距离均值 0.5553 m。
- 已在真实 TELEM1 上以 `CAMERA_OPTICAL + position_valid=0` 发送传感器消息；飞控保持 `PLND_ENABLED=0`、全程未解锁。
- 名义 BODY_FRD 变换已通过 4 个单元测试、43/43 离线回归、SITL 51 条和真实串口回放 49 条；最终真实实时台架在 `30 fps / 4 threads / quad_decimate=3.0` 下发送 179/179 条，实际 12.68 Hz，距离均值 0.55663 m，frame 12、`position_valid=1`。

禁止：名义 BODY_FRD 仅限拆桨、未解锁、PLND 关闭台架和 SITL；不得据此自行启用 PLND、解锁、装桨或真实飞行。

验收：检测率、距离误差、延迟和丢失策略都有 CSV 记录，坐标变换有单元测试。

### M2：Pixhawk 与 MAVLink

平台：ArduCopter 4.7.0、fmuv3 Pixhawk、WSL SITL。

任务：

- 维护 LANDING_TARGET 字段和频率契约；
- 在 SITL 验证 PLND_ENABLED、MAVLink backend 和失效策略；
- 验证消息超时、错误坐标、错误 ID、丢帧和遮挡；
- 编写真实飞控台架接入前检查表；
- 维护参数快照和 DataFlash/MAVLink 日志规范。

已完成台架基线：

- TELEM1/SERIAL1 已验证 MAVLink2、57600 baud 双向通信；
- ArduPilot 4.7 的 TELEM1 `MAV2_*` 流率已备份并配置；
- Pi 参数请求收到飞控回复，证明 Pi TX -> Pixhawk RX 上行有效；
- 真实相机消息旁路联调保持 `PLND_ENABLED=0`，没有解锁、模式或电机命令。

验收：SITL 静态/移动目标通过，BODY_FRD、position_valid、距离和时间戳检查通过。

### M3：无人车 Jetson、ROS 2 和 CAN

平台：Jetson Nano + IMX219；ROS 2 主工作区先放在 WSL Ubuntu 22.04。

任务：

- 维护 Jetson 相机采集和轻量通信；
- 维护 WSL 中的 ugv_sdk、hunter_msgs、hunter_base；
- 按 HUNTER 文档使用 can0、500 kbit/s；
- 设计只读状态、速度限制、看门狗和急停优先级；
- 仅在确认 USB-CAN 和 HUNTER 到货后进行 candump。

限制：Jetson Nano 当前为 Ubuntu 18.04/JetPack 4，不强行安装 ROS 2 Humble；当前没有 can0，不得模拟成真实底盘已连接。

验收：无硬件时完成编译和接口验证；有硬件时先只读 candump，再架空轮测试。

### M4：空地协调器

平台：WSL ROS 2。

任务：

- 定义 UavState、UgvState、RendezvousPlan、DockState；
- 实现任务分配、会合点、ETA、能量余量和失败降级；
- 不直接控制电机和充电接触器；
- 实现 IDLE、ASSIGNED、RENDEZVOUS、TRACK_PAD、DOCK_VERIFIED、READY 和 ABORT。

验收：仿真 100 次无死锁；任一平台失联时进入安全降级；会合失败不循环追逐。

### M5：仿真、回放和测试

平台：WSL SITL/Gazebo。

任务：

- 维护 ArduPilot SITL、Gazebo 和 ROS 2 测试脚本；
- 生成静态、移动、延迟、抖动、丢帧、错误 ID 和坐标错误数据；
- 回放 Pi 的 CSV 和 LANDING_TARGET JSONL；
- 统一保存 BIN、tlog、CSV、JSONL 和测试结果；
- 建立无真实硬件的回归测试。

验收：测试可重复，全程 ARMED=0，不连接真实硬件。

2026-08-07 补充：CH7 单开关模式仲裁已完成 23 项单元测试，并在仅监听 `127.0.0.1:14550` 的真实 ArduCopter SITL 中完成自动 GUIDED、CH7 关闭恢复、摇杆接管、模式拨杆接管锁存以及 CH7 循环重启用验证。SITL 中的软件飞行器最终 LAND 并上锁，未连接真实 Pixhawk；结果见 `output/follow_mode_manager_sitl_20260807_summary.json`。

### M6：对接、STM32 和充电安全

平台：STM32、安全控制器、机械对接和电气系统。

任务：

- 设计落位双重确认、桨停确认、预充、接触器和急停；
- 定义过压、过流、过温、反接、绝缘和预充超时故障；
- 设计 Linux 掉电时默认断开的硬件逻辑；
- 只做接口、状态机和低压空载验证。

禁止：未完成安全评审，不接真实 6S 电池和高功率充电输出。

验收：任意单一软件进程崩溃都不会导致接触器误闭合。

## 四、建议分发顺序

第一批可以立即并行分发：

1. 同学 A：M1，使用现有 Pi 标定和 AprilTag 结果，补坐标变换测试；
2. 同学 B：M5，完善 SITL 移动平台、丢帧和错误坐标回归；
3. 同学 C：M3，整理 HUNTER ROS 2 接口和 CAN 只读检查；
4. 同学 D：M4，设计空地状态消息和协调器状态机；
5. 同学 E：M6，只做 STM32 充电安全接口设计；
6. 同学 F：M2，整理 Pixhawk 台架接入清单，不连接真实飞控。

第二批依赖硬件到货：M1 机体外参、M2 拆桨台架、M3 USB-CAN 只读通信、M6 低压空载和机械对接。

## 五、统一提交格式

~~~text
模块编号：
负责人：
日期：
使用设备：
基于提交号/版本：
完成内容：
验证命令：
验证结果：
生成文件：
未完成项：
风险与回滚方法：
是否接触真实硬件：是/否
~~~

所有模块完成后，由 M0 和 M7 统一合并，不允许直接覆盖其他模块的工作区或配置。
