# Jetson + Pixhawk + Hunter 实机迁移指南

## 1. 先明确控制边界

迁移后的职责分配应保持如下，不建议让 Jetson 直接产生电机 PWM：

```text
AT9S Pro / 独立急停                     人工最高优先级与最终撤权
          │
          ├─ Pixhawk + ArduPilot         UAV 姿态、位置内环、解锁与 failsafe
          │      ▲ MAVLink 速度/任务请求
          │      │
          └─ Jetson + Ubuntu + ROS 2     感知、定位、规划、协同状态机、网页网关
                 │
                 ├─ UAV 相机/雷达/超声驱动
                 ├─ Nav2 / UAV 局部规划 / AprilTag
                 └─ Hunter CAN 驱动      UGV 执行端
```

如果无人机和无人车各有一块 Jetson，二者通过有线/无线 DDS 域或明确的网关交换任务与
相对位姿；如果先用单 Jetson 原型，必须测量最坏 CPU/GPU、内存、温度和传感器吞吐，
避免相机推理拖慢控制与心跳线程。

## 2. 仿真到实机不变与必须替换的部分

保持不变：

- ROS 2 话题语义、FLU/ENU 坐标约定；
- Nav2、无人机局部规划、感知融合、精准降落与任务状态机；
- command mux、命令超时、最大速度、空域和告警结构；
- 网页的只读状态模型与操作审计接口。

必须替换或重新标定：

- Gazebo 相机/LiDAR/IMU/超声波桥 → 厂商 ROS 2 驱动；
- ArduPilot SITL UDP → Pixhawk USB/UART MAVLink；
- Gazebo Ackermann 执行器 → Hunter CAN 驱动；
- 仿真 `DetachableJoint` → 机械甲板、缓冲、锁止、触点/霍尔/载荷检测；
- 仿真真时间与理想 TF → PTP/chrony 时间同步、实测外参和延迟；
- 仿真噪声参数 → rosbag 实测分布与最坏情况。

## 3. 建议硬件接口清单

| 子系统 | 最低接口要求 | 部署注意事项 |
|---|---|---|
| Jetson | Ubuntu、ROS 2、千兆网/USB3、可靠供电与散热 | 锁定功耗模式并记录降频；控制节点与 DNN 隔离资源 |
| Pixhawk | ArduPilot Copter、独立 RC、GPS/罗盘、UART/USB MAVLink | Jetson 不能绕过 pre-arm；设置 GCS/RC/battery/EKF failsafe |
| Hunter | 官方/兼容 ROS 2 驱动、隔离 CAN 适配器 | 确认车型、CAN 比特率、正方向、转角/速度范围、物理急停 |
| 云台相机 | 带时间戳 RGB 流与 CAN/PWM/串口角度控制 | 测云台零位、轴方向、角限位和实际反馈，不只使用命令估计 |
| 双目相机 | 同步左右目、内参/畸变、深度或视差 | 刚性基线、硬同步优先；重新标定并测试弱纹理/逆光 |
| 2D LiDAR | `LaserScan` 与可靠时间戳 | 安装面应水平；注意桨叶/机架自反射遮罩 |
| 3D LiDAR | `PointCloud2` 与每帧/每点时间 | 振动隔离、运动畸变补偿、网络带宽和 EMI |
| 超声波 | 六向 `Range` 或 MCU 聚合消息 | 交替触发防串扰；软物体、斜面和温度测试；只作近场冗余 |
| 降落板 | 大尺寸 AprilTag、漫反射表面、机械导向/缓冲 | 标签尺寸必须实测；避开反光、阴影与旋翼下洗 |
| 锁止机构 | 闭锁/释放命令、双路状态、机械应急释放 | “看到标签”不等于“已锁止”；必须有独立接触证据 |

具体型号尚未确定时不要在任务代码里硬编码厂商 API。先让驱动层适配下列统一契约。

## 4. ROS 2 实机契约

### 无人机

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav/odom` | `nav_msgs/Odometry` | ArduPilot/VIO 融合后的 ENU 本地位置 |
| `/uav/scan` | `sensor_msgs/LaserScan` | 水平 2D 扫描 |
| `/uav/lidar3d/points` | `sensor_msgs/PointCloud2` | 机体系三维点云 |
| `/uav/stereo/left/image_raw` | `sensor_msgs/Image` | 同步左目 |
| `/uav/stereo/right/image_raw` | `sensor_msgs/Image` | 同步右目 |
| `/uav/stereo/depth/depth_image` | `sensor_msgs/Image` | `32FC1` 米或 `16UC1` 毫米深度 |
| `/vision/image_raw` | `sensor_msgs/Image` | 下视精准降落图像 |
| `/uav/gimbal/image_raw` | `sensor_msgs/Image` | 云台主画面 |
| `/uav/range/<direction>` | `sensor_msgs/Range` | 六向超声波，方向名与仿真一致 |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 仲裁后的 FLU 速度，交给 MAVLink bridge |

### 无人车

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Nav2 障碍层输入 |
| `/ugv/imu/data` | `sensor_msgs/Imu` | 车载 IMU |
| `/ugv/wheel/odometry` | `nav_msgs/Odometry` | Hunter 原始里程计 |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF 输出 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 地图定位 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 输出 |
| `/ugv/cmd_vel` | `geometry_msgs/Twist` | 底盘适配器输出 |
| `/hunter_base/cmd_vel` | `geometry_msgs/Twist` | 最终 Hunter 驱动输入 |

真实驱动的 frame 名不同，应在驱动 launch 中 remap 并发布正确静态 TF，不要在感知
算法里写坐标补丁。

## 5. Jetson 软件安装与启动

推荐把以下源码放入同一 ROS 2 工作区：

```text
src/air_ground_sim
src/ugv_sdk
src/hunter_base
src/hunter_msgs
```

安装后先只启动驱动和只读节点。Hunter CAN 示例（比特率以实际底盘手册为准）：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
candump can0
```

实机主启动：

```bash
source /opt/ros/humble/setup.bash
source ~/air_ground_ws/install/setup.bash
ros2 launch air_ground_sim hardware_bringup.launch.py \
  map:=/absolute/path/to/site_map.yaml \
  mission_parameters:=/etc/air-ground/site-a-mission.yaml \
  can_port:=can0 ugv_adapter:=ackermann
```

该启动默认包含只读网页网关和保持 `IDLE` 的任务监督节点；`real_interfaces.yaml` 会拒绝
所有网页写命令。随包的 `real_mission.yaml` 明确标记为 `UNCOMMISSIONED`，因此即使误点
“开始”也会被拒绝。部署前复制该模板，完成地图坐标、空域、速度、停靠测距和机构包线
的现场验证，赋予可追溯 `mission_plan_id`，最后才可将
`mission_plan_validated` 改为 `true`。实机状态机固定使用
`simulation_lifecycle=false`，不会代替飞手完成解锁或起飞。

默认配置位于 `config/real_interfaces.yaml`：

- Pixhawk：`serial:/dev/serial0`，默认 57600 baud；
- UAV lifecycle 指令关闭，不能由协同状态机自动解锁/起飞；
- UAV 速度转发、UGV 适配器和 UGV 最终网关均默认关闭；
- Web 写命令关闭，令牌是必须修改的占位值；
- 数据超时为实机级毫秒范围，不采用仿真的低实时率放宽。

任务/场地配置独立位于 `config/real_mission.yaml` 安全模板中。每个站点和版本应保存为
只读、评审通过的部署文件；修改地图或锁止机构后必须生成新的计划 ID 并重新验证，不能
直接复用仿真坐标。`uav_navigation` 与 `web_gateway` 的禁飞/限高元数据必须保持一致；
前者执行约束，后者只负责向操作员显示，网页图形不得被当作飞控围栏。

连接设备后按实际持久设备名修改，例如使用 `/dev/serial/by-id/...`，避免 USB 枚举变化。

## 6. 坐标、时间与标定门槛

所有控制测试之前必须完成：

1. 坐标方向：ROS 机体为 FLU、世界为 ENU；MAVLink bridge 负责转换 BODY_NED/NED；
2. 相机内参：左右目和下视相机分别标定焦距、主点、畸变；
3. 双目标定：固定曝光下获取内外参，验证深度尺度，不沿用仿真 10 cm 基线参数；
4. 外参：测量 `base_link` 到 LiDAR、IMU、相机、云台零位和降落触点；
5. 近地视场：按起落架接触姿态验证完整 AprilTag（含静区）始终在下视相机视场内，不能
   只在悬停高度做标定；
6. 云台：标定编码器零位、实际反馈、机械限位及相机光轴；
7. 时间：Jetson、两车计算机与载荷使用 PTP 或 chrony；硬件触发设备优先；
8. 延迟：用时间戳测量采集→驱动→融合→控制的 P50/P95/P99，不只看平均帧率；
9. AprilTag：用实际打印尺寸、安装高度和相机分辨率重新设置 tag size/面积阈值；
10. 里程计：实测轮径、轴距、最大转角、CAN 命令比例和正负方向；
11. 空域：现场测绘后生成地图、围栏、禁飞区、限高区和返航/迫降区。

建议将每次标定文件带设备序列号、日期和校验和管理；安装位置变化即视为失效。

## 7. 分阶段放权

不要从完整仿真直接跳到移动平台自动降落。建议门槛如下：

1. **软件在环（SIL）**：完成本项目全状态机、故障注入、重复运行；
2. **飞控 HIL**：真实 Pixhawk 接仿真动力学，验证串口、模式、failsafe、时延；
3. **传感器回放**：用实机 rosbag 离线跑感知，量化漏检、误检和算力；
4. **拆桨/架空轮台架**：验证命令方向、超时、急停、断网、进程崩溃和重启；
5. **UGV 封闭场地**：0.1 m/s 直线、弯道、倒车、静态/动态障碍后逐步升速；
6. **UAV 保护网/系留**：人工起降、定高、避障与 RC 接管；
7. **静止甲板上方悬停**：只做相对定位，不下降；
8. **静止甲板软着陆**：先无锁止，再验证机械锁止和释放；
9. **低速跟车**：保持 3 m 以上高度，逐级测试 0.1/0.2/0.3 m/s；
10. **移动甲板接近**：设置最低复飞高度和随时 go-around，先不接触；
11. **移动着陆与联合运行**：只有前十项有可复现记录后启用，并逐步扩大包线。

每一级都应定义通过率、最大偏差、最小障碍距离、丢帧率、接管时间和 abort 行为；
一次成功演示不能替代统计验收。

## 8. AT9S Pro、急停与失效保护

一个 AT9S Pro 可以通过接收机通道同时服务 UAV 和 UGV，但不能只靠软件“切换对象”
作为唯一安全边界。推荐：

- 四主通道始终控制 UAV；
- 独立三段开关表示 `UAV 手动 / 自主监督 / UGV 遥控`；
- UGV 物理急停独立于 Jetson、ROS 2 和无线网络；
- UAV RC 链路直接进入 Pixhawk，RC loss 由 ArduPilot 执行预设策略；
- 模式切换前要求油门/车速中位，使用明确灯光/蜂鸣反馈；
- 网页和平板只能作为监督与低优先级任务入口，不能替代 RC 与急停。

必须逐项测试：RC 失联、Jetson 断电、ROS 节点退出、DDS 断网、CAN 断线、MAVLink
断线、GPS/VIO 失效、任一相机/LiDAR 超时、电池低压、标签丢失和锁止状态矛盾。
再次起飞还必须联锁三个独立证据：机械锁扣已释放、飞控
`MAV_LANDED_STATE_ON_GROUND`、接触/载荷传感器正常；任一证据未知或矛盾时保持禁止解锁。

## 9. iPad / 电脑操作台部署

iPad 浏览器可以作为操作终端，不需要在 iPad 上运行 ROS 2。推荐网络结构：

```text
iPad Safari ── WPA2/3 专用局域网 ── HTTPS 反向代理 ── Web 前端
                                                   └─ 本机 ROS Web Gateway
```

实机上线前：

- 将 `web_gateway.command_enabled` 保持关闭，直至完成认证和现场验收；
- 设置高熵 `auth_token`，网关只监听受控接口，使用 TLS；
- CORS 限制到操作台来源，网络防火墙禁止公网直接访问 8765；
- 写操作记录操作者、时间、任务状态与结果；
- 页面失联不应让车辆继续接受遥控速度；按住式 UGV teleop 必须有 300 ms 看门狗；
- 保留本地笔记本/有线维护口，平板没电或 Wi-Fi 拥塞时仍可安全停止。

电脑更适合调参、RViz、rosbag 和诊断；iPad 更适合现场监督。二者可以同时使用，
但只允许一个明确的控制租约持有者。

## 10. 移动平台着陆的额外工程要求

这是整套系统风险最高的环节。实机至少还需：

- 比旋翼投影更安全的甲板尺寸、圆角与防卷入结构；
- 视觉标签之外的相对高度源和接触/载荷确认；
- 导向锥、磁吸或机械锁止，以及断电安全状态和人工释放；
- 限制 UGV 加速度、角速度、坡度和甲板振动的着陆许可协议；
- 明确的 go-around 条件：标签超时、横向误差、下沉率、甲板运动、风和定位质量；
- GNSS/罗盘与车体电机、磁吸机构之间的电磁兼容试验；
- 旋翼停转确认后才允许 UGV 正常加速；释放前确认锁止完全打开。

`/uav/dock/attach` 和 `/uav/dock/detach` 在仿真中是物理关节事件；实机由
`docking_hardware_gateway` 转换成受互锁保护的锁止动作。该网关已进入
`hardware_bringup.launch.py` 和系统 readiness，默认 `command_enabled=false`，接口如下：

| 方向 | 话题 | 类型 | 语义 |
|---|---|---|---|
| Jetson → 锁止 MCU/PLC | `/dock_hw/lock_command` | `std_msgs/Bool` | `true` 锁止、`false` 释放；执行端自身仍须有硬件限位和超时 |
| 锁止 MCU/PLC → Jetson | `/dock_hw/contact_a`、`contact_b` | `std_msgs/Bool` | 两条独立接触链路，不允许在软件外并成一个信号 |
| 锁止 MCU/PLC → Jetson | `/dock_hw/locked_a`、`locked_b` | `std_msgs/Bool` | 两条独立锁舌到位链路 |
| 网关 → 任务 | `/uav/dock/detached` | `std_msgs/String` | 仅由一致且新鲜的硬件反馈产生 `attached` / `detached` |
| 网关 → 监督器 | `/uav/dock/hardware_status` | JSON `std_msgs/String` | 健康、故障码、反馈和动作年龄 |

静止锁止要求双路接触、UGV 静止、飞控落地且未解锁；移动捕获要求双路接触、低车速、
低相对高度和飞控 `LAND` 包线。释放只允许在显式释放状态且两车安全时进行。重复任务请求
不会刷新第一次动作的超时起点。任一反馈超时、双通道矛盾、锁住后接触丢失、执行超时或
确认后意外变位都会进入关键故障。通用 abort、进程退出和服务禁用都不会主动解锁承载中
的机构。

联调时必须先保持桨叶拆除和车轮架空，使用独立测试仪逐路翻转四个反馈，验证交叉短路、
断线、粘连和乱序反馈都不能产生误锁/误释放；再用假负载验证执行超时和失电安全状态。
只有保存示波器/总线记录、ROS 日志和机构 FMEA 后，才可在站点覆盖配置中启用该网关。

任务层比例限速通过 Nav2 标准 `/speed_limit` 在闭环速度平滑器上游执行；
`/ugv/speed_scale` 仅作为底盘适配器的最终 0/1 安全门控。不要在闭环平滑器之后再做
比例缩放，否则低速时会因里程计反馈反复重置加速度积分而产生蠕动或停滞。以上两层都不
代替 Hunter 驱动器、电机控制器和物理急停中的硬限制。实机应验证限速后曲率不变，并把
移动对接速度、甲板角速度和加速度上限纳入着陆许可；任何通信超时都应退回底盘本地限速
或停车。

移动下降许可使用带动态时间戳检查的 TF `map→base_link` 获取地图帧位置/航向，
`/amcl_pose` 只作短时回退，并使用 `/odometry/filtered` 的横摆角速度；三者不可混用
坐标系。站点配置中的航向误差、角速度和位姿超时必须由实际定位频率、甲板尺寸、
飞行控制误差和风险评估确定。联合运行末段按地图帧剩余距离逐步下调 Nav2 限速，局部规划
停滞不会继续按墙钟把速度降到 Ackermann 无法转向的蠕动区。

## 11. 实机上线判据

只有下列条件同时满足，才称为“具备实机试验条件”，仍不等于产品认证：

- 所有驱动持续运行、时间戳单调、TF 无冲突；
- 最坏负载下 Jetson 无降频导致的控制超时；
- ArduPilot 正常 pre-arm，无 force-arm；
- UGV/UAV 命令方向、尺度、饱和和 watchdog 经过物理验证；
- 人工 RC 接管和物理急停在最坏网络条件下可用；
- 空域/地图/障碍参数来自现场测量；
- 静止落车、锁止、释放和复飞重复测试达到预设成功率；
- 双通道锁止 I/O 已完成断线、短路、粘连、反馈矛盾和动作超时 HIL；
- 移动落车具备独立 abort/go-around 包线；
- 有现场安全员、隔离区、检查表、日志和事故处置流程。

本仓库提供的是面向迁移的软件基线和验证路径，不替代飞行许可、风险评估、机械设计、
电气认证或现场操作规程。
