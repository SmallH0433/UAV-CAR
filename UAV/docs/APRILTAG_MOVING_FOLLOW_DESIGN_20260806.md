# 移动小车 AprilTag 低空跟随方案

日期：2026-08-06  
适用平台：Pi 4B + IMX296 + Pixhawk/ArduCopter 4.7 + tag36h11 ID 0  
当前状态：方案设计；尚未批准装桨或真实自主飞行

## 0. 已确认硬件（2026-08-07）

- GNSS：QSDZ M9N 模块，用户口述型号为 NEI-M9N，按照片标识和产品系列判断核心接收机应为 u-blox NEO-M9N；它是多星座米级 GNSS，不是 RTK。
- 低空传感器：MicoAir MTF-01P，集成下视光流和短距激光测距。
- 飞控电源模块：已物理安装；但 2026-08-06 飞控只读结果仍为 `BATT_MONITOR=0`，尚未证明电压/电流数据和低电保护有效。
- 机腹计算与视觉：Raspberry Pi 4B、下视 IMX296；照片确认两者已固定安装。
- 当前照片中未安装螺旋桨。

建议接口分配：GPS 使用 Pixhawk GPS 口；Pi 占用 TELEM1；MTF-01P 应直接接空闲遥测串口（通常 TELEM2），不通过 Pi 转发。实际端口和参数必须在上电后只读确认，不能仅凭照片推断。

## 1. 目标与边界

目标是在无人机人工起飞并稳定悬停后，使用下视 IMX296 识别小车上的 AprilTag，使无人机以固定低空高度跟随标签的水平移动。后续可在独立验收后切换到移动平台精准降落。

第一版明确不做：

- 自动解锁、自动起飞；
- 目标丢失后盲目搜索或自动降落；
- 通过 Wi-Fi 远程闭环控制无人机；
- 将当前名义零外参直接视为飞行级实测外参；
- 在没有可靠 EKF 水平位置源时使用 GUIDED 速度控制；
- 同时修改精准降落和跟随控制参数。

## 2. 现有代码可复用能力

现有 `imx296_debug` 已完成：

- IMX296 1456×1088、30 fps 采集；
- `pupil_apriltags` 的 tag36h11/ID 0 检测；
- 13.5 cm 黑色有效边尺寸；
- `imx296_calibration_run4_17mm.yaml` 相机内参；
- `range_correction_20260806.json` 距离修正；
- 相机光学系到 BODY_FRD 的名义转换；
- 真实 Pi–Pixhawk TELEM1，57600 baud；
- 约 12.5 Hz 的实时有效检测和 `LANDING_TARGET` 发送。

需要保留而不直接修改的安全基线：

- `landing_target_serial_bridge.py` 继续作为未解锁精准降落台架桥；
- `mavlink_landing_target.py` 继续负责 `LANDING_TARGET` 编码；
- 新跟随控制器单独建立，不能解除旧桥的未解锁安全门。

当前代码还不具备飞行跟随能力，原因包括：

- 仅发送 `LANDING_TARGET`，没有 GUIDED 速度设定值；
- 串口桥检测到解锁会主动退出；
- 没有目标状态滤波、速度估计和异常值拒绝；
- 没有目标丢失、模式退出、RC 接管和控制超时状态机；
- 图像时间戳取在检测完成后，不是传感器曝光时刻；
- 当前外参文件明确标记 `flight_use_approved=false`；
- 距离修正只在约 0.22–0.87 m 范围内实测，不能直接外推到更高跟随高度。

## 3. 推荐总体架构

```text
小车 AprilTag
      ↓ 图像（本地，不走 Wi-Fi）
IMX296 → Pi AprilTag 检测 → 质量门控 → 目标状态滤波
                                      ↓
                          相对位置/速度控制器（10 Hz）
                                      ↓
               SET_POSITION_TARGET_LOCAL_NED（TELEM1）
                                      ↓
                         Pixhawk GUIDED 位置控制器
                                      ↓
                                  无人机

Jetson/小车里程计 ──UDP──→ 可选速度前馈
Windows/Mission Planner ─→ 只监控、记录和人工接管
RC 遥控器 ──────────────→ 最高优先级模式退出/人工控制
```

飞行关键闭环是 `IMX296 → Pi → TELEM1 → Pixhawk`。Jetson 和无线网络断开时，系统必须仍能减速悬停并允许 RC 接管。

## 4. 为什么不能只用精准降落

`LANDING_TARGET + PLND` 适合 Precision Loiter 和 Land/RTL 最终降落。ArduPilot 支持将 `PLND_OPTIONS` bit 0 用于移动着陆目标，但精准降落控制器的目标是下降和接地，不是长期低空编队跟随。

推荐拆成两个独立功能：

1. `FOLLOW`：GUIDED 模式下发送水平速度，保持固定高度；
2. `PRECISION_LAND`：完成跟随验收后，人工切换 LAND，再由 `LANDING_TARGET` 和移动目标选项负责下降。

第一版只实现 `FOLLOW_XY`，高度由 Pixhawk 保持；不要同时启用视觉垂直速度和精准降落。

## 5. 坐标与目标状态

AprilTag 检测输出相机光学系：

```text
x_camera：画面向右
y_camera：画面向下
z_camera：镜头向外，即朝地面
```

当前名义安装给出：

```text
BODY_FRD = (-x_camera, -y_camera, +z_camera)
```

飞行前必须物理复核该方向，并测量相机光心相对机体中心的 X/Y/Z 偏移。跟随控制只使用通过飞行前复核的 `p_tag_body=[forward,right,down]`。

优先控制方案需要 FC 提供：

- `LOCAL_POSITION_NED`：无人机局部位置和速度；
- `ATTITUDE_QUATERNION`：机体到 NED 的旋转；
- `HEARTBEAT`：模式、解锁状态；
- `SYS_STATUS`/电池状态；
- 可选 `DISTANCE_SENSOR`：真实离地高度。

每次观测计算：

```text
p_target_ned = p_vehicle_ned + R_body_to_ned · p_tag_body
```

使用常速度 alpha-beta 或 Kalman 滤波器估计 `p_target_ned` 和 `v_target_ned`。滤波器必须使用相机曝光时间戳，并补偿检测延迟。

## 6. 第一版控制律

### 6.1 水平跟随

无人机期望位于标签正上方，水平偏移为零：

```text
e_xy = p_target_ned.xy - p_vehicle_ned.xy
v_cmd_xy = v_target_est.xy + Kp_xy · e_xy
```

初次 SITL 建议值：

- `Kp_xy = 0.4 1/s`；
- 不启用积分项；
- 位置死区 `0.05 m`；
- 首次实飞速度上限 `0.20 m/s`；
- 首次实飞加速度上限 `0.20 m/s²`；
- 目标速度估计上限 `0.50 m/s`；
- 控制输出频率 `10 Hz`。

先使用 P 控制，稳定后再加入目标速度前馈。不要直接照搬旧 PX4 项目中的积分项和 `1.5 m/s` 限幅。

### 6.2 高度

第一版 `FOLLOW_XY` 发送 `vz=0`，由 Pixhawk 的高度控制器保持起飞后的固定高度。推荐初始标签上方高度约 `0.8 m`，但必须先补做该高度附近的距离标定和视野测试。

第二版 `FOLLOW_XYZ` 才允许使用：

```text
v_down_cmd = Kp_z · (z_tag - z_desired)
```

建议初始 `|v_down_cmd| ≤ 0.10 m/s`，且必须由激光测距仪和绝对高度上下限双重约束；不能只依赖单目标签距离控制高度。

### 6.3 MAVLink 输出

使用：

- 消息：`SET_POSITION_TARGET_LOCAL_NED`；
- 模式：`GUIDED`；
- 坐标：优先 `MAV_FRAME_LOCAL_NED`，输出 NED 速度；
- 仅启用 `vx/vy/vz`，保持 yaw 不变；
- 发送频率 10 Hz；
- 应用层目标观测超时显著短于 ArduPilot 的 `GUID_TIMEOUT`。

如果尚无可靠 EKF 水平位置源，不得使用该方案。室外可用 GPS；室内需要 Optical Flow + Rangefinder 或经过单独验收的 VIO/ExternalNav。

## 7. 感知质量与滤波门控

飞行配置应比台架更严格，初始建议：

- 只接受 tag36h11/ID 0；
- `hamming == 0`；
- `decision_margin ≥ 35`，根据现场光照再调整；
- 标签四边完整；
- 重投影误差建议 `≤ 1.5 px`；
- 单帧位置跳变 `≤ 0.25 m`；
- 连续 5 帧有效才从 ACQUIRE 进入 FOLLOW；
- 标签面积、倾角和距离必须位于标定有效范围；
- 3 帧中值滤波后进入 alpha-beta/Kalman 滤波器。

IMX296 是全局快门，适合移动目标；仍需限制曝光时间以减少运动模糊。曝光、增益和灯光应在真实小车速度下做离线数据采集后确定。

建议将 AprilTag 朝上刚性固定、保持平整，并增加一个更大的外层标签或多尺寸标签板，以提高高度变化和画面边缘处的检测率。

## 8. 安全状态机

```text
DISABLED
  └─ RC允许 + GUIDED + 健康检查通过 → OBSERVE
OBSERVE
  └─ 连续5帧有效 → ACQUIRE
ACQUIRE
  └─ 位置稳定0.5s → FOLLOW_XY
FOLLOW_XY
  ├─ 标签短暂丢失 → PREDICT_DECEL
  ├─ 标签持续丢失 → HOLD
  ├─ RC切换模式 → PILOT_OVERRIDE
  └─ EKF/电池/高度异常 → ABORT
HOLD
  └─ 人工重新启用后才能回到ACQUIRE
```

目标丢失策略：

- `0–0.25 s`：允许常速度短时预测；
- `0.25–0.7 s`：命令速度按斜坡减到零；
- `>0.7 s`：持续发送零速度，进入 HOLD，不自动搜索、不自动下降；
- 程序崩溃或串口停止：依靠 `GUID_TIMEOUT` 使飞控减速停止；
- RC 模式开关离开 GUIDED：控制器立即停止发送，人工控制优先。

任何时刻满足以下条件都禁止输出运动命令：

- 不在 GUIDED；
- 未经 RC 辅助开关明确启用；
- EKF 水平位置无效；
- 高度不在许可范围；
- 标签观测过期或质量不足；
- 速度/位置创新超限；
- 电池状态未知或低电量；
- 串口出现多个控制进程。

## 9. 新增软件模块

建议新增而不破坏现有精准降落代码：

```text
imx296_debug/
  target_observation.py       # 带曝光时间戳和姿态的标签观测
  target_tracker.py           # 异常值拒绝、alpha-beta/Kalman滤波
  follow_controller.py        # P控制、前馈、限速和加速度限制
  mavlink_guided_client.py    # 只负责状态读取和速度setpoint
  follow_state_machine.py     # RC许可、模式、目标丢失和ABORT
  apriltag_follow_runtime.py  # 相机/控制主循环
  follow_replay.py            # 日志回放，不连接真实飞控

config/
  apriltag_follow_sitl.yaml
  apriltag_follow_bench.yaml
  apriltag_follow_flight.yaml # 通过每级验收后才生成

tests/
  test_target_tracker.py
  test_follow_controller.py
  test_follow_state_machine.py
  test_guided_message.py
```

生产运行器必须使用单实例锁，禁止相机预览、精准降落桥和跟随控制器同时占用相机或 TELEM1。

## 10. Jetson/小车协同

Jetson 可以从小车底盘里程计/CAN 得到车速，通过 UDP 发送：

```text
timestamp, sequence, vx, vy, yaw_rate, health
```

Pi 只在数据新鲜、序号连续且时间同步正常时将其作为 `v_target` 前馈。超过 200 ms 未更新立即舍弃，退回纯视觉 P 控制。Jetson 不直接向 Pixhawk发送飞行命令。

小车在首轮测试中应限制为：

- 直线速度不超过 `0.10 m/s`；
- 加速度不超过 `0.10 m/s²`；
- 不急转弯；
- 具备独立急停；
- 开阔场地内运行。

## 11. 分阶段实施与验收

### 阶段 0：飞行基础条件

全部完成前禁止装桨自主测试：

- 电池监测已于 2026-08-07 启用并完成 6S/6000mAh 未解锁台架配置；正式飞行前仍需用充电器或万用表校准电压倍率，并用负载数据校准电流倍率；
- 验证电机编号、旋向、桨叶方向和 ESC；
- 完成 RC 模式、失控保护、地理围栏和人工接管检查；
- 确认 GPS/EKF，或安装 Optical Flow + Rangefinder/VIO；
- 物理测量相机外参并改为 `flight_use_approved=true`；
- 将距离标定扩展到计划高度，例如 0.5/0.8/1.0/1.2/1.5 m；
- 静态悬停和飞控基础调参必须先独立通过。

### 阶段 1：纯离线

- 录制手推小车的 IMX296 视频和传感器时间戳；
- 回放检测、滤波和控制器；
- 验证坐标正负号、延迟、速度/加速度限制；
- 人工注入丢帧、错误 ID、位置跳变和串口中断。

验收：零真实 MAVLink 控制命令；所有异常在 0.7 s 内输出归零。

### 阶段 2：SITL

- 使用 ArduCopter SITL；
- 模拟标签静止、直线、加减速、转弯和丢失；
- 发送真实 `SET_POSITION_TARGET_LOCAL_NED`；
- 检查 GUID_TIMEOUT、模式退出和 RC 接管。

验收建议：

- 静态稳态水平误差 `<0.10 m`；
- 0.2 m/s 目标的跟随误差 `<0.30 m`；
- 目标丢失 0.7 s 后速度命令为零；
- 无速度尖峰，始终满足限速/限加速度。

状态：**已于 2026-08-07 完成首轮闭环验收**。本地 ArduCopter SITL 通过 MAVProxy 仅转发到 `127.0.0.1:14550`，仿真正常预检、解锁、起飞、跟随、BRAKE 接管、恢复、LAND 和自动上锁。32 秒/10 Hz 共 321 个周期，速度峰值 `0.200000 m/s`，FOLLOW_XY 状态最大水平误差 `0.290473 m`；1.2 秒丢标触发减速与锁存 HOLD，RC 关闭/重新启用后恢复。全过程未连接真实 Pixhawk。

### 阶段 3：拆桨台架命令预览

- 真实相机和 Pixhawk运行；
- 控制器只记录“本应发送”的速度，不发送运动 setpoint；
- 手动移动标签检查方向和数值。

验收：标签向机头方向移动时前向命令为正；向机体右侧移动时右向命令为正；标签停止后命令平滑归零。

### 阶段 4：人工悬停与静态标签

- 开阔场地、低速、保护网/安全员；
- 人工起飞到约 0.8 m；
- 先只启用 FOLLOW_XY；
- 静态标签，速度上限 0.15–0.20 m/s；
- RC 随时切回 Loiter/AltHold。

### 阶段 5：慢速移动目标

- 先由人员以 0.05 m/s 移动标签；
- 再使用小车 0.10 m/s 直线；
- 逐步提升到 0.20 m/s；
- 每一级都先测试目标突然停止和标签遮挡。

### 阶段 6：高度与精准降落

只有 FOLLOW_XY 完成后才分别验证：

1. Rangefinder 约束下的 FOLLOW_XYZ；
2. Precision Loiter；
3. 设置移动目标选项后的 Precision Land；
4. 最终移动平台接地与充电对准。

## 12. 可参考但不能直接照搬的开源代码

- AprilRobotics/AprilTag：检测、坐标约定和姿态估计的上游基础；当前 `pupil_apriltags` 思路可保留。
- `vision_to_mavros/t265_precland_apriltags.py`：可参考相机到机体坐标变换和 `LANDING_TARGET` 发送，但它使用 T265、DroneKit 和旧 ROS，不能作为本机飞行控制器。
- 项目内 `04_moving_platform_landing`：可参考 WAITING/CHECKING/PREPARE/SEARCH/TRACKING 状态机和速度限幅，但它面向 PX4/ROS1/Gazebo，PID 缺少时间尺度和完善失效保护。
- ArduPilot 官方 Precision Landing：用于最终移动平台降落；持续跟随阶段使用官方 GUIDED 速度接口。

## 13. 当前结论

当前最合理的下一项开发工作不是装桨试飞，而是实现并验证：

1. `target_tracker.py`；
2. `follow_controller.py`；
3. `follow_state_machine.py`；
4. 动态目标离线回放；
5. ArduCopter SITL GUIDED 速度闭环。

以上五项软件级工作和阶段 3“拆桨台架命令预览”均已完成。真实相机与真实 Pixhawk 只读取姿态/位置并记录拟发送速度，全程未发送运动 setpoint。电池监测已完成 6S/6000mAh 未解锁台架配置，但电压/电流倍率仍需外部仪表和负载校准；进入阶段 4 前还需取得有效水平位置/速度源、完成实测外参风险复核，并在满足飞行条件的场地重新执行飞行前检查。

### 2026-08-07 第一批实现

已新增：

- `imx296_debug/target_tracker.py`：alpha-beta 常速度滤波、时间戳检查、0.25 m 残差门和连续 5 帧捕获；
- `imx296_debug/follow_controller.py`：NED 水平 P 控制、目标速度前馈、死区、速度和加速度限幅；
- `imx296_debug/follow_state_machine.py`：RC 许可、GUIDED、EKF/电池/高度门、短时预测、减速和锁存 HOLD；
- `imx296_debug/mavlink_guided_velocity.py`：只编码 `SET_POSITION_TARGET_LOCAL_NED`，不打开链路；
- `imx296_debug/follow_replay.py`：纯离线动态目标与遮挡回放；
- `config/apriltag_follow_sitl.yaml`：仅离线/SITL 配置，`flight_use_approved=false`。

`test_apriltag_follow.py` 的 10 项测试全部通过。10 秒、10 Hz 合成回放生成 101 条记录，命令速度峰值 0.153184 m/s；1 秒目标遮挡正确进入 `PREDICT_DECEL` 和锁存 `HOLD`，RC 关闭再启用后才恢复。回放未打开串口、未发送真实 MAVLink 控制命令。证据为 `output/apriltag_follow_synthetic_20260807.jsonl` 和对应 summary JSON。

### 2026-08-07 SITL 闭环验收

- 启动器：`run_sitl_apriltag_follow_test.sh`；测试程序：`uav_sitl_apriltag_follow_test.py`。
- SITL 使用 ArduCopter V4.6.3，保留全部正常预检；没有使用强制解锁或关闭预检。
- 链路固定为本机 `udpin:127.0.0.1:14550`，测试程序拒绝其他端点；结果明确记录 `physical_vehicle_connected=false`。
- 轨迹包含静止捕获、直线、转向、1.2 秒目标遮挡、RC 许可循环和 GUIDED→BRAKE→GUIDED 模式接管。
- 共 321 条记录；状态计数：ACQUIRE 4、FOLLOW_XY 265、PREDICT_DECEL 5、HOLD 23、DISABLED 4、PILOT_OVERRIDE 20。
- 速度峰值 `0.200000 m/s`，FOLLOW_XY 最大水平误差 `0.290473 m`，满足当前 `0.20 m/s` 和 `<0.30 m` 门限。
- 最后 LAND 并自动上锁，综合结果 `passed=true`。
- 证据：`output/apriltag_follow_sitl_closed_loop_20260807.jsonl` 与 `output/apriltag_follow_sitl_closed_loop_20260807_summary.json`。

### 2026-08-07 MTF-01P 真实只读审计

MTF-01P 已确认接在 TELEM2 并输出光流与测距。基础参数为 `SERIAL2_PROTOCOL=1`、`SERIAL2_BAUD=115`、`FLOW_TYPE=5`、`RNGFND1_TYPE=10`、量程 0.01–8 m、方向向下。20 秒静态数据中测距稳定在 0.18–0.20 m，光流质量均值约 100/255，静止速度均值接近零但存在约 ±0.20 m/s 瞬时尖峰。

当前尚不具备位置控制条件：GPS 无定位，EKF 主水平位置与速度源仍是 GPS，EKF 标志 231 不包含水平位置有效位；电池监测仍为关闭。另发现 MTF 原始消息源为 `1/88`，与飞控 `1/1` 使用相同系统 ID，且 `SERIAL2_OPTIONS=0` 会将约 80 Hz 原始消息转发到 TELEM1。按 ArduPilot 4.5+ 官方建议，后续应先通过 MicoAssistant 将 MTF `mav_id` 改为非 1，并单独授权设置 `SERIAL2_OPTIONS=1024`，随后再设计 GPS/OpticalFlow EKF 双源切换。以上改动本次均未执行。

安全开关解除后的复测确认硬件安全预检提示已消失，35 个真实心跳全部未解锁；但 GPS、电池和 EKF 状态不变，光流仍有孤立尖峰。安全开关只改变电机输出许可，不代表整机已满足位置控制或跟随飞行条件。

### 2026-08-07 阶段 3 静态命令预览

真实 IMX296、真实 AprilTag 和真实 Pixhawk 接收遥测已接入 `follow_command_preview.py`。程序只接收串口，在内存中计算/编码候选速度，不发送 MAVLink。20.69 秒得到 351/351 有效检测，标签距离均值 0.735861 m；标签相对机体前方 0.042019 m、左侧 0.004734 m，落在 0.05 m 死区，拟发送速度峰值 0.000071 m/s。26 个飞控心跳均未解锁，真实发送计数为 0。

静态居中归零已通过。随后已完成机头方向、机体右侧和多次遮挡的动态台架验收；结果见下一节。该预览器仍保持只接收、不发送，不能直接改为真实飞行发送器。

### 2026-08-07 阶段 3 动态命令预览

两次动态运行合计 100.97 秒、1747 帧，其中 1406 帧有效、0 帧被质量门拒绝。标签向机头方向移动时，BODY_FRD X 最大为 +0.113914 m，拟前向速度最大为 +0.047155 m/s；标签向机体右侧移动时，BODY_FRD Y 最大为 +0.099909 m，拟右向速度最大为 +0.079681 m/s。两项方向符号均符合预期，所有拟速度均低于 0.20 m/s 上限。

遮挡产生 3 个丢标区段并进入 `PREVIEW_DECEL`/`PREVIEW_HOLD`，最长目标年龄 7.758 秒，末端速度为 0。日志审计发现高速遮挡在刚越过 0.70 秒边界时，旧实现受限加速度器影响，3 帧仍有最高 0.030277 m/s 残余。现已修正为 `ACQUIRE` 和 `PREVIEW_HOLD` 直接绕过限加速度器并强制严格零速度，同时重置控制器；树莓派 15 项测试全部通过。真实动态日志形成于修正前，因此保留该事实，不把它表述为修正后的实时复测。

两次动态运行共收到 112 个真实未解锁心跳、0 个已解锁心跳；离线编码候选包 1747 个，真实发送 0。参数、模式、解锁、电机、起飞和降落命令计数均为 0。阶段 3 至此完成；电池监测随后已完成 6S/6000mAh 未解锁台架配置。阶段 4 人工悬停仍被外部电压/电流校准、有效水平位置/速度源和飞行外参风险复核阻塞。

### 2026-08-07 6S/6000mAh 电池配置与复测

- 用户通过电池标签确认动力电池为 6S LiPo、标称 22.2 V、6000 mAh；满电理论值为 25.2 V。
- Pi 经 TELEM1 在真实飞控 `system 1/component 1`、`ARMED=0` 安全门下写入并逐项回读：`BATT_CAPACITY=6000`、`BATT_LOW_VOLT=21.6`、`BATT_CRT_VOLT=21.0`、`BATT_FS_LOW_ACT=2 (RTL)`、`BATT_FS_CRT_ACT=1 (Land)`、`BATT_ARM_VOLT=22.2`。
- 容量型保护 `BATT_LOW_MAH/BATT_CRT_MAH` 暂时保持 0，避免未校准电流传感器造成错误触发；`BATT_VOLT_MULT=10.1`、`BATT_AMP_PERVLT=17` 也保持不变，等待外部仪表和负载校准。
- 写入后 20 秒只读验收：20/20 心跳未解锁，电池 22.372 V、0.14 A、fault bitmask 0，无 STATUSTEXT 告警；向下测距 0.77 m，光流地距 0.785 m、质量 110。
- GPS 仍为 `fix_type=1`、0 星，EKF flags=231，不具备水平位置飞行条件。系统 `errors_count3=982` 在 12 秒 24 个样本中完全不增长，判定为历史累计值而非当前活动故障。
- 完整机器可读证据：`config/pixhawk_battery_6s_6000_20260807.json`。当前结论是电池配置通过未解锁台架验收，但整机仍未批准飞行。

### 2026-08-07 CH7 树莓派跟随许可开关

- 遥控器为 RadioLink AT9S Pro，接收机为 R9DS SBUS。飞行模式保持 `FLTMODE_CH=5`，发射机姿态选择使用 `CH5 + SwC`；`SwA/SwB` 分别已映射到 CH10/CH9。
- 用户将原 `CH7=VrC` 改为独立两段拨杆 `CH7=SwD`，Mission Planner 实测：SwD 上拨为 1000、下拨为 2000。
- 安全语义冻结为：上拨/1000=`follow_disabled`，下拨/2000=`follow_permitted`。`RC7_OPTION=0` 保持不变，避免飞控对同一通道执行辅助功能。
- 新增 `imx296_debug/rc_follow_gate.py`：CH7 >=1800 才许可，<=1200 关闭；1200–1800、无样本或样本超过 0.5 秒均失效关闭。目标丢失后的锁存 HOLD 仍要求先上拨关闭、再下拨重新许可。
- `follow_command_preview.py` 已接入该门控；关闭时强制 `RC_DISABLED`、速度严格为零并重置控制器，工具仍保持只收不发。

真实 IMX296、真实飞控和真实 CH7 的集成拨杆循环也已完成。上拨 1000 时，居中场景 279/279 帧以及保持同一偏移场景 275/275 帧均为 `RC_DISABLED`，前向和横向拟速度严格为零。下拨 2000 时，居中场景 276/280 帧进入 `PREVIEW_FOLLOW` 且速度峰值仅 0.000095 m/s；偏移场景 273/277 帧进入 `PREVIEW_FOLLOW`，产生正确方向的前向最高 +0.014401 m/s、向左最低 -0.024990 m/s，合速度峰值 0.028815 m/s。四组共 84 个飞控心跳全部未解锁，真实 MAVLink 控制发送为 0。最终物理状态恢复为 SwD 上拨/1000/跟随关闭。
- 状态机、预览零速和RC门共20项Pi端测试通过，Windows/Pi SHA-256一致。真实只读拨杆验收为：上拨60/60样本均1000且门关闭；下拨60/60样本均2000且门许可；最终上拨40/40样本均1000且门关闭。三段采样共32个飞控心跳全部未解锁，无状态告警，真实控制发送为0。
- 最终物理状态为SwD上拨/1000/跟随关闭，无跟随或降落桥进程常驻。证据为 `config/rc_follow_authorization_20260807.json`。这只批准真实RC输入门的只读台架使用，尚未批准真实速度发送或飞行。

### 2026-08-07 无 GPS 光流 EKF 配置

- 五次连续真实飞控未解锁心跳通过后，已按 ArduPilot 光流正常运行参数写入并回读：`EK3_SRC1_POSXY=0`、`EK3_SRC1_VELXY=5`、`EK3_SRC1_POSZ=1`、`EK3_SRC1_VELZ=0`、`EK3_SRC1_YAW=1`、`EK3_SRC_OPTIONS=0`。
- 最初按 MTF-01 文档设置 `SERIAL2_OPTIONS=1024` 后，ArduCopter 4.7 预解锁检查明确拒绝该旧接口并要求使用 `MAVn_OPTIONS bit 1`。已立即回滚 `SERIAL2_OPTIONS=0`，保留现有 `MAV3_OPTIONS=2`；安全重启后警告消失。
- 重启后飞控报告两个 EKF3 lane 均 `fusing optical flow`、`started relative aiding`；EKF flags 从 231 变为 367，已有水平速度、相对水平位置和离地高度有效标志。
- 未解锁模式接受测试确认 `STABILIZE -> LOITER -> STABILIZE`，未发送解锁、起飞、降落、电机或速度命令。
- EKF 原点仍为 0，全部历史 Mission Planner tlog 也没有有效 GPS fix 可复用；正式 GUIDED 跟随前仍需通过室外 GPS fix 或 GCS 设置有效原点。
- 10 秒静态复核中 MTF 质量均值 103.17、测距均值 0.1637 m，但原始光流存在约 -0.47 至 +0.40 m/s 的孤立尖峰。正式 Loiter/Guided 飞行前仍需按官方流程完成低空光流方向和尺度飞行校准。
- 完整机器可读结果与回滚备份：`config/optical_flow_ekf_config_20260807.json`、`config/optical_flow_ekf_before_20260807.json`。

### 2026-08-07 树莓派常开跟随条件监控

- 新增并部署 `follow_readiness_monitor.py`。该服务持续占用 IMX296、检测 `tag36h11 ID0`、只读 TELEM1 遥测，并在 `http://192.168.1.11:8765/` 提供预览，在 `/status.json` 提供机器可读状态。
- 服务已设置为 Pi 用户级 systemd 自启：`apriltag-follow-monitor.service`。它是严格的接收/监控程序；源码中没有 MAVLink 发送、模式切换、解锁、起飞、降落或速度设定值调用，配置也固定为 `control_enabled=false`。
- 跟随申请条件冻结为：飞控心跳与 RC 新鲜、进入模式为 LOITER/GUIDED、CH7 遥测有效、EKF 相对水平位置与速度有效、有效 EKF 全局原点、电池不低于 21.6 V/20%、MTF 高度 0.55–0.85 m、光流质量不低于 80、AprilTag 连续捕获且年龄不超过 0.25 s、相机在线。GPS 本身不是室内跟随的必要条件。
- 2026-08-07 18:36 CST 部署后真实状态：服务 active，EKF flags=367，电池约 22.90 V，MTF 质量 95–104；控制被 `STABILIZE`、原点缺失、地面高度约 0.16 m、标签未捕获阻塞。CH7=1000，飞控未解锁，所有控制发送标志均为 false。
- `RNGFND1_MAX=8 m` 是 MTF 测距最大值。光流是唯一水平源时，ArduPilot 会限制 Loiter/PosHold 不爬升超过该测距上限；这不是室内/室外分界，也不要求室外手改 EKF 参数。GPS/光流切换应使用预先配置的 EKF Source Set 和经过台架/飞行验证的切换流程。
- 真实跟随控制仍未批准：缺少有效 EKF 原点、光流方向/尺度首飞校准尚未完成，且真实速度发送运行器尚未部署。首次飞行必须先独立验证低空手动悬停和 Loiter 光流定点，不能直接进行 AprilTag 移动跟随。

### 2026-08-07 室内 EKF 持久原点

- 用户提供场地坐标：北纬 22°8′6″、东经 113°32′41″，换算为 `22.1350000, 113.5447222`；参考海拔设置为 0 m。
- 在连续未解锁门控下写入 `AHRS_ORIGIN_LAT/LON/ALT`，并把 `AHRS_OPTIONS` 设为 24（bit3 记录原点、bit4 无 GPS 时恢复记录原点）。
- 本固件不接受 EKF 已初始化后的在线原点替换；第一次在线尝试未收到确认且只临时应用了纬度，因此未判成功。随后执行未解锁安全重启，从持久参数完整恢复原点。
- 重启后最终只读验证：`GLOBAL_POSITION_INT lat=22.1350000, lon=113.5447296`，与请求位置相差约 0.8 m（`AHRS_ORIGIN_LON` 为 REAL32 的量化结果）；`origin_valid=true`、EKF flags=367、飞控未解锁。GPS 仍为 fix_type=1/0 星，当前导航源仍是 Source1 光流，手工原点不等于 GPS 已定位。

## 14. 主要资料

- ArduPilot Precision Landing and Loiter：https://ardupilot.org/copter/docs/precision-landing-and-loiter.html
- ArduPilot Copter Commands in Guided Mode：https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- ArduPilot Guided Mode 与 GUID_TIMEOUT：https://ardupilot.org/copter/docs/ac2_guidedmode.html
- AprilRobotics/AprilTag：https://github.com/AprilRobotics/apriltag
- ArduPilot ROS/VIO 对 `vision_to_mavros` 的说明：https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html
