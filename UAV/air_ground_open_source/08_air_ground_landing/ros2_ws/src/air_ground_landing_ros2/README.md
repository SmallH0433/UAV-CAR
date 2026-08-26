# ROS 2 Humble Elastic / IBVS / GUIDED 适配层

本包把旧 ROS 1 研究仓中的可用控制边界迁移到 ROS 2，而不是让旧节点直接连接飞控：

```text
Elastic ROS 2 trajectory_msgs/MultiDOFJointTrajectory
                  -> /landing/elastic/candidate --+
OV9281 dual tag -> IBVS ROS 2 candidate ---------+--> guided_executor
OV9281 dual tag -> LANDING_TARGET adapter -> MAVROS -> ArduPilot AC_PrecLand
IBVS/target freshness + descent request -> simple_landing_coordinator -> owner
Hybrid coordinator -> /landing/control_owner ----+        |
MAVROS state + RC input -------------------------+        +-> /mavros/setpoint_raw/local
                                                           +-> /mavros/set_mode
CH6 follow + CH8/SwD descent --------------------+        +-> /landing/descent_request
```

`guided_executor` 是模式与 GUIDED setpoint 的唯一写入者；`landing_target_adapter` 只向 MAVROS 的 landing-target 插件发布传感器观测。`simple_landing_coordinator` 补齐最简实机路径：IBVS 新鲜时选择 `IBVS_GUIDED`，SwD 请求且 LANDING_TARGET 新鲜时选择 `AC_PRECLAND_LAND`。Elastic 负责可选的远距会合；IBVS 只输出 BODY 水平速度；进入 `LAND` 后由 AC_PrecLand 接管下降。本包没有移植 `ibvs_sim` 的 PX4 电机渐停、解锁/上锁和垂向控制。

## 模式 ACK 与回滚

模式切换不是一个网页或虚拟按钮：

1. 协调器发布 `ELASTIC_GUIDED` 或 `IBVS_GUIDED`，只表示软件控制权请求；
2. 执行器调用 MAVROS `/mavros/set_mode`；服务返回的 `mode_sent=true` 仅表示 MAVROS 已发送 `SET_MODE`；
3. 后续 `/mavros/state.mode == GUIDED` 才是飞控 HEARTBEAT 确认；
4. 超时、RC 撤权、候选指令超过 0.2 s、控制权超过 0.5 s 未刷新，都会停止 setpoint 并请求回到进入前的 LOITER/BRAKE/POSHOLD/ALT_HOLD，其他前态统一回 LOITER；
5. 若回滚也得不到 HEARTBEAT 确认，状态进入 `FAULT`，不再反复抢模式。

遥控器职责为：`CH5` 是飞行模式通道；`CH6` 是跟飞总开关；`CH7` 选择光流/GPS EKF 源；`CH8/SwD` 是下降开关。SwD 只有在 GUIDED HEARTBEAT 已确认跟飞后才生效，而且每次跟飞都要求先低位、再低到高；跟飞建立前已经处于高位不会触发下降。SwD 高位请求 `GUIDED -> LAND`，低位请求 `LAND -> GUIDED` 并恢复定高跟飞；CH6 关闭才是撤销整套自动控制并回滚到进入前模式。`RC6_OPTION`、`RC8_OPTION` 保持 0，`RC7_OPTION=90` 只选择 EKF 源组。

实机已按 `/mavros/rc/in` 验证 CH6、CH7、CH8 原始输入。物理拨杆映射仍应在拆桨状态核对低/中/高 PWM，确认通道未被云台或继电器占用。

## 构建

```bash
cd 08_air_ground_landing/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select air_ground_landing_ros2
source install/setup.bash
ros2 launch air_ground_landing_ros2 adapters.launch.py
```

默认 `environment=offline`，模式服务和 MAVROS setpoint 都关闭；`/landing/guided_executor/preview` 仍可用于核对候选指令。只在 SITL 中把 `environment`、`allow_mode_change` 和 `allow_setpoint_output` 改为相应测试值。实机还要求 `flight_use_approved=true` 且 RC 门有效，本原型没有替用户打开这些开关。

## ROS 2 接口约定

- Elastic 输入：`/elastic_tracker/trajectory`，类型 `trajectory_msgs/msg/MultiDOFJointTrajectory`，位置/速度/加速度为 ROS ENU；适配器按 `time_from_start` 以 20 Hz 插值。
- IBVS 输入：OV9281 `/api/status`，通过已经质量门控的角点控制器生成 ROS FLU 水平速度；MAVROS 转为 BODY_FRD。
- 控制权：`/landing/control_owner`，`std_msgs/msg/String`，值只能由混合协调器发布 `ELASTIC_GUIDED`、`IBVS_GUIDED`、`AC_PRECLAND_LAND`、`HOLD` 或 `NONE`。
- 下降请求：`/landing/descent_request`，`std_msgs/msg/Bool`；由执行器对 CH6、GUIDED HEARTBEAT 和 CH8/SwD 低到高边沿门控后发布，监督器消费。
- 输出：候选始终先发布到 preview；只有 RC、时效、安全配置和 GUIDED HEARTBEAT 同时满足才写 `/mavros/setpoint_raw/local`。

## 跟飞提示音

硬件配置通过 MAVLink `PLAY_TUNE` 向飞控蜂鸣器发送提示：

- 单音 `C`：MAVROS 已连接、飞行模式允许进入跟飞、AprilTag/IBVS 候选和控制权均新鲜；此时可以拨高 CH6；
- 上升音 `C-E-G`：飞控 HEARTBEAT 已确认 GUIDED、速度 setpoint 已放行，并持续收到新鲜的 ID 85 `/mavros/setpoint_raw/target_local` 回显；立即播放，跟飞期间每 3 秒重复；
- 下降音 `G-E-C`：已确认的跟飞进入 LAND 后立即播放，降落期间每 2 秒重复；退出跟飞或落地解除解锁时只播放一次。

仅有 `/mavros/set_mode` 服务 ACK 不会播放跟飞成功音。ID 85 回显只作为存在性与时效门控，不再比较回显速度和发送速度；最近回显时间、连续性、回显速度及其与最新发送值的差异仍写入状态用于诊断。目标回显订阅使用 SensorData/Best-Effort QoS，与 MAVROS `setpoint_raw` 插件一致。每个提示音事件同时写入 journal，格式为 `FOLLOW_TONE_EVENT {...}`，并累计在 `/landing/guided_executor/status.tone_event_counts` 中。

旧 Elastic-Tracker 的完整规划器仍包含 ROS 1 nodelet/PCL/catkin 依赖。本包迁移的是其执行契约；要原生编译完整规划器，还需逐包把 nodelet 改为 ROS 2 component，并替换所有自定义消息。当前主线不需要运行 ROS1 bridge。
