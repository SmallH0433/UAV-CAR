# 唯一动作执行节点

`action_executor` 是任务层与 MAVROS 之间的唯一动作出口。任务层只发布动作、等待状态并决定下一步；任务层不发布 setpoint，也不调用飞控模式或上锁服务。

```text
任务层 JSON 请求
       ↓
action_executor
  ├─ 单动作互斥与 action_id
  ├─ RC6 会话和人工接管锁存
  ├─ 前置条件、完成判据、超时和取消
  ├─ GUIDED/LAND 服务请求 + State.mode 心跳确认
  └─ 唯一 MAVROS setpoint/模式/普通上锁/急停出口
       ↓
Pixhawk / ArduCopter
```

## 启动

新启动文件只启动 IBVS 观测适配器、LandingTarget 观测适配器和唯一动作执行器，不启动 `guided_executor` 和 `simple_landing_coordinator`。默认配置为 `offline`，全部真实输出关闭：

```bash
cd 08_air_ground_landing/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select air_ground_landing_ros2
source install/setup.bash
ros2 launch air_ground_landing_ros2 action_executor.launch.py
```

SITL 联调时显式选择覆盖配置：

```bash
ros2 launch air_ground_landing_ros2 action_executor.launch.py \
  parameters_file:=$(ros2 pkg prefix air_ground_landing_ros2)/share/air_ground_landing_ros2/config/action_executor.sitl.yaml
```

即使错误地启动两个 `action_executor` 进程，第二个进程也无法取得 `/tmp/uav_action_executor.lock`。这个锁不能阻止旧版 `guided_executor` 被单独启动，因此部署服务必须只使用 `action_executor.launch.py`。

## 动作协议

请求话题为 `/landing/action/request`，消息类型为 `std_msgs/msg/String`，内容是 JSON。每个动作都必须具有唯一 `action_id` 和有限 `timeout_s`。同一进程生命周期内已经使用过的 `action_id` 不会再次执行；重放请求返回 `REJECTED_ACTION_ID_REPLAY`。

移动 1 米：

```json
{
  "action_id": "mission-42/move-1",
  "action": "MOVE",
  "timeout_s": 8.0,
  "params": {
    "frame": "LOCAL_ENU",
    "direction": [1.0, 0.0, 0.0],
    "distance_m": 1.0,
    "speed_mps": 0.3
  }
}
```

其他动作示例：

```json
{"action_id":"m42/hover","action":"HOVER","timeout_s":6,"params":{"duration_s":5}}
{"action_id":"m42/yaw","action":"ROTATE","timeout_s":5,"params":{"angle_deg":30}}
{"action_id":"m42/follow","action":"FOLLOW","timeout_s":60,"params":{"maximum_speed_mps":0.3}}
{"action_id":"m42/guided-descent","action":"PRECISION_LAND","timeout_s":30,"params":{}}
{"action_id":"m42/native-land","action":"LAND","timeout_s":60,"params":{"precision":false,"guided_descent":false}}
{"action_id":"m42/disarm","action":"DISARM","timeout_s":5,"params":{}}
{"action_id":"m42/estop","action":"EMERGENCY_STOP","timeout_s":3,"params":{}}
```

取消话题为 `/landing/action/cancel`。消息可以直接是 `action_id`，也可以是：

```json
{"action_id":"mission-42/move-1","reason":"TASK_ABORTED"}
```

普通动作不能抢占正在运行的动作，返回 `REJECTED_BUSY`。`EMERGENCY_STOP` 是唯一可抢占动作，并且不能通过任务取消解除。AUX 31 的解除必须走单独、经审查的人工流程。

## 状态协议

状态发布在 `/landing/action/status`，对外状态只有：

```text
IDLE → RUNNING → DONE | FAILED | TIMEOUT | CANCELLED
```

`detail` 给出 `WAITING_GUIDED_HEARTBEAT`、`WAITING_LAND_HEARTBEAT`、`FOLLOW_TARGET_GRACE_HOLD`、`TERMINAL_DESCENT` 等动作内部阶段。它们没有独立控制权。

`DONE`、普通 `FAILED` 或 `TIMEOUT` 后可能以零速度继续保留 GUIDED 控制，状态中的 `control_retained=true` 会明确表示这一点。任务层必须立即提交后继动作或发布取消；飞手切模、RC6 撤权、遥测失效会立即释放该保持输出。

## 当前动作边界

- `MOVE` 使用 ROS `LOCAL_ENU`，斜向动作通过方向向量表达，不建立额外动作类型。
- `ROTATE` 支持相对 `angle_deg`、绝对 `heading_deg`，或持续 `yaw_rate_deg_s`。
- `FOLLOW` 使用 IBVS 的机体 FLU 水平速度候选，在执行节点内转换为本地 ENU。
- `PRECISION_LAND` 先以水平速度和偏航角速度将飞机居中、机头朝向对齐 Tag；连续对准后才输出 GUIDED 下降速度。终端低速下降阶段仍持续检查对准并修正；对任务层仍只有一个 `RUNNING` 状态。
- `LAND` 只允许从已确认的 GUIDED 会话开始。`precision` 默认为 `true`，`guided_descent` 默认跟随 `precision`，默认采用先居中和对齐机头、再下降的 GUIDED 路径，到 0.10 m 交接原生 LAND。开始前必须确认 LandingTarget 流新鲜且正在输出；`rc_managed=true` 时由 CH8 电平管理进入和退出。显式 `guided_descent=false` 仍可选择原生 LAND 兼容路径，该路径不提供动作层偏航闭环。
- `DISARM` 仅在飞控 `ON_GROUND` 证据新鲜时允许开始，并以之后的 `armed=false` 心跳作为完成证据。
- `EMERGENCY_STOP` 使用 `MAV_CMD_DO_AUX_FUNCTION` 的 AUX 31。当前系统没有可信的电机转速反馈，所以 ACK 不会被报告为“电机已经停转”；动作会保持 `RUNNING` 并最终 `TIMEOUT`，同时飞控侧急停保持锁存。

飞行动作启动前统一检查：新鲜飞控状态、RC6 会话授权、已解锁、EKF 水平状态健康、Home 已建立以及本地位姿和速度新鲜。GUIDED 和 LAND 的服务 ACK 只表示请求已发送；执行器分别等待 `/mavros/state.mode == GUIDED` 或 `LAND` 才确认转换。CH8 新流程先发送 GUIDED 跟踪下降速度，到 0.10 米才进入原生 LAND；原生 LAND 阶段不发送速度 setpoint。

## 飞行模式流程（CH8 GUIDED 下降）

CH5 由飞手切到 LOITER；CH6 低位驻留后拨高建立 GUIDED 跟飞会话。
CH8 高位发起一个 `LAND` 动作，带 `guided_descent: true`，其内部流程为：

- 高于 0.10 m：先保持 GUIDED 定高，将 AprilTag 修正到画面中心，机头朝向对齐校准后的 Tag 方向（同向，反向 180° 不算对齐）。同帧新鲜观测的中心误差不超过 20 px、朝向误差不超过 4°，连续满足 0.4 s 后，才以默认 0.10 m/s 下降。下降期间任一误差超限，垂向指令立即归零并重新对准、计时。
- 降落居中使用画面几何中心：1280×800 图像的目标为 (640, 400)，不使用相机标定主点。普通 FOLLOW 候选不变。大小 Tag 共用已完成布局补偿的 pad 姿态，避免小 Tag 切换时额外旋转 45°。
- 偏航误差来自桥接器校验过的 BODY_FRD 姿态，转换为 ROS 左转为正的角速度；默认比例增益 0.8/s、最大 15°/s、4°死区。水平居中、转向、下降在同一条本地 ENU setpoint 中发送。
- GUIDED 修正阶段，Tag 仍可见但校准姿态缺失/无效、观测总年龄超过 0.3 s：立即清零平移和偏航角速度，进入同样的 1 秒 GUIDED 恢复悬停；不把姿态失效当成低空丢标上锁依据。截止时间前正常观测恢复则重新对准并完成驻留后下降；超时且 Tag 仍可见则退出到纯跟飞。
- 丢标且高于阈值：立即输出三轴零速度，进入恢复锁存；等待 1.0 秒，期间 CH8 退出暂缓，CH6 撤权/飞手接管仍优先。
- 截止时间前重捕获：恢复 GUIDED 对准，重新满足 0.4 s 驻留后下降；若 CH8 已拨低，则解除恢复锁存并退出到纯跟飞。
- 到达恢复截止时间：先退出降落，不再因该帧重捕获继续下降；有 Tag 回 GUIDED 纯跟飞，无 Tag 到 LOITER。
- 退出后、等待飞控确认 LOITER 的期间，只要飞控仍处于 GUIDED 且授权有效，就持续发送三轴零速度及零偏航角速度，避免保留上一个下降/转向指令。
- 未锁存时 CH8 退出：有 Tag 回 GUIDED 纯跟飞（垂直速度为零），无 Tag 到 LOITER。
- 测距等于 0.10 m：交接飞控 LAND，锁存终端阶段；CH8 普通退出不再恢复跟飞。
- 测距严格小于 0.10 m 且丢标：立即请求普通 DISARM，同时保持/请求 LAND。该分支持续到 `armed=false`；不使用强制上锁或 AUX 急停。请求被拒绝或通信失败时保留 LAND 并限频重试，ACK 不算完成。
- 测距过期或无效：GUIDED 零速度保持，不用旧高度判定低空停桨。

超时退出后，CH6 仍授权且 Tag 重新有效时，可以自动恢复 FOLLOW。CH8 若仍保持高位，成功恢复 FOLLOW 会重新允许降落请求，随后在 GUIDED 心跳确认后再次发起 LAND 动作；CH8 已拨低则保持跟飞。
正常 LAND 阶段以 ON_GROUND 表示已落地，动作继续等待 `armed=false` 确认上锁完成。

JSON 的默认精准 `LAND` 使用该流程；任务层也可显式传入：

```json
{"action_id":"m42/hybrid-land","action":"LAND","timeout_s":60,"params":{"precision":true,"rc_managed":true,"guided_descent":true}}
```

配置参数：`land_recovery_height_m: 0.10`、`land_guided_descent_mps: 0.10`、
`land_recovery_timeout_s: 1.0`、`land_reacquire_dwell_s: 0.0`、
`land_yaw_tolerance_deg: 4.0`、`land_yaw_gain_per_s: 0.8`、
`land_maximum_yaw_rate_deg_s: 15.0`、`landing_alignment_maximum_age_s: 0.3`、
`land_center_tolerance_px: 20.0`、`land_alignment_dwell_s: 0.4`。
中心和朝向误差共同构成下降许可门槛，执行层使用同一份观测，不依赖任务层的 `target_aligned` 标记。状态 `GUIDED_ALIGN` 表示对准中，`GUIDED_VERIFY_ALIGNMENT` 表示驻留确认，`GUIDED_TRACK_DESCENT` 表示允许下降。0.10 m 交接后继续由飞控原生 LAND 执行末端落地，动作层不向 LAND 发送偏航/速度指令；既有末端锁存与低空丢标处理保持原有行为。
普通独立 DISARM 的许可不变；低空丢标上锁由 `allow_landing_disarm_output` 单独控制。
仅 hardware 和 sitl 配置打开此开关，offline/preview 保持关闭。

本地纯逻辑及真实 RC 编排方法测试：

```bash
PYTHONPATH=src:test python3 -m unittest test_action_execution test_guided_land_action test_landing_alignment test_guided_descent
```

ROS 模拟 MAVROS 测试脚本为 `test/smoke_action_executor_mode_flow_ros2.sh`；
模拟输入验证不等价于飞控接受普通空中 DISARM 或实机飞行验收。

## 启用边界

硬件输出要求同时满足：

- `environment: hardware`
- `flight_use_approved: true`
- 分别打开需要的 `allow_*_output`
- RC6 先稳定低位至少 0.4 秒，再拨到高位

普通 setpoint、普通上锁和急停分别有独立开关。不要因为要测试移动动作而顺便打开急停或上锁输出。

仓库提供三个部署覆盖配置：`action_executor.sitl.yaml`、`action_executor.hardware-preview.yaml` 和 `action_executor.hardware.yaml`。硬件配置保持独立 DISARM 与急停指令关闭，仅打开低空丢标普通 DISARM 请求；原生 LAND 的正常停桨/上锁由飞控落地检测负责。

不连接飞控也可以用模拟 MAVROS 心跳和模式服务复验完整模式链：

```bash
bash test/smoke_action_executor_ros2.sh
bash test/smoke_action_executor_mode_flow_ros2.sh
```

第二个测试会启动 ROS 2 执行器，模拟 CH6/CH8、Tag 丢失重捕获和测距，验证对准后 GUIDED 下降、偏离停止下降、恢复悬停、退出与 0.10 米 LAND 交接。

## 2026-10-05 重复帧与恢复限速修复

IBVS 轮询遇到 `DUPLICATE_FRAME` 时，如果上一份有效观测仍在 bridge 和 IBVS 特征有效期内，保留已经发布的证据，不发布新的候选或状态。原拍摄时间、候选时间戳和 landing_alignment 的接收时间均不刷新，画面冻结不能无限保持健康。有效期取 `max_message_age_ms` 与 `maximum_feature_age_s` 中更严格的一项。

超过有效期、真正丢标、HTTP 失败或质量校验失败仍发布不健康状态，并清除重复帧保留资格。无效帧之后的重复轮询不能恢复旧观测，必须等下一份通过校验的新帧。执行器自己的时效门也继续生效。

FOLLOW 丢标或候选速度缺失时，在立即输出零速度的同一周期同步清零限速器记忆和更新时间。恢复时从最近实际发送的零速度起步，遵守向量加速度上限。

验证命令：

```bash
PYTHONPATH=src:test python3 -m unittest test_ibvs_duplicate_frames test_action_execution test_guided_land_action test_landing_alignment test_guided_descent test_landing_disarm test_action_peripherals
```

本地 122 项离线测试通过，包含真实适配器方法、帧去重与时效门、观测接收方法和降落状态机的联合验证。该记录不代表树莓派部署或 ROS/SITL 飞控验证；光流/EKF 估计异常仍需要独立传感器证据。

## 2026-09-29 并行下降修正验证

以下是旧行为的历史验证记录。2026-10-05 起执行层改为先对准再下降，当前行为和参数以上文为准。

本次本地验证：`test_action_execution`、`test_guided_land_action`、`test_guided_descent`、
`test_landing_alignment`、`test_action_peripherals` 和 `test_moving_landing_stack.HybridGuidanceTests`
合计 83 项通过，修改后的 Python 模块语法检查通过。覆盖同时下降/居中/转向、FRD→FLU 符号、180°反向、
大小 Tag 公共 pad 姿态使用、源数据与接收时间累积过期、无效姿态恢复、重捕获、超时退出及等待 LOITER 心跳时的零速度/零转向，以及 setpoint 的 yaw-rate 掩码。

扩大回归时，`test_moving_landing_stack` 中两个原有用例失败：
`test_quality_gate_and_body_frd_packet` 与 `test_fuses_body_observation_and_aligned_ugv_odometry`。
两者预期使用旧外参 diag(-1,-1,1)，而当前配置是 [[0,-1,0],[1,0,0],[0,0,1]]。
仅在测试进程内恢复旧外参后两者通过；未修改实际外参或这些测试的预期。

ROS 模拟流程脚本已增加同时下降、居中和转向的消息断言，但本次未运行 ROS/SITL 或实飞验证。
2026-09-29 已在目标树莓派切换到 `action_executor.launch.py`；
部署记录见项目 `docs/ACTION_DIRECTION_DEPLOYMENT_20260929.md`。

用户描述的「高于最低高度丢 Tag 后的锁存」在本实现中叫 `GUIDED_REACQUIRE_HOLD`：
在 0.10 m 以上保持 CH8 降落动作、飞控 GUIDED 和零速度最多 1 秒；新鲜且质量通过的 Tag
及校准朝向在截止前恢复时，同一个动作继续并行下降/居中/转向。截止后按是否有新鲜 Tag 候选
分别退出到 GUIDED/LOITER。`HYBRID_LAND_COMMITTED` 是达到 0.10 m 后的另一种近地锁存，
已交接飞控原生 LAND，不会按高于阈值的 1 秒超时逻辑退出。
