# AC_PrecLand + Elastic-Tracker + IBVS 组合实验原理

## 结论先行

三套方法不能并联控制飞机。组合时必须保证任意时刻只有一个制导写入者：

1. `Elastic-Tracker` 只负责远端会合、保持目标可见和避障轨迹；
2. `ibvs_sim` 提取出的 IBVS 只在 `GUIDED` 阶段完成图像对准与速度同步，并在下降后作为图像特征安全门；
3. `AC_PrecLand` 在进入 `LAND` 后独占水平精准降落和垂直下降控制；
4. `moving_landing_supervisor` 决定阶段，`hybrid_guidance` 决定当前唯一控制权，二者都只输出请求。

`Elastic-Tracker` 的原始 `PositionCommand`、`ibvs_sim` 的 MAVROS 速度指令和 ArduPilot `LAND` 控制不能同时发送。

## AC_PrecLand 的移动目标原理

当前锁定的 ArduCopter 源码中：

- `PLND_TYPE=1` 选择 MAVLink companion backend；
- `LANDING_TARGET.position_valid=1` 时只接受 `MAV_FRAME_BODY_FRD`，用 `x/y/z` 和正的 `distance` 构造目标视线；
- MAVLink `time_usec` 会先被飞控修正到本机时间，再结合 `PLND_LAG` 回看对应时刻的姿态和速度；
- `PLND_EST_TYPE=1` 的内部二维 Kalman 滤波器估计“目标相对飞机”的水平位置和速度；
- `PLND_OPTIONS` bit 0 开启后，飞控把“相对速度估计 + 飞机自身速度”还原成移动目标绝对速度，并作为位置控制器的速度前馈；
- LAND 控制器同时把目标位置作为水平位置目标，把目标速度作为水平速度目标；当横向误差超过 `PLND_XY_DIST_MAX` 时暂停下降。

因此“移动目标模式”并不是 MAVLink 额外发送小车速度。桥接器仍发送每帧相对位姿；AC_PrecLand 根据连续观测和飞机惯导自行估计目标速度。外部小车里程计主要用于提前会合、交叉验证和协调器门控。

关键源码位置：

- `libraries/AC_PrecLand/AC_PrecLand_Companion.cpp`：MAVLink BODY_FRD 观测入口；
- `libraries/AC_PrecLand/AC_PrecLand.cpp`：延迟补偿、目标 EKF 和移动目标速度；
- `ArduCopter/mode.cpp`：目标位置/速度写入水平位置控制器以及下降门限。

## AprilTag → MAVLink 桥接

OV9281 服务给出相机光学系位姿：右、下、前。桥接器按已配置外参转换为机体 BODY_FRD：前、右、下，然后执行：

1. 检查唯一 `analysis_sequence`，防止把同一张 10 Hz 图像重复伪装成 20 Hz；
2. 检查采集时延、Tag ID/边长、decision margin、hamming、重投影误差和距离一致性；
3. 生成 `LANDING_TARGET`：`frame=BODY_FRD`、`position_valid=1`、`x/y/z/distance`；
4. 保留真实采集时间，使飞控能把视觉观测与历史惯导状态对齐；
5. 丢标时停止发布，不能发送“最后一次位置”冒充新观测。

当前 10 cm Tag 对应 ID 0。近地阶段需要候选 2 cm Tag（ID 1），但它仍处于禁用和待验证状态。

## 从 Elastic-Tracker 提取什么

原仓库是 ROS 1、自定义 SO3 控制器和 MINCO 多项式轨迹，不能把 `/position_cmd` 原样接到 ArduPilot。可提取的核心是：

- 由平台位置/速度预测短时间目标轨迹；
- 在占据栅格中搜索安全路径和飞行走廊；
- 用可见性约束保证相机与目标之间无遮挡；
- 以无人机当前位置、速度、加速度为初始状态，以平台预测位置和速度为终端状态滚动重规划；
- 规划失败时悬停或继续已验证的上一条轨迹。

组合适配器只接收其“轨迹有效、走廊有效、目标预测新鲜、heartbeat 正常”等状态。未来真正接入时，应把轨迹采样为有速率/加速度限制的 ArduPilot `GUIDED` LOCAL_NED 请求，并单独验证 ENU→NED、时间戳和模式 ACK。

## 从 ibvs_sim 提取什么

原仓库的主要思想是：

- 用四个标签角点构成图像特征向量；
- 误差大时使用 2-DOF 质心平移，避免全自由度耦合；
- 质心接近目标后用带滞回的 4-DOF 特征判断尺度、形状和对准；
- 使用外/内嵌套标签完成远近距离切换；
- IBVS 水平修正上叠加移动平台速度前馈。

本项目没有移植其直接降推力、自动反解锁和 PX4/MAVROS 控制代码。`IbvsFeatureController` 只产生有速度上限的 BODY_FRD 水平修正请求；垂直速度和偏航不输出。进入 AC_PrecLand 后，IBVS 只做交接稳定性及最终近距标签门控。

简化的质心控制为：

```text
e = [(u-cx)/fx, (v-cy)/fy]
v_camera_xy = gain × Z × e
v_body_xy = R(camera→BODY_FRD) × v_camera_xy
v_guided = v_pad_feedforward + saturate(v_body_xy)
```

其中 `Z` 来自已经通过质量门的 AprilTag 距离。该请求只允许在协调器选择 `IBVS_GUIDED` 时使用。

## 单写入者状态交接

| 监督状态 | 唯一控制者 | 作用 | 交接条件 |
|---|---|---|---|
| `RENDEZVOUS` | `ELASTIC_GUIDED` | 飞到预测会合点、避障、保持可见 | 规划/地图/目标预测均新鲜 |
| `TRACK_PAD` | `ELASTIC_GUIDED` | 跟踪平台并缩小相对位置误差 | 进入跟踪半径 |
| `MATCH_VELOCITY` | `IBVS_GUIDED` | 图像居中 + 小车速度前馈 | 图像误差和相对速度稳定 |
| `DESCEND` | `AC_PRECLAND_LAND` | ArduPilot 精准下降 | IBVS 对准稳定、目标流新鲜、监督器授权 |
| `FINAL_APPROACH` | `AC_PRECLAND_LAND` | 近地精准降落 | 小标签/接触条件、速度和偏航率均合格 |
| `ABORT` | `HOLD` | 退出下降，要求小车停车 | 操作员重新处置 |

`hybrid_guidance.py` 对上述授权做 one-hot 输出，任何输入超龄或冲突都关闭下降。

## 推荐实验顺序

### 阶段 0：纯离线回放

- 把 OV9281 `/api/status`、飞机 LOCAL_NED、小车对齐后的里程计和模拟 Elastic 状态写成 JSONL；
- 验证每一帧只有一个 `control_owner`；
- 注入丢标、旧时间戳、轨迹失效、地图失效和里程计跳变；
- 所有输出保持 `mavlink_transmitted=false`、`vehicle_command_transmitted=false`。

### 阶段 1：SITL 静态平台

- 先不用 Elastic 和 IBVS 主动控制，只验证 AprilTag 桥接与 AC_PrecLand；
- 使用 `PLND_ENABLED=1`、`PLND_TYPE=1`、`PLND_EST_TYPE=1`；
- `PLND_OPTIONS` 先为 0，验证静态精准降落；
- 参数数值必须根据日志中的真实时延、量程和误差设置，不直接照抄论文。

### 阶段 2：SITL 移动平台但不下降

- `PLND_OPTIONS` 仅开启 bit 0，即数值 1；不要同时开启 fast descent bit；
- 验证 AC_PrecLand 估计的目标速度方向和小车里程计一致；
- Elastic 只运行会合/跟踪，IBVS 只记录误差，不切换 LAND。

### 阶段 3：SITL 完整交接、真机小车停车后落地

- Elastic → IBVS → AC_PrecLand 按状态依次交接；
- 首次真机最终进近要求小车停车；
- 丢标、相对速度超限、IBVS 不稳定或模式 ACK 失败立即 HOLD；
- 触地必须由接触传感器或飞控 landed 状态确认。

### 阶段 4：连续移动触地

只有近距小标签、共同原点、独立于移动甲板的无人机速度源、测距重复性和接触检测全部通过后才允许。当前配置仍明确禁止此阶段。

## 目前仍需完成的硬件适配

1. OV9281 服务支持同时识别 ID 0/ID 1，并按各自真实边长解算位姿；当前服务一次只配置一个 ID；
2. Elastic ROS 1 输出到 ArduPilot GUIDED 的受限适配器；
3. 飞行模式切换的 ACK、超时和回滚执行层；
4. HUNTER ENU `/odom` 与飞控 LOCAL_NED 的共同原点标定；
5. 2 cm 近距标签识别、焦距和触地遮挡验证。

这些项目完成前，新增代码保持离线/SITL 请求层，不获得真机控制权。
