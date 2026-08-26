# OV9281 移动平台降落集成包

本目录补齐工作区原先只在文档中定义的三个模块，并增加 Elastic/IBVS/AC_PrecLand 的单写入者组合层：

- `landing_target_bridge`：读取 OV9281 `http://127.0.0.1:8765/api/status`，检查标签、时间戳和质量，将相机光学坐标转换为 `MAV_FRAME_BODY_FRD`，生成三维 `LANDING_TARGET`；
- `moving_pad_estimator`：把 BODY_FRD 标签观测与无人机 LOCAL_NED 状态组合，可选融合已经对齐共同原点的 HUNTER `/odom`，输出小车位置、速度、协方差和短时预测；
- `moving_landing_supervisor`：执行会合、跟踪、速度匹配、下降、最终进近、触地、完成和中止状态机，只输出受约束的请求，不直接操作电机。
- `hybrid_guidance`：提取 Elastic-Tracker 的远端轨迹状态和 ibvs_sim 的近端角点控制思想，在 `ELASTIC_GUIDED -> IBVS_GUIDED -> AC_PRECLAND_LAND` 之间做互斥仲裁。

组合原理、控制权边界和分阶段实验见 [AC_PrecLand + Elastic-Tracker + IBVS 组合实验原理](docs/HYBRID_EXPERIMENT_PRINCIPLE.md)。ROS 2 适配、模式 ACK/回滚、RC 门和双标签约定见 [ROS 2 GUIDED 与双标签说明](docs/ROS2_GUIDED_DUAL_TAG.md)。

## 当前安全边界

`config/moving_landing.prototype.json` 默认只能用于离线回放、SITL 和拆桨台架：

- MAVLink 发送、模式切换、解锁和电机指令全部关闭；
- `landing-target-bridge` 默认 dry-run，不会打开飞控串口；
- 即使使用 `--transmit`，仍必须同时把配置中的 MAVLink、相机外参和飞行批准三个开关设为真，否则程序拒绝启动；
- 当前连续移动触地关闭。最终进近会请求小车停车；触地必须由飞控 landed 状态或接触传感器确认，不能仅凭 11 cm 测距读数判断。
- Elastic、IBVS 和 AC_PrecLand 永远不能同时成为控制写入者；离线仲裁输出采用 one-hot 授权。

## 安装与 dry-run

在树莓派或 Ubuntu 主机执行：

```bash
cd 08_air_ground_landing
python3 -m pip install -e .

landing-target-bridge \
  --config config/moving_landing.prototype.json \
  --duration-s 10
```

当前 OV9281 服务默认同时检测同心嵌套 `tag36h11 / ID 0 / 黑边 0.100 m` 与 `ID 1 / 黑边 0.020 m`。桥接器按主标签 ID 核对对应尺寸，拒绝尺寸不一致的位姿。

ROS 2 Humble 适配包位于 `ros2_ws/src/air_ground_landing_ros2`。它以标准 `MultiDOFJointTrajectory` 接收 Elastic 轨迹，从 OV9281 状态生成 IBVS 水平速度候选，并通过带 HEARTBEAT ACK、超时回滚和 RC 授权门的唯一执行器连接 MAVROS。默认只发布 preview，不会切模式或写飞控。

遥控职责固定为：`CH5` 是 ArduPilot 飞行模式通道；`CH6` 开启或关闭跟飞；`CH7` 选择 EKF 定位源（低位光流，中/高位 GPS）；`CH8/SwD` 是独立下降开关。只有飞控 HEARTBEAT 已确认处于 GUIDED 跟飞，且 SwD 在该次跟飞中先回到低位再拨到高位，才发布下降请求。SwD 关闭会取消下降并返回速度匹配/定高跟飞；CH6 关闭则撤销整套自动控制。CH6 与 CH8 只由伴随计算机读取原始 PWM，CH7 由 ArduPilot `RC7_OPTION=90` 选择已经配置好的 EKF 源组。

## 离线/SITL联合回放

`moving-landing-replay` 从 JSONL 逐帧读取以下数据：

```json
{
  "timestamp_s": 1.0,
  "mission_enabled": true,
  "operator_authorized": true,
  "descent_requested": false,
  "vision_status": {"sensor": "ov9281", "mode": "apriltag"},
  "uav": {
    "position_ned_m": [0, 0, -1],
    "velocity_ned_mps": [0.1, 0, 0],
    "quaternion_body_to_ned": [1, 0, 0, 0],
    "mode": "LOITER",
    "armed": true,
    "landed": false,
    "link_healthy": true,
    "velocity_source_independent_of_deck": false
  },
  "ugv": {
    "position_ned_m": [0, 0, 0],
    "velocity_ned_mps": [0.1, 0, 0],
    "healthy": true,
    "emergency_stop": false,
    "common_origin_valid": false
  },
  "rangefinder_distance_m": 1.0
}
```

运行：

```bash
moving-landing-replay \
  --config config/moving_landing.prototype.json \
  --input snapshots.jsonl \
  --output decisions.jsonl
```

输出同时包含桥接结果、平台估计和协调器决策，并固定标记 `mavlink_transmitted=false`、`vehicle_command_transmitted=false`。

若输入还包含 OV9281 `overlay_points`（四个 Tag 角点）和 `elastic_tracker` 健康状态，输出会增加 `ibvs_features` 与 `hybrid_guidance`。例如：

```json
"elastic_tracker": {
  "heartbeat_healthy": true,
  "map_fresh": true,
  "target_prediction_fresh": true,
  "trajectory_valid": true,
  "visibility_corridor_valid": true,
  "trajectory_id": 7
}
```

## 坐标契约

- OV9281 光学系：`x` 向图像右、`y` 向图像下、`z` 沿镜头前方；
- 飞控机体系：BODY_FRD，前、右、下；
- 估计器公共系：LOCAL_NED；
- ROS ENU `/odom` 不能直接与 LOCAL_NED 混合。必须先完成原点和平面偏航对齐，再把 `common_origin_valid` 设为真。

## 尚未批准的实机条件

以下条件全部验证前，不得打开连续移动触地：

1. OV9281 相机到机体的完整三维外参；
2. 10 cm 标签的多距离测距校验；
3. 约 2 cm 近距离标签的打印、对焦和识别验证；
4. HUNTER 里程计与无人机 LOCAL_NED 的共同原点；
5. 不受移动甲板污染的无人机水平速度源；
6. MTF-01P 在真实甲板上的重复触地读数和接触/landed 双重确认。
