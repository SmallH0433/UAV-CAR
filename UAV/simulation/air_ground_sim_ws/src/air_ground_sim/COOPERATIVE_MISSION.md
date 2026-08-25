# 空地协同任务设计与演示

## 1. 目标场景

完整演示不是预录轨迹，而是由 ROS 2 状态机监督的闭环任务：

1. 无人机从世界原点附近正常预检、GUIDED 解锁并起飞；
2. 无人机从远处导航到位于 `(-9, -6)` 的无人车，识别车顶 AprilTag 并首次降落；
3. 仿真物理锁止，短暂停留后释放；
4. 无人机再次起飞，无人车由 Nav2、无人机由三维局部规划器同时前往汇合区；
5. 二者分别绕开地面/空中障碍，无人机同时遵守禁飞区、最高高度和局部限高区；
6. 小车停稳后无人机完成第二次视觉降落并锁止；
7. 无人机再次释放、起飞；小车以 Dubins 前进路径驶入清晰的北侧移动对接航段，
   状态机用里程计连续确认车速后，
   无人机保持相对跟飞；
8. 无人机在移动小车上完成第三次降落并锁止；
9. 状态机以独立 Nav2 目标接续剩余联合航段，小车携带无人机继续运动并平滑减速，
   最终停稳后任务进入 `COMPLETE`。

任何阶段超时、飞控预检失败、关键感知陈旧或控制源失活都会停止速度转发并进入
`FAULT`/`ABORTED`，不会强制解锁或绕过 ArduPilot 的 pre-arm 检查。

## 2. 系统分层

```text
Gazebo Harmonic（动力学、碰撞、传感器、空域可视物）
  ├─ ArduPilot Gazebo 插件 ↔ Copter SITL ↔ MAVLink bridge
  ├─ Hunter Ackermann + 传感器 ↔ ros_gz_bridge
  └─ UAV 多载荷 + DetachableJoint ↔ ros_gz_bridge
                          │
                          ▼
ROS 2 感知与定位
  ├─ UAV: 3D LiDAR + OV9281 stereo depth → 扇区/斥力融合
  ├─ UAV: 下视 OV9281 + ToF → 光流、AprilTag、甲板相对高度
  ├─ UAV: GNSS + HMC5883 + barometer + IMU → 飞行状态估计输入
  └─ UGV: wheel odom + IMU + AMCL + scan → EKF/map pose
                          │
                          ▼
规划、控制与安全仲裁
  ├─ UAV 三维速度采样 + 禁飞/限高预测
  ├─ UAV 粗定位/视觉精准降落控制
  ├─ UGV Nav2 Hybrid-A* + RPP
  └─ command mux / watchdog / enable gates
                          │
                          ▼
任务状态机 + Web Gateway + RViz/浏览器操作台
```

任务与规划层只读取 ROS/MAVLink 估计量，不读取 Gazebo 实体真值。唯一仿真专用的执行
机制是起落架与甲板间的可分离关节；实机必须把它替换成机械捕获和接触检测接口。

## 3. 无人机载荷与数据契约

| 载荷 | 仿真实现 | ROS 2 输出 | 主要用途 |
|---|---|---|---|
| GNSS / HMC5883 / 气压计 | NavSat、Magnetometer、AirPressure | `/uav/gnss/fix`、`/uav/magnetometer`、`/uav/barometer` | 位置、航向和气压高度输入 |
| 下视 OV9281 | 640×400、20 Hz、单色全局快门等效 | `/vision/image_raw` | 光流与 AprilTag 精准降落 |
| 下视 ToF | 9 ray 窄视场、30 Hz | `/uav/downward_tof/scan` | 光流尺度和甲板相对高度 |
| 双目 OV9281 | 12 cm 基线、同步参数 | `/uav/stereo/{left,right}/image_raw` | 双目深度与 VIO 输入 |
| 双目深度处理输出 | Gazebo RGB-D 代理实机 stereo_image_proc | `/uav/stereo/depth/depth_image` | 近场深度避障 |
| 3D LiDAR | 多线球形 GPU ray | `/uav/lidar3d/points` | 上下/侧向三维障碍 |

默认传感器分辨率/频率采用 WSL 软件渲染性能档。它们用于稳定的多传感器闭环测试，
不是 OV9281 的 120 Hz 硬件上限；实机配置中的超时更严格。

## 4. 无人机避障逻辑

`uav_perception` 将 3D LiDAR、双目深度和下视 ToF 转换到机体 FLU 坐标系并输出：

- 前、后、左、右、上、下六个最近障碍距离；
- 各来源最小值和帧率/数据年龄；
- 三维归一化斥力向量；
- `healthy`、`degraded_sensors` 与近障 `hard_stop`。

`uav_navigation` 对目标吸引速度、感知斥力和候选三维速度做预测评分。候选轨迹必须：

- 留在水平地理围栏和全局高度范围内；
- 不进入配置的圆柱禁飞区；
- 穿越限高矩形走廊时低于该区域上限；
- 不朝硬停止距离内的障碍继续推进；
- 在里程计、飞控、扫描或融合状态超时后输出零速度。

当前算法是迁移友好的轻量局部规划器，适合本任务与 Jetson 原型验证。若后续使用
EGO-Planner-v2、Fast-Planner 等更复杂规划器，应保留本项目的输入健康门、空域检查、
速度仲裁和飞控 failsafe，不要直接把规划器输出接到飞控。

## 5. 精准降落与移动平台跟随

降落控制分两层：

1. 粗引导使用 UAV `/uav/odom`、UGV `/amcl_pose` 和 `/odometry/filtered`。AMCL 在
   小车静止时可能不重复发布，因此控制器保存最近地图锚点并用新鲜里程计传播当前
   地图位姿；仿真先到 2.6 m 安全等待点，再以地图闭环下降到 1.8 m 视觉获取高度；
2. 标签进入下视画面后切换到视觉精引导，以图像中心误差控制机体前/左速度，并在
   对中、标签面积、相对高度、数据新鲜度和障碍状态全部满足时才允许 `capture_ready`。

视觉控制不持续复用旧图像：横向命令只在新鲜帧窗口内有效；已对中的高空下降使用独立
受限窗口；进入捕获高度上方 0.55 m 后自动切换到更短的新鲜度窗口和 0.10 m/s 近地下降。
下视相机采用无遮挡的机腹外置安装和 130° 广角精降镜头，使实体甲板接触高度仍能看到
360 mm 标志的完整编码区；这项几何约束不能用放宽 `capture_ready` 代替，实机必须按
真实起落架、镜头畸变、相机和标志尺寸复核。

每次锁止后由 ArduPilot `LAND` 判定并自动解锁。释放后，状态机必须同时收到锁扣
`detached`、MAVLink `MAV_LANDED_STATE_ON_GROUND`，并经过按实时率缩放的稳定窗口，才会
再次请求解锁。`NAV_TAKEOFF` 采用 5 秒限频重试，但每次仍由 ArduPilot 正常拒绝或接受，
不使用 force-arm，也不伪造落地状态。
短时丢标进入 `visual_reacquire_hold`，依靠地图锚点与里程计保持相对位置和高度，持续
丢失才退回粗引导。这样既能适配软件渲染低实时率，也不会把过期图像当成新观测。

`stopped` 模式要求小车静止；`follow` 模式保持高度与相对距离；只有
`/odometry/filtered` 实测平面速度不低于 0.04 m/s 且连续 2 秒，状态机才允许切入
`moving` 模式。该模式传播移动甲板位置并以较慢下降率对中，而且在捕获瞬间再次要求
连续速度门成立，防止把“已发送 Nav2 目标”或“已经停下”误当成移动降落。控制优先级固定为：

```text
精准降落 > 受保护的视觉跟随 > 普通三维导航 > 零速度
```

每个控制源必须同时提供新鲜命令和 `active=true` 状态，否则仲裁器立即退回下一安全
来源或零速度。捕获后任务先停止 UAV 速度转发、发布挂接请求，再由 SITL 正常卸载/解锁。
Gazebo 使用 UAV 顶层 `dock_link` 创建/移除真实固定关节，并在成功后发布
`attached` / `detached`；状态机不再把“发出命令”等同于“机构已确认”。

UGV 的比例限速使用 Nav2 标准 `/speed_limit`，在闭环 `velocity_smoother` **之前**同时
约束线速度及其对应曲率；`/ugv/speed_scale` 在底盘适配器处只承担 0/1 最终安全门控。
这样避免“闭环平滑器先按里程计加速、适配器随后再按比例降速”形成低速反馈死区。并行
运输使用 100%，跟飞与移动对接使用 15%（当前 RPP 标称约 0.09 m/s），锁止后的联合
航段再从 15% 平滑降至 8%（约 0.048 m/s），最后由 Nav2 目标检查器、速度平滑器与
底盘加速度限制共同完成停车。移动捕获点和最终停车点使用两个独立 Nav2 目标；旧目标由
代次号保护，迟到的 Action 回调不会覆盖新目标状态。

自主任务的 Smac Hybrid 采用前进约束的 Dubins 模型，Pure Pursuit 禁止自动倒车，避免
移动对接前出现 Reeds-Shepp 倒车 cusp。底盘适配器和人工遥控仍接受有符号线速度，因此
救援/人工倒车能力没有被删除；如实机确需自主倒车，应使用独立行为树并禁止进入精降状态。
并行汇合目标的终端航向与直达路径切线一致，避免前进约束为满足不合理航向而生成贴近边界的
大回环；每个实机场景也应按道路方向设置 staging pose，而不只设置目标点坐标。

## 6. 任务状态机

| 阶段 | 主要状态 | 通过条件 |
|---|---|---|
| 远端接近 | `WAIT_AUTOPILOT` → `ARM_INITIAL` → `TAKEOFF_INITIAL` → `NAVIGATE_TO_START_DOCK` | 飞控自身预检、GPS、本地位置、正常解锁、到达小车上方 |
| 首次落车 | `DOCK_AT_START` → `LATCH_AT_START` → `DWELL_AT_START` | 标签精引导满足捕获门、物理挂接 |
| 并行作业 | `RELEASE_FOR_TRANSIT` → `TAKEOFF_FOR_TRANSIT` → `PARALLEL_TRANSIT` | UAV 航点与 UGV Nav2 目标均成功 |
| 静态汇合 | `DOCK_STOPPED` → `LATCH_STOPPED` → `DWELL_STOPPED` | 静止甲板视觉捕获与挂接 |
| 跟车 | `RELEASE_FOR_FOLLOW` → `TAKEOFF_FOR_FOLLOW` → `FOLLOW_MOVING_UGV` | 跟飞至少 8 秒、距离门满足、实测车速连续达标 |
| 移动落车 | `DOCK_MOVING` → `LATCH_MOVING` | 捕获瞬间实测车速仍连续达标、移动甲板视觉捕获、物理挂接和飞控落地确认 |
| 联合停车 | `RIDE_AND_DECELERATE` → `COMPLETE` | 接续独立 Nav2 最终目标、曲率保持减速、车辆到达并停止 |

仿真配置的 `timeout_scale: 8.0` 只补偿 WSL 软件渲染的低实时率。实机使用
`real_interfaces.yaml` 的严格数据超时，不能照搬该放大系数。

## 7. 启动与观察

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/Codex/UAV/simulation/air_ground_sim_ws
source install/setup.bash

# 有 Gazebo GUI 与 RViz
ros2 launch air_ground_sim cooperative_mission.launch.py

# 无头稳定运行
ros2 launch air_ground_sim cooperative_mission.launch.py \
  headless:=true start_rviz:=false auto_start:=true
```

若 `ARDUPILOT_DIR` 不在默认搜索位置，显式传入
`ardupilot_dir:=/absolute/path/to/ardupilot`。

网页端：

```bash
cd /mnt/d/Codex/UAV/simulation/air_ground_sim_ws/src/air_ground_sim/web_ground_station
pnpm install
pnpm run dev -- --host 0.0.0.0
```

打开 `http://localhost:3000`。可在页面查看任务时间线、ArduPilot 模式与预检、UAV/UGV
位置、Nav2 路径、障碍距离、每个传感器新鲜度、云台/下视/标签/双目/车载画面；也可
启动、暂停、中止任务，发送测试目标、遥控 UGV、调整云台及暂停/继续 Gazebo。

RViz 适合看 TF、地图、代价地图、激光与路径；Gazebo 适合看动力学、模型和碰撞；
浏览器操作台适合任务监督。三者用途不同，可以同时运行。

## 8. 验收与故障定位

完整任务应最终看到：

```bash
ros2 topic echo --once /mission/status
# state: COMPLETE
```

常用检查：

```bash
ros2 topic echo /uav/mavlink/status
ros2 topic echo /uav/perception/status
ros2 topic echo /uav/navigation/status
ros2 topic echo /uav/docking/status
ros2 topic echo /apriltag/status
ros2 topic echo /mission/events
ros2 action list | grep navigate_to_pose
```

- 长时间停在 `WAIT_AUTOPILOT`：查看 MAVLink `status_texts`，修正飞控预检原因；
- 导航不动：检查 `flight_ready`、感知 `healthy`、空域拒绝原因和 command mux 模式；
- 降落悬停：检查标签是否可见、标签面积、相机/起落架几何、UGV/UAV 里程计年龄和
  `capture_ready`；
- 释放后不起飞：检查 `dock_detached` 与 MAVLink `landed_state_name`，不要跳过
  `ON_GROUND` 门；
- UGV 无路径：检查 AMCL、TF、Nav2 lifecycle 和 `/scan`；
- 长时间停在 `FOLLOW_MOVING_UGV`：检查 `ugv_measured_speed_mps`、
  `ugv_moving_confirmed`、Dubins 路径是否存在，以及平滑器/底盘适配器是否持续输出前进速度；
- 页面无图：先检查 ROS 图像话题，再检查 `http://127.0.0.1:8765/api/status`。

不要通过 force-arm、删除传感器健康门或扩大捕获高度来“修好”演示；这些会掩盖最有
价值的实机风险。

## 9. 参考路线

本实现参考了 ArduPilot/Gazebo/ROS 2 的官方接口和成熟开源项目的架构思想，未复制
ZJU FAST Lab、EGO-Planner-v2 或 Fast-Planner 的源码。相关入口：

- ZJU FAST Lab：https://github.com/ZJU-FAST-Lab
- EGO-Planner-v2：https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
- Fast-Planner：https://github.com/HKUST-Aerial-Robotics/Fast-Planner
- AprilTag：https://github.com/AprilRobotics/apriltag
- ArduPilot：https://github.com/ArduPilot/ardupilot
- ros_gz：https://github.com/gazebosim/ros_gz

这些项目的 ROS 版本、飞控栈、许可证和状态估计假设并不完全相同，因此这里只采用
可兼容的设计原则。替换规划器前应单独评审许可证、坐标系、动态约束和失效行为。
