# 面向实机的单系统仿真架构

`deployment_sim.launch.py` 用于分别调试无人车 Nav2 和无人机导航/避障；
`cooperative_mission.launch.py` 在其上增加任务状态机、精准降落、可分离甲板、协同场景、
RViz 与网页网关。完整任务见 [COOPERATIVE_MISSION.md](COOPERATIVE_MISSION.md)，实机
分阶段部署见 [REAL_HARDWARE_MIGRATION.md](REAL_HARDWARE_MIGRATION.md)。

## 1. 无人车闭环

```text
/scan + /ugv/imu/data + /ugv/wheel/odometry
       │
       ├─ robot_localization EKF ── /odometry/filtered ── odom→base_link
       └─ AMCL + static map ───────────────────────────── map→odom
                                  │
                                  ▼
      Nav2 Smac Hybrid/Dubins + Regulated Pure Pursuit + obstacle layers
                                  │ /cmd_vel
                                  ▼
  ChassisAdapter(diff/ackermann/4WS) ── /ugv/cmd_vel
                                  │
                                  ▼
             rate limit + enable + watchdog gateway
                 ├─ sim: /ugv/sim/cmd_vel → Gazebo
                 └─ real: /hunter_base/cmd_vel → CAN driver
```

Hunter 仿真模型使用 0.650 m 轴距、0.605 m 轮距、0.165 m 轮半径和前轮 Ackermann
转向，不能原地旋转。Nav2 使用非完整约束规划与无 Spin 恢复行为，动态障碍不写入静态
地图，只能由实时 `/scan` 进入代价地图后绕开。

三种底盘适配器统一接收 `Twist`：

```bash
ros2 launch air_ground_sim deployment_sim.launch.py ugv_adapter:=ackermann
ros2 launch air_ground_sim deployment_sim.launch.py ugv_adapter:=diff_drive
ros2 launch air_ground_sim deployment_sim.launch.py ugv_adapter:=four_wheel_steering
```

`four_wheel_steering` 当前完成曲率、限幅和接口适配；接实车时仍需按具体底盘驱动协议
实现四个转角或前/后桥指令输出。

## 2. 无人机闭环

```text
ArduPilot LOCAL_POSITION_NED ── MAVLink bridge ── /uav/odom (ENU)
2D scan + 3D points + stereo depth + six Range
                  │
                  ▼
       health/freshness + 3D obstacle fusion
                  │
/uav/nav/goal → candidate velocity planner + airspace prediction
                  │ /uav/nav/cmd_vel
AprilTag follow ──┼─ command mux ── /uav/cmd_vel
docking control ──┘                 │
                                    ▼
                         MAVLink BODY_NED → ArduPilot
```

仲裁优先级是精准降落、视觉跟随、普通导航、零速度。每一层都有 enable gate 和
watchdog；MAVLink 断联、飞控状态不满足、里程计/感知超时、近障硬停止或空域预测失败
都会抑制前进命令。

`deployment_sim.launch.py` 默认不启动任务，也不会自行解锁。可在操作员完成 SITL
检查和起飞后单独测试：

```bash
ros2 service call /uav_navigation/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /uav/nav/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: uav_odom}, pose: {position: {x: 5.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}"
```

完整协同仿真在 `simulation_lifecycle=true` 时可调用 SITL 的正常 GUIDED、arm、takeoff、
land 服务；它仍要求 ArduPilot 自己报告 pre-arm passed，绝不 force-arm。实机配置将
`allow_lifecycle_commands` 设为 `false`，由飞手和飞控策略负责放权。

## 3. 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/Codex/UAV/simulation/air_ground_sim_ws
colcon build --symlink-install --packages-select air_ground_sim
source install/setup.bash

ros2 launch air_ground_sim deployment_sim.launch.py \
  ardupilot_dir:=/mnt/d/Codex/UAV/air_ground_open_source/01_flight_stack/ardupilot
```

只验证无人车：

```bash
ros2 launch air_ground_sim deployment_sim.launch.py \
  world:=hunter_navigation_test.sdf \
  start_sitl:=false start_uav_interfaces:=false start_uav_navigation:=false
```

发送 Nav2 目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 7.5, y: 5.5}, orientation: {w: 1.0}}}}"
```

关键检查：

```bash
ros2 topic hz /scan
ros2 topic hz /ugv/wheel/odometry
ros2 topic hz /odometry/filtered
ros2 topic echo /ugv/chassis_adapter/status
ros2 lifecycle get /bt_navigator

ros2 topic echo /uav/mavlink/status
ros2 topic echo /uav/perception/status
ros2 topic echo /uav/navigation/status
ros2 topic echo /uav/command_mux/status
```

## 4. RViz、Gazebo 与网页的分工

- Gazebo：模型、碰撞、动力学、传感器视锥与世界障碍；
- RViz：地图、TF、里程计、激光、点云、Nav2 路径/代价地图和目标；
- 网页操作台：任务阶段、设备健康、相机、告警、控制入口和 iPad 访问。

RViz 不是飞控，也不等于 Gazebo。它是 ROS 2 数据可视化与目标交互工具；关闭 RViz
不影响任务节点运行。

## 5. 仿真与实机配置差异

| 项目 | `sim_interfaces.yaml` / cooperative override | `real_interfaces.yaml` |
|---|---|---|
| 时间 | `use_sim_time=true` | 系统/硬件时间 |
| Pixhawk | UDP SITL | 串口真实飞控 |
| 生命周期 | 协同演示可正常 arm/takeoff/land | 默认禁止软件生命周期指令 |
| 超时 | 对低 RTF 有限放宽 | 0.3–0.8 s 严格超时 |
| UGV 输出 | Gazebo Ackermann | Hunter CAN 驱动 |
| 传感器 | ros_gz bridge | 厂商 ROS 2 驱动/remap |
| 网页写操作 | 仿真可启用 | 默认关闭，需认证/TLS |
| 甲板捕获 | DetachableJoint | 必须实现机械硬件接口 |

业务层不得通过 `use_sim_time`、Gazebo 服务或实体真值判断自己处于仿真。profile 差异
集中在 launch、驱动和参数层，才能在迁移时保持同一套任务逻辑。

## 6. 当前模型边界

- Jetson、树莓派并未被虚拟成 CPU/GPU/温度模型；
- LiDAR 和相机可验证几何、消息与算法链路，但不覆盖所有多径、曝光和运动模糊；
- 超声波使用共享三维几何加独立传输误差，不是声学波场；
- Hunter 模型不覆盖真实轮胎、悬架、地面附着和 CAN 故障；
- SITL 不覆盖真实供电、桨叶、振动、EMI 与传感器安装误差；
- 移动平台着陆模型不能替代甲板与锁止机构的机械安全验证。

因此验收顺序应始终是单元测试 → SIL → HIL → 拆桨/架空轮 → 封闭低速 → 静止甲板
→ 低速跟车 → 移动着陆，而不是从仿真 `COMPLETE` 直接进入无人值守实飞。
