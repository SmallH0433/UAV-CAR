# ROS 2 空地协同仿真与实机迁移工程

本软件包把 Gazebo Harmonic、ArduPilot SITL、ROS 2 Humble、Nav2、视觉与多传感器
避障组合成一个可闭环运行的无人机/无人车系统。仿真和实机共用任务、规划、控制仲裁、
状态与网页接口；迁移时替换设备驱动和执行器端，不把 Gazebo 真值带到实机业务层。

## 当前能力

- 无人车：Hunter V2 尺寸的 Ackermann 模型、2D LiDAR、相机、IMU、轮速里程计、
  EKF/AMCL、Nav2 Smac Hybrid（自主精降航段采用 Dubins 前进约束）与 Regulated Pure
  Pursuit 闭环；
- 底盘适配器：`diff_drive`、`ackermann`、`four_wheel_steering` 三种接口；
- 无人机：ArduPilot Copter SITL 与 Gazebo 电机/IMU/GPS 闭环；
- 无人机载荷：两轴云台 RGB、双目左右目与深度、360° 2D LiDAR、球形多线 3D LiDAR、
  前后左右上下六向超声波；
- 感知与避障：点云、深度、平面扫描和超声波健康检查、扇区融合、三维局部速度规划、
  地理围栏、禁飞柱与限高走廊；
- 精准降落：小车地图/里程计粗引导、下视 AprilTag 精引导、静止与移动甲板捕获条件，
  以及 Gazebo 可分离物理关节；
- 协同任务：远端飞来并首次落车、空地分离避障、目的地汇合、再次起飞跟车、移动落车、
  联合减速停车；
- 动态安全：旧视觉帧限时生效、近地分段下降、丢标保持/复飞路径、UGV 分阶段曲率保持
  限速、紧弯最低速度包线、里程计实测车速连续确认、Gazebo 固定关节真实状态确认，以及 MAVLink
  `EXTENDED_SYS_STATE` 落地确认后才允许移动降落或释放后的再次解锁；
- 操作台：浏览器实时状态、任务阶段、告警、轨迹、六路相机、传感器健康、云台、
  Nav2/UAV 目标、UGV 遥控，以及 Gazebo 暂停/继续/复位。

这里的 Jetson 是目标 ROS 2 计算平台，不是被虚拟化的硬件。仿真复用了将来运行在
Jetson 上的节点图、话题、参数和安全状态机；CPU/GPU 负载、温度、降频、供电、振动、
电磁干扰和真实网络必须在 Jetson 与整机上另行测试。

## 推荐环境

- Ubuntu 22.04（本机或 WSL 2）；
- ROS 2 Humble；
- Gazebo Harmonic；
- ArduPilot 与 `ardupilot_gazebo` 源码；
- Python 3、`pymavlink`、OpenCV；
- 网页操作台需要 Node.js 22+ 与 pnpm。

在当前仓库布局中：

```bash
export UAV_REPO=/mnt/d/Codex/UAV
export ARDUPILOT_DIR=$UAV_REPO/air_ground_open_source/01_flight_stack/ardupilot
export ARDUPILOT_GAZEBO_DIR=$UAV_REPO/air_ground_open_source/06_simulation/ardupilot_gazebo
export AIR_GROUND_WS=$UAV_REPO/simulation/air_ground_sim_ws
export GZ_VERSION=harmonic
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd "$AIR_GROUND_WS"
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/air_ground_sim/requirements.txt
colcon build --symlink-install --packages-select air_ground_sim
source install/setup.bash
```

若尚未编译 ArduPilot Gazebo 插件，先执行：

```bash
cd "$ARDUPILOT_GAZEBO_DIR"
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
```

## 一条命令运行完整任务

有桌面显示时：

```bash
source /opt/ros/humble/setup.bash
source "$AIR_GROUND_WS/install/setup.bash"
ros2 launch air_ground_sim cooperative_mission.launch.py \
  ardupilot_dir:="$ARDUPILOT_DIR"
```

WSL 或服务器上稳定的无头方式：

```bash
ros2 launch air_ground_sim cooperative_mission.launch.py \
  ardupilot_dir:="$ARDUPILOT_DIR" \
  headless:=true start_rviz:=false auto_start:=true
```

默认使用软件渲染，因为 WSL GPU 路径在多相机场景中不一定更快。物理步长保持
`1 ms`，让 ArduPilot 400 Hz 主循环和陀螺仪采样通过正常预检；任务不会强制解锁。
软件渲染时一个仿真秒可能对应多个现实秒，完整任务需要耐心等待。

## 启动网页操作台

另开 Windows 终端：

```powershell
cd D:\Codex\UAV\simulation\air_ground_sim_ws\src\air_ground_sim\web_ground_station
pnpm install
pnpm run dev -- --host 0.0.0.0
```

电脑打开 `http://localhost:3000`。同一局域网的 iPad 可打开
`http://<运行前端的电脑IP>:3000`。网页经 ROS 网关控制任务和仿真；实机配置默认关闭
网页写命令，并要求先配置令牌与 TLS 反向代理。

命令行可查看关键状态：

```bash
ros2 topic echo /mission/status
ros2 topic echo /uav/mavlink/status
ros2 topic echo /uav/perception/status
ros2 topic echo /uav/docking/status
ros2 topic echo /ugv/chassis_adapter/status
```

## 文档入口

- [COOPERATIVE_MISSION.md](COOPERATIVE_MISSION.md)：载荷、避障、状态机、可视化与完整演示；
- [DEPLOYMENT_SIMULATION.md](DEPLOYMENT_SIMULATION.md)：节点图、接口和单系统调试；
- [REAL_HARDWARE_MIGRATION.md](REAL_HARDWARE_MIGRATION.md)：Jetson/Pixhawk/Hunter 接线边界、
  标定、安全门槛和分阶段实机部署；
- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)：外部项目与许可范围。

## 安全边界

仿真通过只证明软件接口、坐标转换、规划控制和主要故障逻辑在模型条件下成立，
不等于实机安全认证。实机首次测试必须拆桨/架空轮、限制速度、使用封闭场地，保留
独立物理急停、AT9S Pro 人工最高优先级、飞控 failsafe、地理围栏和现场安全员。
移动平台着陆还需要真实减振甲板、定位标志、捕获/锁止机构与接触检测，不能用仿真
中的 `DetachableJoint` 代替机械设计。
