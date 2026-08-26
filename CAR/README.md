# CAR — R680 阿克曼转向小车

树莓派 4B + STM32 下位机（WHEELTEC R680 阿克曼底盘：前轮舵机转向 +
后轮双编码器电机驱动）的 ROS 2 Humble 项目。注意阿克曼底盘**不能横移、
不能原地自旋**，最小转弯半径由轴距和最大转向角决定。
主控制架构为 `car_nodes` 自定义轻量节点；Gazebo 仿真设施（`car_sim`）移植自
`UAV/simulation/air_ground_sim_ws/src/air_ground_sim`，不含 Nav2。

## 包结构

- `car_interfaces` — 自定义 msg/srv（AckermannCommand / MotorFeedback / Obstacle(Array) / UavCommand / UavStatus / SetGoal）
- `car_nodes` — 8 个功能节点（含 `ultrasonic_driver` 车尾 HC-SR04 超声波，实机脱困倒车盲区急停）+ `sim_motor_bridge`（仿真电机桥）
- `car_description` — R680 URDF + Gazebo 模型 `models/r680_4wd`
- `car_sim` — gz 世界、ros_gz_bridge 配置、控制权 mux、指令网关、网页遥控、实机 bringup launch（`real_bringup.launch.py`）、Nav2 备用栈（`config/nav2_params.yaml` + `launch/nav2_stack.launch.py`，见下文「Nav2 自主导航（已屏蔽）」）
- `vendor/lslidar_ros2` — 镭神 N10P 雷达厂商 ROS2 SDK（lslidar_driver + lslidar_msgs，实机用）
- `vendor/wheeltec_gps` — WHEELTEC G60 GPS 厂商 ROS2 SDK（nmea_msgs + nmea_navsat_driver
  + wheeltec_gps_path + wheeltec_udev.sh，实机用；wheeltec 修改版，支持 $GN/$GL talker）

## 架构（仿真话题链）

```
[浏览器] POST /api/ugv/teleop ──→ /ugv/teleop/cmd_vel + /ugv/operator/heartbeat
[avoidance_node] ──→ /cmd_vel（避障/导航指令）
        │
        ▼
ugv_control_mux            （teleop 优先；无 operator heartbeat 时放行 /cmd_vel）
  订阅: /cmd_vel, /ugv/teleop/cmd_vel, /ugv/operator/heartbeat, /system/emergency_stop
  发布: /ugv/control/cmd_vel, /ugv/speed_scale, /ugv/control_mux/status
        ▼
ugv_command_gateway        （限幅 ±1.0 m/s、±1.0 rad/s；0.5s 超时停车；20Hz 重发）
  输入 input_topic:=/ugv/control/cmd_vel（参数覆盖默认值 /ugv/cmd_vel）
  输出 output_topic:=/ugv/gateway/cmd_vel（参数覆盖默认值 /ugv/sim/cmd_vel）
        ▼
chassis_controller         （其 /cmd_vel 输入 remap 自 /ugv/gateway/cmd_vel）
  → /ackermann_cmd（自行车模型：后轮速[左后/右后] rad/s + 前轮转向角 rad）
  → /odom + TF(odom→base_footprint)（优先走 /motor_feedback 真实反馈）
        ▼
sim_motor_bridge           （仿真里替代实机 motor_driver）
  /ackermann_cmd → (v,w) → /ugv/sim/cmd_vel ──[ros_gz_bridge]──→ gz /model/ground_vehicle/cmd_vel
  gz /model/ground_vehicle/odometry ──[桥]──→ /ugv/wheel/odometry → 反算后轮速+转向角 → /motor_feedback
```

传感器桥（gz → ROS）：`/model/ground_vehicle/scan` → `/scan`（→ perception_node →
`/perception/obstacles` → avoidance_node）；`/model/ground_vehicle/front_camera` →
`/camera/image_raw`、`/model/ground_vehicle/rear_camera` → `/camera/rear/image_raw`
（+各自 `camera_info`，lazy）；`/model/ground_vehicle/imu` → `/imu/data`（备用）。
网页遥控页前后双画面：`/api/camera.jpg`（前视）与 `/api/camera_rear.jpg`（后视）。

### 话题名冲突处理（最终 remap 方案）

- `/cmd_vel` 只有 avoidance_node 一个发布者；mux 的 `navigation_topic` 保持默认 `/cmd_vel`，无需 remap。
- mux 输出 `/ugv/control/cmd_vel` → gateway 用**参数** `input_topic:=/ugv/control/cmd_vel` 对接（不用 remap）。
- gateway 默认输出 `/ugv/sim/cmd_vel` 与 sim_motor_bridge 的 gz 下发话题同名，会短路掉
  chassis_controller；因此 gateway 用**参数** `output_topic:=/ugv/gateway/cmd_vel` 改名。
- chassis_controller 的 `/cmd_vel` 输入**remap** 到 `/ugv/gateway/cmd_vel`。
- `/ugv/sim/cmd_vel` 仅由 sim_motor_bridge 发布，经 ros_gz_bridge（ROS_TO_GZ）送入 gz。

## 构建（WSL2，Ubuntu + ROS 2 Humble + Gazebo Harmonic）

```bash
wsl -e bash -lc "cd /mnt/d/Codex/CAR/CAR_ws && source /opt/ros/humble/setup.bash && colcon build"
```

> **依赖说明**：`ros_gz_bridge` 不在 apt 源（Humble+Harmonic 需源码构建）。
> 本机已编译于 `UAV/simulation/air_ground_sim_ws`（src 下有 vendored `ros_gz`），
> 运行前需先 source 该工作区（见下）。换机器时把 `ros_gz` 源码复制进本工作区
> 一起 `colcon build` 即可。

## 仿真启动

```bash
source /opt/ros/humble/setup.bash
source /mnt/d/Codex/UAV/simulation/air_ground_sim_ws/install/setup.bash   # 提供 ros_gz_bridge
source /mnt/d/Codex/CAR/CAR_ws/install/setup.bash

# 带避障全链路（GUI）
ros2 launch car_sim car_sim.launch.py
# 无头
ros2 launch car_sim car_sim.launch.py headless:=true
# 纯遥控链路（不起 perception/avoidance）
ros2 launch car_sim teleop_test.launch.py
```

网页遥控：浏览器打开 <http://localhost:8765>（WSL 内运行时用 Windows 浏览器同样可访问）。
方向键按钮 / WASD / 方向键控制，按住行驶、松开停车；页面显示位姿、速度、控制权与前后视相机。
网页控制台为卡片式网格布局（遥控/状态/相机/雷达/GPS 地图/建图六卡片），另有雷达扫描图
（`/api/scan.json`）、GPS 面板 + 高德瓦片地图定位（WGS-84→GCJ-02
纠偏）、激光建图面板与一键建图按钮（slam_toolbox，实机用，详见 `real_bringup.launch.py` 链路）、
已保存建图结果预览（`/api/maps` 列表 + pgm→png）、固定地图自动巡航（地图上点选
初始点/目标点）、雷达图脱困路径显示。「拍照」按钮直接把当前帧下载到浏览器本地
（`/api/photo/download`）。

## Nav2 自主导航（已屏蔽）

Nav2 地图自主导航（AMCL 定位 + NavFn 规划 + RPP 跟踪 + costmap 实时避障）
**因实机性能不足已屏蔽**：Pi 4B 同时运行 GNOME 桌面/远程桌面/VS Code 时，
雷达 460800 波特串口大量丢包（/scan 掉到 3Hz），AMCL 无法收敛，导航不可用。
`real_bringup.launch.py` 已移除 `nav_mode` 参数，不再启动 Nav2 栈。

相关实现保留备用：`car_sim/config/nav2_params.yaml`、
`car_sim/launch/nav2_stack.launch.py`、web_gateway 的 `nav_backend` 后端。
如需恢复：关掉 Pi 的桌面环境（纯 SSH 运行）释放算力后，在 real_bringup
重新 include nav2_stack.launch.py 并恢复 nav_mode/map 参数即可（改动见
git 历史或该 launch 文件头部注释）。自主巡航/避障不受影响，仍由
avoidance_node 承担。

### 一键启停脚本

`scripts/sim.sh`（WSL 内）封装了仿真的一键启动/停止/状态查询；Windows 侧可双击
`scripts/sim_start.bat` / `scripts/sim_stop.bat`（start 支持 `headless` 参数无头运行）。
stop 会先杀 launch 进程组，再按本项目特征兜底清理 gz/桥/节点残留并校验：

```bash
bash /mnt/d/Codex/CAR/scripts/sim.sh start [headless]
bash /mnt/d/Codex/CAR/scripts/sim.sh status
bash /mnt/d/Codex/CAR/scripts/sim.sh stop
```

自主避障：默认 `enable_cruise:=false`，车原地待命。可直接点网页遥控页的
**「开启巡航」**按钮（一键开关，经 `/api/ugv/cruise` 动态设置避障节点参数），
或设定目标后由避障节点输出 `/cmd_vel`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
# 或命令行定速巡航：ros2 param set /avoidance_node enable_cruise true
```

遥控优先：网页按住方向键时 operator heartbeat 新鲜（0.6s 内），teleop 覆盖避障指令；松手
350ms 后网关看门狗自动停车，600ms 后 mux 把控制权交还避障。

巡航转向辅助：巡航开启时 web_gateway 自动把 mux 的 `steering_assist` 置 true（关巡航即复位）。
此时按住 A/D 只接管方向（angular 用遥控值），线速度保持巡航速度；若遥控给了非零线速度
（W/S 加速/刹车）则以遥控线速度为准。松手后恢复全自主避障。mux 状态中该模式
authority 为 `operator_steering`。

## 双向丝杆对接机构（ESP8266 下位机）

双 42 步进 + T8 双向丝杆（电机在丝杆中点双出轴，左正牙右反牙，导程 2mm，
单边行程 57mm），两个控制组。固件在 `tools/leadscrew42`（本仓
`esp8266-deploy` 分支），文本行协议（115200 8N1）编解码在
`car_nodes/esp_leadscrew_protocol.py`，节点 `leadscrew_driver_node`：

- 订阅 `/leadscrew/cmd`（LeadscrewCommand：`group` 0=两组/1/2，
  `command` 0=STOP 1=IN 2=OUT 3=RELAX 4=LOCK，`speed` steps/s 0=不变）
- 发布 `/leadscrew/status`（LeadscrewStatus，2Hz：两组状态/位置/使能）
- 状态机：`AT_OUTER →(IN) MOVING_IN → AT_INNER`，反向同理；运动中
  `STOP` 进入 `HOLD_MID`（原地自锁，可继续 IN/OUT 走完剩余行程）
- 启动：`ros2 launch car_sim real_bringup.launch.py leadscrew_port:=/dev/ttyUSB1`
  （默认关闭；`leadscrew_simulate:=true` 为本地仿真，不开串口）
- 仿真（Gazebo）：`car_sim.launch.py` 已挂载 `leadscrew_driver_node`
  （simulate + publish_sim_joints），推杆模型在
  `car_description/models/r680_4wd/model.sdf`（停机坪 450x450，电机挂 A/B 边
  中点下方，4 个 "[" 型推杆棱柱关节，组1 跨 A↔C @h1=30mm、组2 跨 B↔D
  @h2=55mm）。关节语义 q=0 外侧 / -0.057 内侧，与固件 pos 0~57mm 对应；
  关节指令经 ros_gz_bridge（`/leadscrew/sim/pusher_{a,b,c,d}/cmd_pos`），
  实际关节位置可 `ros2 topic echo /leadscrew/sim/joint_states` 查看。
- 硬件注意：ESP8266 上电假定螺母在外侧（pos=0），实际不在时先手动归位；
  打开串口瞬间 DTR/RTS 会触发 ESP 复位（节点内已显式释放）；
  12V 电机配 24V 驱动供电时电流档按电机额定电流设。

## 实机待办

- 实机一键全链路：`ros2 launch car_sim real_bringup.launch.py`（树莓派实测版，
  参数 `motor_port`/`lidar_port`/`gps_port`/`front_camera:=v4l2|k210|none` 等见文件
  docstring；避障已带实机调优值）。树莓派部署子集维护在本仓 `rpi-deploy` 分支
  （对应 `D:\CAR_deploy`）。
- ~~用真实 `motor_driver`（WHEELTEC 串口协议）替换 `sim_motor_bridge`~~ 已完成：
  协议编解码在 `car_nodes/wheeltec_protocol.py`（0x7B/0x7D 帧、BCC 异或校验），
  话题契约不变——订阅 `/ackermann_cmd`（AckermannCommand：后轮速[左后/右后] rad/s +
  前轮转向角 rad），发布 `/motor_feedback`（后轮速 + 转向角 + 电压），并附带发布
  板载 IMU `/imu/data`。下发帧只含车体 (vx, vz)，转向角→舵机、后轮差速由 STM32
  固件内部完成。
  实机以 `simulate:=false`、`port:=/dev/ttyACM0`（按实际串口）启动即可，
  chassis_controller 里程计自动切到真实反馈。协议细节见
  [docs/HARDWARE_RESERVED.md](docs/HARDWARE_RESERVED.md)。
- 待实机联调：确认串口设备名、用厂商串口助手核对首帧数据，再开电机使能。
  **实测修正 `wheelbase`（默认 0.31 m）与 `max_steering_angle`（默认 0.5 rad）**
  （chassis_controller / sim_motor_bridge / motor_driver 三处参数保持一致）；
  刷写 STM32 固件前先用蓝牙 APP 备份舵机零点参数（见 HARDWARE_RESERVED）。
- `lidar_driver`（镭神 N10P 串口版）替换 gz 桥接的 `/scan`：厂商 ROS2 SDK 已 vendored 在
  `CAR_ws/src/vendor/lslidar_ros2`，实机 `ros2 launch car_nodes n10p_lidar.launch.py`
  （详见 docs/HARDWARE_RESERVED.md），自带节点仅空载仿真用；
  `camera_driver`（CSI 摄像头）替换桥接的 `/camera/image_raw`——**或用 K210 作前摄**
  （固件 `scripts/k210_firmware.py` 烧录到 K210，USB 直连树莓派；
  `ros2 run car_nodes k210_camera_driver_node --ros-args -p port:=/dev/ttyUSB0`，
  发布契约与 camera_driver 相同）；后置 USB 摄像头以
  `camera_driver` 第二实例（`device:=/dev/video1`、`image_topic:=/camera/rear/image_raw` 等，
  见节点 docstring）替换桥接的 `/camera/rear/image_raw`。
- GPS（WHEELTEC G60，ATGM336H + CH9102F，9600 波特）发布 `/fix`（NavSatFix）：厂商 ROS2
  SDK 已 vendored 在 `CAR_ws/src/vendor/wheeltec_gps`，实机
  `ros2 launch car_nodes g60_gps.launch.py`（配置 `car_nodes/config/g60_gps.yaml`，
  串口默认 `/dev/wheeltec_gps`，详见 docs/HARDWARE_RESERVED.md）。GPS 上电即输出，
  无需启动命令。
