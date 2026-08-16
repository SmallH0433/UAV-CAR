# CAR_ws — R680 阿克曼小车实机部署版（树莓派 4B + Ubuntu）

从主仓 `D:/Codex/CAR` 摘出的实机运行子集，去掉了所有 Gazebo 仿真设施
（gz 世界、ros_gz_bridge、仿真 launch、URDF 模型）。本目录内容对应
树莓派上的 `~/CAR_ws` 工作区。

对应主仓版本：提交 `a45a45f`（release 4.1，K210 前视摄像头适配；含 4.0 的 WHEELTEC G60 GPS 适配、3.0 的后置双摄像头 + 巡航转向辅助 + N10P 雷达适配）。

## 目录结构

- `src/car_interfaces` — 自定义 msg/srv
- `src/car_nodes` — 实机全部节点：motor_driver（WHEELTEC 串口协议）、
  chassis_controller（自行车模型）、perception、avoidance（自主绕行）、
  lidar_driver（仅空载仿真用；实机雷达为镭神 N10P，用厂商驱动 lslidar_driver）、
  camera_driver（V4L2，支持前摄 + USB 后摄双实例）、
  k210_camera_driver（K210 串口推流前摄）、uav_bridge、
  sim_motor_bridge（承载运动学函数 + 空载测试用）；
  `launch/n10p_lidar.launch.py` + `config/lslidar_n10p_uart.yaml` 为雷达单独启动入口；
  `launch/g60_gps.launch.py` + `config/g60_gps.yaml` 为 GPS 单独启动入口
- `src/car_sim` — 运行基础设施（包名沿用主仓）：控制权 mux（含巡航转向辅助
  steering_assist）、指令网关、网页遥控（前后双画面）、`real_bringup.launch.py`
- `src/vendor/lslidar_ros2` — 镭神 N10P 厂商 ROS2 SDK（lslidar_driver + lslidar_msgs +
  wheeltec_udev.sh），随工作区一起 colcon build，无需再从 GitHub 拉取
- `src/vendor/wheeltec_gps` — WHEELTEC G60 GPS 厂商 ROS2 SDK（nmea_msgs +
  nmea_navsat_driver（wheeltec 修改版，支持 $GN talker）+ wheeltec_gps_path +
  wheeltec_udev.sh），同样随工作区一起 colcon build
- `config/99-car-devices.rules` — udev 固定设备名（/dev/wheeltec、/dev/wheeltec_lidar）
- `scripts/setup_pi.sh` — 树莓派一键环境配置
- `scripts/sync_to_pi.sh` — PC（WSL）侧 rsync 同步到树莓派
- `scripts/k210_firmware.py` — K210 前摄固件（MaixPy IDE 烧录为 K210 的 main.py）
- `docs/HARDWARE_RESERVED.md` — 硬件接线/串口协议/舵机零点备份注意事项

## 部署流程

### 1. 树莓派系统

Ubuntu Server 22.04 arm64（ROS 2 Humble 有官方 arm64 apt 源，
**切勿在 Pi 上源码编译 ROS**）。

### 2. 拷贝到树莓派

方式 A（PC 的 WSL 里一键同步，推荐）：

```bash
cd /mnt/d/CAR_deploy
bash scripts/sync_to_pi.sh <树莓派IP> <用户名>
```

方式 B（U盘/scp 手动拷贝）：把整个目录拷到树莓派 `~/CAR_ws`。

### 3. 树莓派上配置环境 + 编译

```bash
cd ~/CAR_ws
bash scripts/setup_pi.sh     # 装 ROS/依赖、配 dialout 组、装 udev 规则
# 注销重新登录（dialout 生效）后：
source /opt/ros/humble/setup.bash
colcon build                 # Pi 4B 约 2~5 分钟
echo "source ~/CAR_ws/install/setup.bash" >> ~/.bashrc
```

## 运行

```bash
# 实机全链路（装了 udev 规则可用 /dev/wheeltec /dev/wheeltec_lidar）
ros2 launch car_sim real_bringup.launch.py \
  motor_port:=/dev/wheeltec lidar_port:=/dev/wheeltec_lidar

# 接了 USB 后置摄像头时（遥控页显示后视画面）
ros2 launch car_sim real_bringup.launch.py rear_camera_device:=/dev/video1

# 用 K210 作前摄（先烧录 scripts/k210_firmware.py 为 K210 的 main.py，USB 直连）
ros2 launch car_sim real_bringup.launch.py front_camera:=k210 k210_port:=/dev/ttyUSB0

# 空载链路自检（不接任何硬件，模拟雷达/电机/相机）
ros2 launch car_sim real_bringup.launch.py \
  lidar_mode:=sim motor_simulate:=true camera_simulate:=true
```

雷达说明：实机雷达为**镭神 N10P 串口版**（双回波，默认单回波），厂商 ROS2 SDK
已 vendored 在 `src/vendor/lslidar_ros2`，随工作区一起编译即可（驱动编译期依赖
PCL/pcap，`setup_pi.sh` 已含）：

```bash
sudo apt install libpcl-dev ros-humble-pcl-conversions libpcap-dev
cd ~/CAR_ws && colcon build && source install/setup.bash
```

launch 用 `src/car_nodes/config/lslidar_n10p_uart.yaml`（话题 `/scan`、
frame_id `laser_frame`、串口默认 `/dev/wheeltec_lidar`、10Hz、高精度、单回波）。
N10P 电机上电即转，**无需开工令**；停转/恢复用
`ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "{data: 0}"`（1 转 0 停）。
自带 `lidar_driver` 节点仅用于 `lidar_mode:=sim` 空载自检。

GPS 说明：实机 GPS 为 **WHEELTEC G60**（ATGM336H + CH9102F，9600 波特），厂商 ROS2
SDK 已 vendored 在 `src/vendor/wheeltec_gps`。real_bringup **默认随车启动**
（`gps_port:=/dev/wheeltec_gps`，`gps_port:=''` 关闭），定位输出
`ros2 topic echo /fix`（NavSatFix）查看。GPS 上电即输出，无需启动命令。
⚠ GPS 与 STM32 控制板同为 CH9102（1a86:55d4），udev 规则已用序列号 0005 区分
GPS，但控制板规则需补实际 serial 过滤（见 `config/99-car-devices.rules` 注释）。

网页控制台：同一局域网的电脑/手机浏览器打开 `http://<树莓派IP>:8765`
（默认监听 0.0.0.0；WASD 组合按键弧线遥控，遥控优先于避障；前视画面，
接了后摄时前后双画面；巡航开启后可用 A/D 手动微调方向，即转向辅助）。

## 实机联调顺序（重要）

1. **先备份舵机零点**（蓝牙 APP「获取设备参数」倒数 2/3/4 行，见 docs），再动固件。
2. 电机使能开关保持关闭，先跑空载自检确认链路通畅。
3. 接 STM32 串口 3（**不要用串口 1**，上电帧会被当烧录包卡死），
   `simulate:=false` 启动后用 `ros2 topic echo /motor_feedback` 核对电压/轮速。
4. 架空小车使能电机，网页低速遥控验证转向方向与轮速方向。
5. **实测修正参数**：`wheelbase`（默认 0.31 m）、`max_steering_angle`
   （默认 0.5 rad）——chassis_controller / motor_driver 两处保持一致。
6. 落地测试避障绕行；需要巡航时网页点「开启巡航」或 `enable_cruise:=true`。

## 与主仓的关系

本目录是发布快照，日常开发在主仓 `D:/Codex/CAR`（含仿真）进行；
实机节点有更新时重新从主仓拷贝对应包，用 `sync_to_pi.sh` 增量同步到树莓派。
