# 硬件预留清单（HARDWARE_RESERVED）

## 物料状态

| 物料 | 状态 | 备注 |
| --- | --- | --- |
| 树莓派 4B | 已备 | 主控，运行 ROS 2 节点 |
| WHEELTEC R680 底盘 + STM32 下位机 | 已备 | **阿克曼转向版**（卖家发错货，项目已适配）：前轮舵机转向 + 后轮双编码器电机，一块双路电机驱动板；轮径 152mm，轮距 0.32m，轴距约 0.31m（待实测） |
| 镭神 N10P 激光雷达 + 串口转接模块 | 已备 | **双回波雷达**（默认单回波，建图导航建议保持单回波）。厂商 ROS2 SDK 已收进本工作区 `src/vendor/lslidar_ros2`（`lslidar_driver` + `lslidar_msgs` + `wheeltec_udev.sh`）；启动入口 `car_nodes/launch/n10p_lidar.launch.py` + `car_nodes/config/lslidar_n10p_uart.yaml`；自带 `lidar_driver` 节点仅用于空载仿真 |
| WHEELTEC G60 GPS 模块 | 已备 | ATGM336H + **CH9102F** USB 转串口（出厂串口序列号 **0005**），波特率 **9600**，上电即输出 NMEA（GN 系语句）。厂商 ROS2 SDK 已收进本工作区 `src/vendor/wheeltec_gps`（`nmea_msgs` + `nmea_navsat_driver` + `wheeltec_gps_path` + `wheeltec_udev.sh`）；启动入口 `car_nodes/launch/g60_gps.launch.py` + `car_nodes/config/g60_gps.yaml`，real_bringup 默认随车启动（`gps_port` 参数） |
| CSI 摄像头 | 待购 | 对应 `camera_driver` 节点（当前仅渐变测试图） |
| 免驱 USB 摄像头（前摄） | 待购 | **实机前视摄像头**（方案已从 K210 变更为免驱 UVC 摄像头）：插上即识别为 `/dev/video0`（V4L2，无需驱动），由 `camera_driver`（`simulate:=false`）发布 `/camera/image_raw`，real_bringup 默认即此路径（`camera_device` 参数） |
| USB 后置摄像头 | 待购 | Pi 4B 只有一个 CSI 口，后摄走 USB（如 `/dev/video1`）；以 `camera_driver` 第二实例发布 `/camera/rear/image_raw`（real_bringup 加 `rear_camera_device:=/dev/video1` 启动） |
| 24V→5V 5A 降压模块 | 待购 | 树莓派供电 |
| 4G 模块 | 暂缓 | 远程链路，后期评估 |

## 阿克曼底盘注意事项

- **不能横移、不能原地自旋**（厂商开发手册底盘特性表）：上层规划/遥控只允许
  (vx, vz) 且 vz 需伴随 vx 才有效；避障节点已改为带速弧线掉头（`turn_speed` 参数）。
- **舵机零点出厂已标定，每辆车不同**：如需刷写 STM32 固件，先按
  《顶配阿克曼更新固件必读说明》用蓝牙 APP（BT04-A，密码 1234）「获取设备参数」
  备份倒数第 2/3/4 行三个舵机零点数据，刷完再写回并掉电保存。
- 阿克曼车型接线：电机 A/B 接双路驱动板电机 1/2 接口，转向结构分舵机开环/闭环
  两种（见《阿克曼底盘的接线说明和 OLED 显示说明》）；舵机中值可用控制板电位器微调。
- 测试固件（架空跑轮子）：`（架空小车再烧录）Akm阿克曼车测试代码.hex`。

## STM32 串口接入点

仿真中的 `sim_motor_bridge`（car_nodes 包）就是实机 `motor_driver` 的占位替换。
切到实机时话题契约保持不变：

- 下发：订阅 `/ackermann_cmd`（car_interfaces/AckermannCommand：float32[2] 后轮速
  rad/s，左后/右后 + float32 前轮转向角 rad），换算成车体 (vx, vz) 后按
  WHEELTEC 串口协议写入 STM32（转向角→舵机由固件内部完成；v≈0 时 vz 强制为 0）。
- 回读：发布 `/motor_feedback`（car_interfaces/MotorFeedback：float32[2] 实际后轮速
  rad/s 同序 + float32 转向角 rad + float32 电压 V），10Hz 即可。
- `motor_driver.py` 已实现 WHEELTEC 二进制协议（实机时 `simulate:=false`），编解码
  在 `car_nodes/wheeltec_protocol.py`（纯函数，可单测）。STM32 侧按车体三轴速度收发，
  节点内部做 阿克曼指令 ↔ (vx, vz) 换算（与 sim_motor_bridge 同一套运动学函数）。
- launch 切换点：实机 bringup 中用 `motor_driver_node`（`simulate:=false`）替换
  `sim_motor_bridge_node`，其余链路（mux/gateway/chassis_controller/avoidance）不动。

### WHEELTEC 串口协议（已实现）

来源：厂商资料《串口通信控制与反馈_2026-8-12.pdf》（教育机器人与大型科研机器人同一协议）。
接线：控制板**串口 3**（USB 转 TTL，CH9102/CH2102 芯片），波特率 **115200**；
Linux 上设备一般为 `/dev/ttyACM*`（节点默认 `/dev/ttyACM0`，参数 `port` 可改）。
注意不要用串口 1 通信（上电时数据帧会被当成烧录包导致卡死）。

- 下行（上位机→STM32，11 字节）：`0x7B | 00 00 | vx(int16 大端, mm/s) |
  vy(int16, mm/s, 仅全向车型有效，阿克曼填 0) | vz(int16, rad/s×1000) | BCC(前9字节异或) | 0x7D`
- 上行（STM32→上位机，24 字节）：`0x7B | flag_stop(0=电机使能) | vx mm/s | vy mm/s |
  vz rad/s×1000 | 三轴加速度原始值(÷1672→m/s²) | 三轴陀螺仪原始值(÷3753→rad/s) |
  电压 mV | BCC(前22字节异或) | 0x7D`（除注明外均为 int16 大端）
- Z 轴正值=逆时针，与 ROS REP-103 一致，无需换号（厂商文档 3.1 节示例：负值=顺时针）。
- 上行帧自带板载 IMU 原始数据，`motor_driver` 在 `publish_imu:=true`（默认）时
  同步发布 `/imu/data`（sensor_msgs/Imu，无姿态角，`orientation_covariance[0]=-1`）。
- 实机联调前建议关闭电机使能开关（大车 SW1），通过 OLED 确认目标速度后再使能。

## 传感器接入点

- `/scan`（sensor_msgs/LaserScan，frame_id `laser_frame`）：实机为**镭神 N10P 串口版**，
  厂商 ROS2 SDK 已 vendored 在 `src/vendor/lslidar_ros2`（**不要**用 GitHub 的
  `Lslidar_ROS2_driver` N10_V1.0 分支，那是 N10 的，N10P 不适用）。配置
  `src/car_nodes/config/lslidar_n10p_uart.yaml`：`frame_id: laser_frame`、
  `laserscan_topic: /scan`、串口默认 `/dev/wheeltec_lidar`（udev 规则见
  `config/99-car-devices.rules` 或 vendor 里的 `wheeltec_udev.sh`）。
  - 电机上电即转，驱动内 `motor_running` 默认 true，**无需先发启动命令**；如需停转/恢复：
    `ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "{data: 0}"`（1=转，0=停）。
  - N10P 双回波：保持 `publish_multiecholaserscan: false`（单回波；双回波噪点多、强度低，
    厂商建议常规建图导航用单回波）。扫描频率 `N10Plus_hz`（6–12，默认 10）；
    `use_high_precision` 建议开；量程 `min_range 0.15` / `max_range 50.0`。
  - 构建依赖：`sudo apt install libpcl-dev ros-humble-pcl-conversions libpcap-dev`。
  自带 `lidar_driver` 节点仅作空载仿真占位，不用于实机（协议解析未实现）。
- `/fix`（sensor_msgs/NavSatFix，frame_id `gps`）：实机为 **WHEELTEC G60**（ATGM336H），
  厂商 ROS2 SDK 已 vendored 在 `src/vendor/wheeltec_gps`。real_bringup 默认随车启动
  （`gps_port:=/dev/wheeltec_gps`，留空关闭）；单独启动：
  `ros2 launch car_nodes g60_gps.launch.py`（配置 `src/car_nodes/config/g60_gps.yaml`：
  `baud: 9600`、`useRMC: false`）。
  - **必须用 vendored 的 wheeltec 修改版 nmea_navsat_driver，不能 apt 装上游**：
    G60 输出 `$GNxxx` 语句，上游旧版解析器只吃 `$GP`（vendored 版支持 GP/GN/GL/IN）。
  - udev：模块为 **CH9102F**（`1a86:55d4`），出厂串口序列号 **0005**，规则见
    `config/99-car-devices.rules`（**坑**：厂商 zip 自带 udev 脚本只有 CP2102 规则，
    与实物不符，vendored 的 `wheeltec_udev.sh` 已补 CH9102 规则）。
    ⚠ GPS 与 STM32 控制板同为 1a86:55d4，控制板规则需补 serial 过滤（见规则文件注释）。
  - GPS 上电即输出 NMEA，无需启动命令；依赖 `ros-humble-tf-transformations`
    （`setup_pi.sh` 已含）。
  - NavSatFix 是全球坐标不依赖 TF；URDF 无 gps_link，后续定位融合时再补。
  - `wheeltec_gps_path`（/fix → rviz 轨迹）为可选可视化工具，常规运行不需要。
- `/camera/image_raw` + `/camera/camera_info`（frame_id `camera_optical_frame`）：实机前摄为
  **免驱 USB 摄像头**（UVC/V4L2，插上即 `/dev/video*`，无需驱动），由 `camera_driver`
  （`simulate:=false`，real_bringup 参数 `camera_device`，默认 `/dev/video0`）发布。
- `/camera/rear/image_raw` + `/camera/rear/camera_info`（frame_id `rear_camera_optical_frame`）：
  后置相机，USB 摄像头以 `camera_driver` 第二实例发布（real_bringup 参数
  `rear_camera_device`）。遥控页前后双画面（`/api/camera.jpg` / `/api/camera_rear.jpg`）。
- `/imu/data`：仿真已桥接备用；实机由 `motor_driver` 从上行帧中的 STM32 板载 IMU
  原始数据发布（`publish_imu` 参数控制，默认开）。
