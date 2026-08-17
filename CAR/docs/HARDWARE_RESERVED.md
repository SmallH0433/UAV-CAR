# 硬件预留清单（HARDWARE_RESERVED）

## 物料状态

| 物料 | 状态 | 备注 |
| --- | --- | --- |
| 树莓派 4B | 已备 | 主控，运行 ROS 2 节点 |
| WHEELTEC R680 底盘 + STM32 下位机 | 已备 | **阿克曼转向版**（卖家发错货，项目已适配）：前轮舵机转向 + 后轮双编码器电机，一块双路电机驱动板；轮径 152mm，轮距 0.32m，轴距约 0.31m（待实测） |
| 镭神 N10P 激光雷达 + 串口转接模块 | 已备 | **双回波雷达**（默认单回波，建图导航建议保持单回波）。厂商 ROS2 SDK 已收进项目 `CAR_ws/src/vendor/lslidar_ros2`（`lslidar_driver` + `lslidar_msgs` + `wheeltec_udev.sh`，源自 `D:\资料\N10系列激光雷达附送资料\...\2.ROS2_SDK`）；项目侧启动入口 `car_nodes/launch/n10p_lidar.launch.py` + `car_nodes/config/lslidar_n10p_uart.yaml`；自带 `lidar_driver` 节点仅用于空载仿真；本机实测串口转接为 CH9102（1a86:55d4，serial `5B8E677903`） |
| WHEELTEC G60 GPS 模块 | 已备 | ATGM336H + **CH9102F** USB 转串口（1a86:55d4，本机实测串口序列号 **5B0B031719**，非出厂标称 0005，须按实机核对），波特率 **9600**，上电即输出 NMEA（GN 系语句）。厂商 ROS2 SDK 已收进项目 `CAR_ws/src/vendor/wheeltec_gps`（`nmea_msgs` + `nmea_navsat_driver` + `wheeltec_gps_path` + `wheeltec_udev.sh`，源自 `D:\资料\WHEELTEC G60模块附送资料\...\2.Linux解析例程`）；项目侧启动入口 `car_nodes/launch/g60_gps.launch.py` + `car_nodes/config/g60_gps.yaml` |
| CSI 摄像头 | 待购 | 对应 `camera_driver` 节点（当前仅渐变测试图） |
| K210 开发板（MaixPy） | 已备 | **实机前视摄像头**：烧录 `scripts/k210_firmware.py`（QVGA JPEG 串口推流 + LCD 本地预览，921600 波特），USB 线直连树莓派（同时供电，识别为 /dev/ttyUSB*，本机实测 CH9102 serial `558B018651`），Pi 侧 `k210_camera_driver_node` 发布 `/camera/image_raw`，发布契约与 `camera_driver` 相同可直接替换。实测两坑：① CanMV makerobo 固件 REPL 不占 machine.UART1、UART1 上电无引脚映射，固件已补 `fm.register(5, fm.fpioa.UART1_TX)`（IO5 为 USB 串口 K210→PC 方向）；② CH9102 的 DTR/RTS 接 K210 复位/BOOT，驱动打开串口后必须清除 DTR/RTS，否则 K210 被按在复位态收不到流（`k210_camera_driver` 已处理） |
| USB 后置摄像头 | 待购 | Pi 4B 只有一个 CSI 口，后摄走 USB（如 `/dev/video1`）；以 `camera_driver` 第二实例发布 `/camera/rear/image_raw`（参数 `device`/`image_topic`/`info_topic`/`frame_id`，见节点 docstring） |
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
- 实机全链路 bringup：`car_sim/launch/real_bringup.launch.py`（树莓派实测版；
  `motor_port`/`lidar_port`/`gps_port`/`front_camera:=v4l2|k210|none`/`k210_port`/
  `rear_camera_device` 等参数见文件 docstring），其中用 `motor_driver_node`
  （`simulate:=false`）替换 `sim_motor_bridge_node`，其余链路
  （mux/gateway/chassis_controller/avoidance）不动。避障实机调优值（rpi-4.3）：
  `safety_distance` 1.0 / `slow_down_distance` 1.8 / `creep_speed` 0.25。

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
  厂商 ROS2 SDK 已 vendored 在 `CAR_ws/src/vendor/lslidar_ros2`（**不要**用 GitHub 的
  `Lslidar_ROS2_driver` N10_V1.0 分支，那是 N10 的，N10P 不适用）。一键启动：
  `ros2 launch car_nodes n10p_lidar.launch.py`（配置 `car_nodes/config/lslidar_n10p_uart.yaml`：
  `frame_id: laser_frame`、`laserscan_topic: /scan`、串口默认 `/dev/wheeltec_lidar`——
  用 vendor 目录里的 `wheeltec_udev.sh` 建 udev 规则（本机实测转接模块为 CH9102
  1a86:55d4 serial `5B8E677903`，按 serial 建规则），或改为实际 `/dev/ttyUSB*`/`/dev/ttyACM*`）。
  - 电机上电即转，驱动内 `motor_running` 默认 true，**无需先发启动命令**；如需停转/恢复：
    `ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "{data: 0}"`（1=转，0=停）。
  - N10P 双回波：保持 `publish_multiecholaserscan: false`（单回波；双回波噪点多、强度低，
    厂商建议常规建图导航用单回波）。注意即使单回波模式，N10P 原始点序在同方向相邻点间
    仍有远近回波跳变，perception 聚类已改为按角度分 bin（360×1° 取最近回波，
    `body_filter_distance` 0.25m 车身自遮罩 + `min_cluster_points` 噪点过滤，
    rpi-4.3 修复——此前按原始点序聚类会把障碍打成单点碎片导致避障收不到真实障碍）。
    扫描频率 `N10Plus_hz`（6–12，默认 10）；
    `use_high_precision` 建议开（厂商注释同系 N10 建议开启）；量程 `min_range 0.15` /
    `max_range 50.0`（仿真模型为 0.12/25，以实机为准）。
  - 构建依赖：`sudo apt install libpcl-dev ros-humble-pcl-conversions libpcap-dev`
    （驱动里 PCL 仅用于可选点云预处理，pcap 仅网口/离线回放用到，但编译期都需要）。
  自带 `lidar_driver` 节点仅作空载仿真占位，不用于实机（协议解析未实现）；
  仿真由 ros_gz_bridge 桥接 gz gpu_lidar。
- `/fix`（sensor_msgs/NavSatFix，frame_id `gps`）：实机为 **WHEELTEC G60**（ATGM336H），
  厂商 ROS2 SDK 已 vendored 在 `CAR_ws/src/vendor/wheeltec_gps`。一键启动：
  `ros2 launch car_nodes g60_gps.launch.py`（配置 `car_nodes/config/g60_gps.yaml`：
  `port: /dev/wheeltec_gps`、`baud: 9600`、`frame_id: gps`、`useRMC: false`）。
  - **必须用 vendored 的 wheeltec 修改版 nmea_navsat_driver，不能 apt 装上游**：
    G60 输出 `$GNxxx` 语句，上游旧版解析器只吃 `$GP`（vendored 版支持 GP/GN/GL/IN）。
  - udev：模块为 **CH9102F**（`1a86:55d4`），本机实测串口序列号 **5B0B031719**（非出厂
    标称 0005，用 `udevadm info -a -n /dev/ttyACM* | grep serial` 按实机核对）；用 vendor
    目录里的 `wheeltec_udev.sh` 建规则得 `/dev/wheeltec_gps`（**坑**：zip 自带脚本只写了
    CP2102 规则，与实物 CH9102F 不符，vendored 版已补上 CH9102 两条规则）。
    ⚠ 控制板 / GPS / 雷达 / K210 四个设备同为 **1a86:55d4**，udev 规则必须全部按
    serial 区分（本机实测：控制板 `0002`、GPS `5B0B031719`、雷达 `5B8E677903`、
    K210 `558B018651`）；控制板规则若无 serial 过滤，插 GPS 时 `/dev/wheeltec`
    可能指向 GPS。
  - GPS 上电即输出 NMEA，无需启动命令；依赖 `ros-humble-tf-transformations`
    （手册 FAQ 同款报错 `No module named 'tf_transformations'` 即缺此包）。
  - NavSatFix 是全球坐标不依赖 TF；URDF 暂无 gps_link（天线安装位置未实测），
    后续定位融合（robot_localization）时再补。仿真无 GPS 传感器。
  - `wheeltec_gps_path`（/fix → rviz 轨迹）为可选可视化工具，常规运行不需要。
- `/camera/image_raw` + `/camera/camera_info`（frame_id `camera_optical_frame`）：实机前摄
  二选一——`camera_driver`（CSI/USB，`simulate:=false`），或 **K210**（MaixPy 固件
  `scripts/k210_firmware.py`：QVGA JPEG 串口推流，921600 波特，帧按 FFD8/FFD9 标记切分；
  Pi 侧 `ros2 run car_nodes k210_camera_driver_node --ros-args -p port:=/dev/ttyUSB0`，
  图像尺寸/编码 rgb8 与 camera_driver 一致，web 画面/感知链路无需改动）。
  仿真桥接 gz 相机（frame_id `camera_link`）。
- `/camera/rear/image_raw` + `/camera/rear/camera_info`（frame_id `rear_camera_optical_frame`）：
  后置相机，实机为 USB 摄像头以 `camera_driver` 第二实例发布；仿真桥接 gz
  `rear_camera`（frame_id `rear_camera_link`）。遥控页前后双画面
  （`/api/camera.jpg` / `/api/camera_rear.jpg`）。
- `/imu/data`：仿真已桥接备用；实机由 `motor_driver` 从上行帧中的 STM32 板载 IMU
  原始数据发布（`publish_imu` 参数控制，默认开）。
