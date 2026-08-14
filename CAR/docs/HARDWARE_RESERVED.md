# 硬件预留清单（HARDWARE_RESERVED）

## 物料状态

| 物料 | 状态 | 备注 |
| --- | --- | --- |
| 树莓派 4B | 已备 | 主控，运行 ROS 2 节点 |
| WHEELTEC R680 底盘 + STM32 下位机 | 已备 | **阿克曼转向版**（卖家发错货，项目已适配）：前轮舵机转向 + 后轮双编码器电机，一块双路电机驱动板；轮径 152mm，轮距 0.32m，轴距约 0.31m（待实测） |
| RPLIDAR C1 | 待购 | 对应 `lidar_driver` 节点（当前仅模拟数据） |
| CSI 摄像头 | 待购 | 对应 `camera_driver` 节点（当前仅渐变测试图） |
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

- `/scan`（sensor_msgs/LaserScan，frame_id `laser_frame`）：实机由 `lidar_driver`
  （RPLIDAR C1 串口）发布；仿真由 ros_gz_bridge 桥接 gz gpu_lidar。
- `/camera/image_raw` + `/camera/camera_info`（frame_id `camera_optical_frame`）：实机由
  `camera_driver`（CSI，`simulate:=false`）发布；仿真桥接 gz 相机（frame_id `camera_link`）。
- `/imu/data`：仿真已桥接备用；实机由 `motor_driver` 从上行帧中的 STM32 板载 IMU
  原始数据发布（`publish_imu` 参数控制，默认开）。
