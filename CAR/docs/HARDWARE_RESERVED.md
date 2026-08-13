# 硬件预留清单（HARDWARE_RESERVED）

## 物料状态

| 物料 | 状态 | 备注 |
| --- | --- | --- |
| 树莓派 4B | 已备 | 主控，运行 ROS 2 节点 |
| WHEELTEC R680 底盘 + STM32 下位机 | 已备 | 4WD，轮径 152mm，轮距 0.32m |
| RPLIDAR C1 | 待购 | 对应 `lidar_driver` 节点（当前仅模拟数据） |
| CSI 摄像头 | 待购 | 对应 `camera_driver` 节点（当前仅渐变测试图） |
| 24V→5V 5A 降压模块 | 待购 | 树莓派供电 |
| 4G 模块 | 暂缓 | 远程链路，后期评估 |

## STM32 串口接入点

仿真中的 `sim_motor_bridge`（car_nodes 包）就是实机 `motor_driver` 的占位替换。
切到实机时话题契约保持不变：

- 下发：订阅 `/wheel_speeds`（car_interfaces/WheelSpeeds，float32[4]，单位 rad/s，
  顺序左前/右前/左后/右后），按 WHEELTEC 串口协议写入 STM32。
- 回读：发布 `/motor_feedback`（car_interfaces/MotorFeedback，float32[4] 实际轮速 rad/s
  同序 + float32 电压 V），10Hz 即可。
- 现有 `motor_driver.py` 中 `_send_command()` 的 `"V w0 w1 w2 w3\n"` 文本协议和
  `_read_feedback()` 的 `"F w0 w1 w2 w3 voltage\n"` 解析均为占位实现，需替换为
  WHEELTEC 实际帧格式（帧头/校验/二进制）。协议文档待从网盘下载后补充到此文件。
- launch 切换点：实机 bringup 中用 `motor_driver_node`（`simulate:=false`）替换
  `sim_motor_bridge_node`，其余链路（mux/gateway/chassis_controller/avoidance）不动。

## 传感器接入点

- `/scan`（sensor_msgs/LaserScan，frame_id `laser_frame`）：实机由 `lidar_driver`
  （RPLIDAR C1 串口）发布；仿真由 ros_gz_bridge 桥接 gz gpu_lidar。
- `/camera/image_raw` + `/camera/camera_info`（frame_id `camera_optical_frame`）：实机由
  `camera_driver`（CSI，`simulate:=false`）发布；仿真桥接 gz 相机（frame_id `camera_link`）。
- `/imu/data`：仿真已桥接备用；实机 IMU 来源待定（STM32 板载或外置）。
