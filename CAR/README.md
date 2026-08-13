# CAR —— R680 4WD 无人车（ROS 2 + Gazebo 仿真）

本目录为 UAV-CAR 空地协同项目的地面车部分，基于 ROS 2（Humble）与 Gazebo classic 11，
提供 R680 四驱小车的模型描述、一键仿真启动与键盘遥控。

## 目录结构

```
CAR/
└── CAR_ws/                      # ROS 2 colcon 工作区
    └── src/
        ├── car_description/     # （src/description）小车描述包
        │   ├── urdf/
        │   │   ├── CAR_description.urdf   # 原始 URDF：底盘、四轮、相机、雷达
        │   │   └── car_gazebo.xacro       # Gazebo 插件封装（驱动/相机/雷达）
        │   ├── launch/view_car.launch.py  # RViz 模型查看
        │   └── rviz/view_car.rviz
        ├── CAR_pkg/             # 仿真启动包
        │   ├── launch/gazebo_sim.launch.py# 一键启动 Gazebo 仿真
        │   └── worlds/car_test.world      # 10x10 m 围栏测试场地
        └── car_control/         # 控制工具包
            └── car_control/teleop_keyboard.py  # 键盘遥控节点
```

## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo classic 11 与 ROS 接口：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui
```

## 编译

```bash
cd CAR/CAR_ws
colcon build --symlink-install
source install/setup.bash
```

## 使用

### 1. 一键启动 Gazebo 仿真

```bash
ros2 launch CAR_pkg gazebo_sim.launch.py
```

可选参数：

```bash
# 指定出生位置与朝向（米 / 弧度）
ros2 launch CAR_pkg gazebo_sim.launch.py x:=1.0 y:=-0.5 yaw:=1.57

# 使用自定义世界
ros2 launch CAR_pkg gazebo_sim.launch.py world:=/absolute/path/to.world
```

### 2. 键盘遥控（另开一个终端）

```bash
source CAR/CAR_ws/install/setup.bash
ros2 run car_control teleop_keyboard
```

| 按键 | 功能 |
| ---- | ---- |
| w / s | 前进 / 后退 |
| a / d | 左转 / 右转 |
| 空格 | 立即停车 |
| q / z | 线速度档位 +10% / -10% |
| e / c | 角速度档位 +10% / -10% |
| Ctrl-C | 退出 |

按键松开超过 0.5 s 自动停车，防止终端失焦后小车失控。

### 3. 仅查看模型（不开 Gazebo）

```bash
ros2 launch car_description view_car.launch.py
```

## 话题一览

仿真中小车话题统一在 `/car` 命名空间下：

| 话题 | 类型 | 说明 |
| ---- | ---- | ---- |
| `/car/cmd_vel` | geometry_msgs/Twist | 速度指令（订阅） |
| `/car/odom` | nav_msgs/Odometry | 里程计（发布） |
| `/car/scan` | sensor_msgs/LaserScan | 顶部 360° 激光雷达 |
| `/car/front_camera/image_raw` | sensor_msgs/Image | 车头相机图像 |
| `/car/front_camera/camera_info` | sensor_msgs/CameraInfo | 相机内参 |

TF 树：`odom -> base_link`（驱动插件广播），车轮 TF 由驱动插件广播，
其余固定关节（相机 `camera_optical_frame`、雷达 `laser_frame`）由 robot_state_publisher 发布。

## 已知参数与待办

- 整车 465 x 385 x 218 mm，轮径 152 mm，4WD 滑移转向；估算参数见 URDF 头部注释。
- 许可证暂按 MIT 填写，可按需调整。
- 后续可扩展：SLAM（slam_toolbox）、Nav2 导航、与 UAV 端的降落对接协同（参见 `UAV/docking_design`）。
