# UAV / AprilTag 项目迁移包

生成日期：2026-08-11  
原项目路径：`D:\Codex\UAV`  
建议恢复路径：`D:\Codex\UAV`

## 项目目标

本项目使用 ArduPilot SITL、Gazebo Harmonic、ROS 2 Humble 和 Mission Planner，
并将 Raspberry Pi 4B、IMX296、Pixhawk、MTF-01P、NEO-M9N 与移动小车结合，
实现无人机识别并跟随小车顶部 `tag36h11 / ID 0` AprilTag 的实验。

## 新 Codex 首先阅读

1. `WINDOWS_SETUP_STATUS.md`
2. `docs/APRILTAG_MOVING_FOLLOW_DESIGN_20260806.md`
3. `docs/PI_PIXHAWK_BENCH_HANDOFF_20260806.md`
4. `simulation/air_ground_sim_ws/src/air_ground_sim/README.md`
5. `imx296_debug/README.md`

## 目录说明

- `docs/`：总体方案、当前阶段和安全边界。
- `config/`：AprilTag、光流、飞控、电池和RC授权配置。
- `imx296_debug/`：IMX296采集、标定、AprilTag识别和跟随控制代码。
- `esp32_motor_uart_test/`：小车电机驱动板串口测试代码。
- `simulation/air_ground_sim_ws/src/air_ground_sim/`：ROS 2、Gazebo、SITL、小车和网页操作台源码。
- `docking_design/`：空地协同与对接结构设计资料。
- `evidence_summary/`：精选测试摘要，不包含大型原始日志。
- `printables/`：200 mm AprilTag打印文件。
- `tools/`：飞行日志复盘、标定和部署辅助工具。
- 根目录脚本：Pixhawk、Pi、SITL和MAVLink调试工具。

## 实机部署分支

- `uav-rpi-deploy` / `uav-rpi-7.6`：无人机机载 Raspberry Pi 4B 的最小部署包。
- `uav-pixhawk-deploy` / `uav-pixhawk-7.6`：QAV280 Pixhawk1 的固件和参数恢复包。

`main` 保存完整的无人机工程源码、配置与分析工具；部署分支只保留对应设备运行或恢复所需的内容。
无人车工程独立保存在 `CAR/`，不属于上述两个无人机部署包。

## 关联项目

- `../CAR/`：R680 4WD 小车独立项目（树莓派4B + STM32）。其 `car_sim` 仿真包
  （Gazebo 世界/桥接/控制权 mux/指令网关/网页遥控）移植自本项目的
  `simulation/air_ground_sim_ws/src/air_ground_sim`（仅 UGV 部分，不含 Nav2 与无人机）。
  CAR 运行仿真时需 source 本工作区的 install（提供 `ros_gz_bridge`）。

## 未包含的内容

为保证可以稳定压缩、传输和解压，本包没有包含：

- Python虚拟环境；
- `node_modules`；
- ROS 2的`build/`、`install/`和`log/`；
- Python缓存和测试缓存；
- Git元数据；
- 大型第三方源码；
- 大型原始图像、TLOG和JSONL测试日志；
- SSH私钥、密码、Codex账号配置和网络令牌。

第三方源码的精确版本见`THIRD_PARTY_REVISIONS.md`，在新电脑重新克隆并编译。

## 推荐环境

- Windows + WSL2；
- Ubuntu 22.04；
- ROS 2 Humble；
- Gazebo Harmonic；
- ArduPilot SITL与`ardupilot_gazebo`；
- Python 3、OpenCV、pymavlink、pupil-apriltags；
- Node.js 22+与pnpm（网页操作台）。

## 恢复仿真工作区

```bash
source /opt/ros/humble/setup.bash
cd /mnt/d/Codex/UAV/simulation/air_ground_sim_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/air_ground_sim/requirements.txt
colcon build --symlink-install --packages-select air_ground_sim
source install/setup.bash
```

网页操作台依赖需要重新安装：

```bash
cd /mnt/d/Codex/UAV/simulation/air_ground_sim_ws/src/air_ground_sim/web_ground_station
pnpm install
```

## 安全边界

迁移包中的仿真通过记录不代表实机已经具备安全飞行条件。新的Codex任务不得自动解锁、
启动电机或操纵真实无人机/小车。实机测试必须保留拆桨或架空轮、人工RC最高优先级、
硬件急停、速度限制和现场安全员。

