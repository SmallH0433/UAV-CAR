# UAV-CAR 空地协同项目

无人机（UAV）与地面无人车（CAR）协同系统的开发与调试仓库。

## 目录结构

| 目录 | 说明 |
| ---- | ---- |
| [UAV/](UAV/) | 无人机部分：Pixhawk/MAVLink 工具链、SITL 仿真测试、精准降落与跟随、降落对接设计、日志分析脚本等 |
| [CAR/](CAR/) | 地面车部分：R680 4WD 小车 ROS 2 工作区（模型描述、Gazebo 仿真、键盘遥控），详见 [CAR/README.md](CAR/README.md) |

## 快速开始

- 地面车仿真：见 [CAR/README.md](CAR/README.md)，`colcon build` 后 `ros2 launch CAR_pkg gazebo_sim.launch.py` 一键启动。
- 无人机 SITL：见 [UAV/README_MIGRATION.md](UAV/README_MIGRATION.md) 与 UAV 目录下的 `run_sitl_*.sh` 脚本。
