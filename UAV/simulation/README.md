# 空地协同仿真入口

当前可运行工程位于：

`simulation/air_ground_sim_ws/src/air_ground_sim`

它把以下三组接口统一成ROS 2话题：

| 业务接口 | 仿真端 | 实机端 |
|---|---|---|
| `/vision/image_raw` | Gazebo向下相机 | IMX296 ROS相机驱动 |
| `/uav/cmd_vel` | UDP → ArduPilot SITL | 57600串口 → Pixhawk |
| `/ugv/cmd_vel` | Gazebo差速车辆插件 | Hunter底盘ROS 2驱动 |

完整安装、启动和验收步骤见
`simulation/air_ground_sim_ws/src/air_ground_sim/README.md`。
