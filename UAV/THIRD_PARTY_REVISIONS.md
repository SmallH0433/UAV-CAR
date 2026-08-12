# 第三方源码版本

大型第三方仓库未放入迁移包，以避免超长路径、Git子模块、编译产物和压缩兼容问题。
请在新电脑恢复到以下版本。

## ArduPilot

- 仓库：`https://github.com/ArduPilot/ardupilot.git`
- 提交：`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538
git submodule update --init --recursive
```

## ArduPilot Gazebo

- 仓库：`https://github.com/ArduPilot/ardupilot_gazebo.git`
- 提交：`082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
```

恢复后的推荐目录：

```text
D:\Codex\UAV\air_ground_open_source\01_flight_stack\ardupilot
D:\Codex\UAV\air_ground_open_source\06_simulation\ardupilot_gazebo
```

