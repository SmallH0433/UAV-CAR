# 树莓派开机飞行日志

## 当前状态（2026-09-29）

已在目标树莓派部署，`pi-flight-recorder.service` 为 enabled / active，用户 Linger=yes。
5 项本地单元测试通过；实机已记录姿态、电池、位置、速度、RC、模式、测距和控制状态。
已重启记录服务，确认旧会话正常关闭保留，新会话继续收数。
飞控内部日志存储问题单独排查，不能将树莓派遥测记录正常等同于内部 BIN 已修复。

## 全零问题的证据和修复验收

现场保留的 82–88 号完整日志，经 MAVFTP、MAVLink 和 Mission Planner 交叉核查，内容全零。这些原始日志不纳入公开仓库。
此前实时参数读取为 `LOG_BACKEND_TYPE=1`、`LOG_DISARMED=0`、`LOG_BITMASK=180222`。
这不是日志功能被整体关闭的配置；`LOG_DISARMED=0` 仅表示未解锁期间不记录，不能解释已有文件全部为零。
参数含义参见 [ArduPilot 官方日志说明](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html)。

恢复连接后先确认未解锁，读取当前固件版本、日志参数、状态报错及日志样本。
如仍为全零，需要在飞控断电后直接备份 SD 卡原文件，再区分存储介质、文件系统、固件写入或读取故障。
现有证据尚不能断定是哪一项。不要用清空日志或格式化来代替诊断。
如需换卡或重建文件系统，应保留原卡和完整参数备份。
修复必须通过新产生的日志验收：存在有效 FMT，能解析姿态/状态等记录，时间和数据随真实输入变化。
现有全零导出文件自身无法重建原飞行数据。

## 记录内容

程序订阅现有 MAVROS 和 `/landing/*/status`，不打开串口，不发送飞行命令或改写飞控参数。
实机发现缺少姿态、电池和位置遥测；请求相应消息后已恢复。
服务启用 `--request-missing-telemetry`：收到新鲜未解锁状态时，对缺失/超过 10 秒无数据的关键消息
按 1–10 Hz 请求遥测，每 8 秒最多发一条，每类最多尝试 3 次，断连重连后重新检查。
请求及应答也写入日志；收不到数据不会冒充为有效零值。未设置此选项时为完全被动记录。
包括状态、模式、姿态、位置/速度、高度、测距、RC 输入/输出、电池、飞控报错、控制目标和跟随/降落状态。
这是 ROS 遥测 JSONL，不能替代飞控高频 DataFlash BIN；记录频率取决于上游实际发布频率。
未发布的话题不生成虚假的零值；RC 通道本身为空时保留空数组。

树莓派保存路径为运行用户的 `~/flight_logs/<UTC时间>_<随机标识>/`。
每次进程启动创建新目录，记录 boot_id，因此开机和服务重启都不覆盖历史文件。
时间包含系统时间及单调时钟，避免未校时或 NTP 跳变导致事件顺序不可辨。
每 64 MiB 分段，每秒 flush/fsync；突然断电仍可能损失最近数据或留下最后一条不完整 JSON。
`status.json` 保存每个话题的收到条数、最近收到时间和数据年龄，不能仅凭文件存在判断记录正常。
磁盘剩余小于 512 MiB 时停止并在 journal 报错，不自动删除旧日志；服务每 30 秒尝试恢复。

## 部署

将 `tools/pi_flight_recorder.py`、`tools/install_pi_flight_recorder.sh`、
`config/systemd/pi-flight-recorder.service` 按目录结构复制到树莓派同一目录。
以当前 MAVROS 所属用户执行：

```bash
bash tools/install_pi_flight_recorder.sh
```

脚本使用现有 `localhost/air-ground-landing-ros2:humble` 镜像，启用用户 systemd 服务，
并设置该用户的 linger，确保无需 SSH 登录也能在开机后启动。
若当前容器使用非默认 `ROS_DOMAIN_ID` 或 `RMW_IMPLEMENTATION`，
需将相同值放入 `~/.config/pi-flight-recorder/environment`，然后重启记录服务。
部署前应核对当前镜像和 ROS 网络配置，本配置依据仓库现有 Humble 服务编写。

```bash
systemctl --user status pi-flight-recorder
journalctl --user -u pi-flight-recorder -n 30 --no-pager
find ~/flight_logs -maxdepth 2 -name status.json
```

必须确认 `/mavros/state`、`/mavros/imu/data` 等计数持续增加，RC 空值/断连等情况如实呈现。
确认真实未解锁状态后，单独重启记录服务，检查新旧目录均保留。
开机配置、真实收数及记录服务重启验证已完成。未重启树莓派整机，整机开机验证仍待完成。

## 本地验证

```powershell
.\.venv-mavlink-windows\Scripts\python.exe -m unittest discover -s tools -p test_pi_flight_recorder.py -v
```

覆盖重启不覆盖、真实数值和 NaN 保留、轮转/统计、低空间不删除、缺数据不填零、过滤图像和日志下载流量。
本地测试不包含 ROS 2 订阅兼容性、容器启动或实机开机验证；ROS 收数和容器运行另外通过实机检查。
