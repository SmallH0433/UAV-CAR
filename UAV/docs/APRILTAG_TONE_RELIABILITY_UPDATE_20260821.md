# AprilTag 识别提示音可靠性改进（2026-08-21）

## 新语义

- 单音 `C`：`landing_target_adapter` 本轮接受了有效 AprilTag 观测，且 MAVROS 与飞控相连。
- Tag 首次有效识别时只提示一次单音 `C`，持续识别但尚未进入跟飞时不再周期重复。
- 飞控心跳确认 `GUIDED`、跟飞速度设定值实际输出时，以 `GUIDED_ACTIVE` 事件每 2 秒重复一次单音 `C`；退出该控制状态立即停止。
- `C-E-G`：GUIDED、速度输出和飞控目标回显均已确认。
- `G-E-C`：此前确认的跟飞已退出。

单音 `C` 不再依赖 CH6、控制 owner 或 GUIDED 进入条件。没有 Tag、Tag 未通过质量门控、MAVROS 未连接时不会误报。

## ArduPilot 4.7 兼容修复

MAVROS `play_tune` 插件固定发送 MAVLink `PLAY_TUNE_V2`（消息 ID 400），而本机 ArduPilot 4.7 只处理旧版 `PLAY_TUNE`（消息 ID 258）。因此早期 journal 中的 `transmitted:true` 只表示 ROS 消息已发布，飞控会静默忽略。

现已改为由 `guided_executor` 通过 MAVROS 路由器 `/uas1/mavlink_sink` 发送旧版 `PLAY_TUNE`。MAVROS 仍是 TELEM 串口唯一所有者，不增加第二个串口进程。纯 Python 编码器不依赖 `pymavlink`，并使用 common.xml 的 CRC extra 187 生成 MAVLink 2 帧。

网页同时增加候选质量诊断：绿框代表正式接受，蓝框代表已解码但被质量门控拒绝；页面会显示实际 margin、hamming、重投影或位姿不可用原因。

## 修改范围

- `follow_tone_policy.py`：增加有界周期提醒，跟飞确认后停止重复。
- `guided_executor.py`：订阅 `/landing/landing_target/status`，以 `accepted_this_poll` 作为真实 Tag 检测事件；MAVROS 恢复连接且 Tag 仍新鲜时补发提示音。
- `adapters.hardware.yaml`：实机启用提示音，检测新鲜度 0.5 秒，重复周期 2.0 秒。
- `test_follow_tone_policy.py`：覆盖首次提示、2 秒重提醒和跟飞后停止提醒。

## 验证与部署状态

- 本地提示音策略测试：3/3 通过。
- 两个修改后的 Python 文件语法检查通过。
- 已部署到树莓派 `192.168.1.126`，新镜像 ID 为 `c135befca9325c4a34a4228d473c41913b22b2966aed9c5de4b36b0fe5db21bb`。
- 回滚镜像为 `localhost/air-ground-landing-ros2:humble-before-tag-tone-repeat-20260821`。
- 远端文件备份为 `/home/PI/air_ground_landing/backups/tag_tone_repeat_before_20260821_1140`。
- 服务启动后读回：`tone_output_enabled=true`、`tag_detection_timeout_s=0.5`、`observe_ready_tone_repeat_s=2.0`。
- 在飞控 `armed=false`、STABILIZE、无 GUIDED/LAND 候选条件下发送 5.2 秒仅提示音的模拟有效 Tag 状态；journal 分别在 11:55:36、11:55:38、11:55:40 记录三次 `OBSERVE_READY`，均为 `transmitted:true` 和单音 `C`。
- 兼容修复镜像 ID：`d34d59bed3c9a40772c15bd1f2e445509cc754b898620ff14addc1ebb2b4a37e`。
- 兼容修复回滚镜像：`localhost/air-ground-landing-ros2:humble-before-legacy-tone-20260821`。
- MAVROS sink 实测捕获：`msgid=258`、MAVLink 2 magic 253、源 system/component 191/191、13 字节有效负载。
- 切换前后飞控均为 `armed=false`、`guided=false`、STABILIZE；未发送模式、速度或电机指令。

## GUIDED 周期音部署（2026-08-21 16:41）

- `FollowTonePolicy` 新增 `GUIDED_ACTIVE` 事件，使用与首次识别相同的单音 `C`。
- 周期参数改为 `guided_active_tone_repeat_s=2.0`；原 `observe_ready_tone_repeat_s` 语义已移除。
- 本地策略测试通过：非 GUIDED 持续识别不会重响，GUIDED 控制有效后每 2 秒产生一次 `GUIDED_ACTIVE`。
- 正式镜像 ID：`910ee1c68b6b43cb813bb6ed07e5d5b8221dd9de13897226a022409cec489d40`。
- 回滚镜像：`localhost/air-ground-landing-ros2:humble-before-guided-c-20260821`。
- 远端文件备份：`/home/PI/air_ground_landing/backups/guided_c_tone_before_20260821_164130`。
- 部署后服务为 `active`，飞控 `armed=false`、`LOITER`，CH6/CH8 均为低位；未发送解锁或电机指令。
