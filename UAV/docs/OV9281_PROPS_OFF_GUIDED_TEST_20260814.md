# OV9281 无桨 GUIDED 跟飞控制验证

## 适用范围

本流程只验证树莓派是否完成 AprilTag 观察、请求并确认 GUIDED、发送水平速度目标，以及飞控是否据此改变姿态目标和四路电机 PWM。它不批准装桨飞行。

专用入口：`/home/PI/ov9281_debug/ov9281_follow_props_off_test.py`

专用配置：`/home/PI/ov9281_debug/ov9281_follow_props_off_control_20260814.json`

配置中 `control_enabled=true`、`mavlink_transmit=true`、`mode_change=true`；同时保持 `flight_use_approved=false`、`arm_command=false`、`takeoff_command=false`、`land_command=false`、`motor_command=false`。

## 提示音语义

- 单个低音 `C`：不要求 RC7 高位。飞控已由飞手解锁、当前处于允许的入口模式，并且相机/标签、姿态、EKF、原点、电池、高度、光流和遥测条件全部满足；表示现在可以拨高 RC7。
- 上升音 `C-E-G`：飞控已连续 3 个心跳确认 GUIDED，树莓派已发送速度目标，并收到飞控回传的匹配 `POSITION_TARGET_LOCAL_NED`；表示跟飞控制链路确认。
- 下降音 `G-E-C`：此前确认的跟飞会话已经由新心跳确认离开 GUIDED，或者飞控已经上锁。
- 两个低音 `C-C`：旧观察器结束观察；它不表示曾经进入 GUIDED。

如果听到单个 `C`，保持摇杆居中后拨高 RC7。若随后没有 `C-E-G`，说明 GUIDED 切换、速度发送或目标回显尚未全部成功。

## CH7 状态与作用

- `CH7 <= 1200`：跟飞关闭。若此前由程序进入 GUIDED，程序先发送零速度，再请求恢复进入前的 `ALT_HOLD`、`LOITER` 或 `POSHOLD`。
- `CH7 >= 1800`：允许申请跟飞，但不会单独触发控制；标签、遥测、EKF、电池、高度、光流、飞行模式和解锁状态仍需满足。
- `1200 < CH7 < 1800`：模糊区，按关闭处理。
- 超过 `0.5 s` 没有新 CH7 遥测：按关闭处理。
- 程序启动时 CH7 必须先处于低位。发生退出、故障或上锁后，需要再次完成“低位→高位”循环才能重新申请。

高度就绪门限为测距高度 `0.50–1.50 m`（包含边界）；超出该范围会报告 `HEIGHT_OUTSIDE_FOLLOW_GATE`，不会申请进入跟飞。

因此，CH7 是树莓派跟飞的授权开关，不是飞控自身的飞行模式通道，也不会直接改变电机输出。

## 启动前硬条件

1. 四个螺旋桨全部物理拆除；不要仅关闭油门。
2. 机体周围清空，人员避开电机轴和松动物件，机体由一人稳定抓持或固定在不会限制姿态传感器的支架上。
3. 不得关闭 ArduPilot 预解锁检查；当前罗盘、电池监测、EKF 原点等阻塞项必须按正常方式解决。
4. RC7 起始必须小于或等于 1200；程序启动后必须先看到 5 个未解锁飞控心跳。
5. 停止 `ov9281-follow-observer.service` 和 `apriltag-follow-monitor.service`，避免多个进程争用 `/dev/serial0`。保持 `ov9281-vision.service` 运行。
6. 飞手全程掌握正常飞行模式开关；移动横滚、俯仰或偏航杆超过 150 PWM 并持续 0.20 秒会触发接管退出。

## 手动启动

```bash
systemctl --user stop ov9281-follow-observer.service apriltag-follow-monitor.service

systemctl --user start ov9281-follow-props-off-manual.service
```

服务启动后持续运行，不再设置 300 秒自动停止时限；结束测试时需手动停止服务。无桨确认令牌已经取消。该程序仍禁止配置为自启动；树莓派不会发送解锁、起飞、降落、参数写入或直接电机命令，解锁只能由飞手执行。

## 自启动与允许启动的时间

“自启动”是指树莓派开机后由 systemd 自动运行跟飞控制程序，不需要人在终端输入启动命令。无桨专用程序只安装了手动服务，并且服务文件故意没有 `[Install]` 段，不能被启用为开机自启动；配置也会拒绝 `autostart_forbidden=false`，避免以后装回桨叶时因树莓派开机而意外进入控制流程。

当前程序允许手动启动的条件是：桨叶已经拆除、飞控未解锁、CH7 低位、`ov9281-vision.service` 正常、其他占用 `/dev/serial0` 的观察服务已经停止。程序可以在罗盘、电池或 EKF 条件尚未满足时启动并显示阻塞原因，但只有全部控制条件满足后才会请求 GUIDED 和发送速度目标。

## 建议测试序列

为了把 AprilTag 控制与手持晃动造成的飞控稳定动作区分开，优先固定并保持机体水平，移动小车上的标签：

1. RC7 低、标签居中，记录 10 秒基线。
2. 保持飞行模式为 `ALT_HOLD`、`LOITER` 或 `POSHOLD`，由飞手手动解锁。
3. 保持 RC7 低，等待单音 `C`，它表示除 CH7 授权外的控制条件已全部满足。
4. 拨高 RC7；必须等到上升音 `C-E-G`，才算控制链路确认。
5. 标签依次向机头方向、机尾方向、机体左侧、机体右侧移动，每段保持 5 秒，中间回到中心 5 秒；不要推动遥控器横滚、俯仰和偏航杆。
6. RC7 拉低，等待下降音 `G-E-C`；确认飞控恢复进入前模式后，再由飞手上锁。

手持无人机跟随小车也可以生成数据，但手部倾斜会独立改变四路 PWM，因此不能仅凭电机差速证明 AprilTag 跟飞。固定机体、移动标签的证据更干净。

## 通过判据

伴随日志必须同时出现：

- `tone_events=["OBSERVE_READY"]`；
- 三次独立 GUIDED 心跳确认后的 `manager_reason="GUIDED_CONFIRMED"`；
- `movement_setpoint_tx_total` 持续增加；
- `target_echo_confirmed=true`；
- `tone_events=["FOLLOW_CONFIRMED"]`；
- 退出后 `tone_events=["EXIT_CONFIRMED"]` 且模式不再是 GUIDED。

DataFlash 应同时显示 GUIDED 模式区间、随标签方向改变的水平速度/姿态目标，以及与目标姿态相符的 `RCOU.C1...C4` 差速。仅有四路 PWM 不同不算通过，因为手持姿态扰动也会产生同样现象。

输出文件：

- `/home/PI/ov9281_debug/follow_props_off_latest.jsonl`
- `/home/PI/ov9281_debug/follow_props_off_latest.summary.json`
- `/home/PI/ov9281_debug/follow_props_off_status.json`
