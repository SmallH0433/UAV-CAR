# 旧系统 → 当前 QAV280 系统一致性审计

审计日期：2026-08-21  
审计方式：只读参数、代码、服务、ROS 2 图、运行日志与 DataFlash 对比。未写飞控参数，未切换模式，未解锁，未发送电机命令。

## 一、审计结论

这次迁移的 **Pixhawk 选择性参数写入本身是完整的**：迁移清单中的 108 项，在本次 Log 41 参数快照中仍为 **108/108 一致，0 项缺失**。

目前出现不一致的主要原因不是“旧参数没有写进去”，而是：

1. 旧的单体 `pymavlink` 跟飞程序被替换为 ROS 2/MAVROS 后，部分隐含运行行为没有移植；
2. Elastic、平台估计器和完整移动降落监督器虽然存在代码和单元测试，但没有全部接入当前实时 launch；
3. QAV280 更换机体后，部分必须重新测量的物理标定仍处于假设值或单点候选值；
4. 8 月 18 日迁移完成后，又陆续修改了 RC、提示音、双 Tag、精准降落和遥测参数，配置文件之间发生语义漂移。

当前系统不应按“完整移动平台自主降落闭环已经迁移完成”验收。当前能确认的是：双 Tag 视觉、IBVS 候选、GUIDED 模式请求、速度输出、LANDING_TARGET 和 CH8 LAND 分支分别工作过；但完整的目标回显、安全门、Elastic 会合、平台运动估计和移动触地监督尚未形成统一实机链路。

## 二、审计基线

- 旧飞控完整母本：`pixhawk_backups/pixhawk_parameters_20260818_190345/pixhawk_full.param`，1032 项。
- QAV280 迁移完成快照：`qav280_migration/final/pixhawk_parameters_20260818_193506/pixhawk_full.param`，1015 项。
- 迁移清单：`qav280_migration/stage2_selected_migration.json`，108 项。
- 当前飞控快照：`flight_logs/20260821_171322_log041/pixhawk_log_041.BIN` 的 PARM 表，1015 项。
- 旧跟飞程序：`ov9281_debug/ov9281_follow_props_off_test.py` 与 `config/ov9281_follow_props_off_control_20260814.json`。
- 当前伴随计算机：树莓派 `192.168.1.126`、镜像 `910ee1c...d40`。
- 当前视觉、MAVROS、ROS 2 降落服务均为 enabled/active，`NRestarts=0`。

COM10 被 Mission Planner 占用，无法再次在线下载参数；当前参数以刚导出的完整 Log 41 启动参数表为准。

## 三、飞控参数审计

### 3.1 迁移项

- 108/108 个选择性迁移参数仍与迁移清单一致。
- CH5/CH6/CH7/CH8 业务角色、EKF Source、TELEM1/TELEM2、光流/测距、PLND、LAND、失控保护和日志策略没有出现迁移后漂移。
- 当前仍为 `RC6_OPTION=0`、`RC7_OPTION=90`、`RC8_OPTION=0`、`PLND_ENABLED=1`。

### 3.2 迁移后变化

迁移完成快照到 Log 41 共有 52 个共同参数发生变化：

- 45 项为新机 IMU、罗盘、气压计、姿态 Trim 或动力学习等硬件相关值；这类变化符合“新机重新标定”的原则。
- 6 项为运行/飞行计数。
- 1 项为遥测：`MAV2_RC_CHAN: 0 → 5`。

需要单独关注：

- `COMPASS_USE2=0`、`COMPASS_USE3=0`：当前只使用一套罗盘，功能可运行，但失去多罗盘冗余。
- `BATT_MONITOR=0`：当前没有飞控电池监测，也没有 ROS 2 电池门控。
- `RNGFND1_GNDCLR=0.10 m`：仍是迁移时临时值，尚未按 QAV280 落地后测距窗口到地面的实际高度确认。
- `MOT_THST_HOVER` 已由 0.35 变为约 0.68；可能是飞行学习结果，但应结合悬停日志复核。
- `MOT_SPIN_ARM=0`：与当前希望落地/低油门减少电机空转的方向一致，但属于新机动力行为变化。

完整逐参数结果见 `parameter_differences.csv` 与 `parameter_audit.json`。

## 四、已一致或已正确替代的部分

1. 当前容器内 `guided_executor`、`ibvs_adapter`、`landing_target_adapter`、`simple_landing_coordinator`、提示音策略与本机源码 SHA-256 全部一致。
2. 当前视觉服务和双 Tag 代码也与本机工作副本一致。
3. OV9281 内参文件保持一致；双 Tag 为 ID0/100 mm 与 ID1/20 mm。
4. 当前 RC 输入实测约 5.0 Hz，CH6/CH7/CH8 能被 MAVROS读取。
5. MAVROS 与飞控连接正常，服务无重启。
6. LANDING_TARGET 已被飞控记录为 `PrecLand: Target Found`，说明输出通路曾到达飞控。
7. 模式管理器已实现 MAVROS service ACK、飞控 HEARTBEAT ACK 和回滚状态机，纯逻辑测试通过。
8. 当前纯逻辑测试共运行 26 项：25 项通过，1 项因 Windows 测试环境没有 OpenCV而跳过；跳过项是双 Tag 图片解码测试，不是实机服务启动测试。

## 五、阻断完整实机闭环的问题（P0）

### P0-1：旧飞行就绪门没有完整迁移

旧程序进入跟飞前检查：已解锁、允许入口模式、EKF 相对位置、EKF 原点、姿态、光流质量 ≥80、距离 0.5–1.5 m、遥测新鲜度和 Tag 状态。

当前执行器只检查：MAVROS连接、owner 新鲜、候选 ≤0.4 s、入口模式和 CH6。没有检查已解锁、光流、EKF、距离、电池或姿态。

结果：当前程序可以在未解锁时请求 GUIDED；本次审计实时读取到飞控 `armed=false` 但 `mode=GUIDED`。

### P0-2：旧的驾驶员摇杆接管保护没有迁移

旧程序监视 RC1/RC2/RC4，摇杆偏离中心超过阈值会退出跟飞。当前 ROS 2执行器没有等价逻辑；飞手只能通过 CH6低位或原飞行模式开关退出。

### P0-3：目标回显动态申请没有迁移

旧程序启动时执行：

- `REQUEST_DATA_STREAM_ALL=10 Hz`；
- 单独请求 `POSITION_TARGET_LOCAL_NED=5 Hz`。

当前项目 ROS 2节点没有等价的消息间隔申请。`/mavros/setpoint_raw/target_local` 的发布者和订阅者 QoS 匹配，但当前一直没有收到回显；部署计数为 `FOLLOW_CONFIRMED=0`。

应只补目标消息的定向申请，不建议重新申请全部遥测 10 Hz。

### P0-4：退出后没有可靠恢复入口模式

最新测试结束后的实时状态为：

```text
phase=IDLE
current_mode=GUIDED
control_owner=HOLD
candidate_gate=OWNER_HOLD_NOT_GUIDED
setpoint_transmitted=false
CH6=2000
CH8=1000
```

即系统不再发送跟飞设定值，但飞控仍留在 GUIDED。现有回滚逻辑在正常活动会话中存在，但服务重连、目标丢失、未确认会话或链路异常后没有保证最终恢复 LOITER。

### P0-5：Elastic 没有进入当前控制仲裁

`elastic_trajectory_adapter` 被 launch 启动，但 `simple_landing_coordinator` 只订阅 IBVS候选，不订阅 Elastic候选。即使 `/elastic_tracker/trajectory` 有数据，当前 owner 也不会切到 Elastic。

### P0-6：平台估计器和完整监督器没有成为 ROS 2运行节点

`moving_pad_estimator` 和 `moving_landing_supervisor` 的纯 Python逻辑及测试存在，但 `setup.py` 没有对应 console entry point，`adapters.launch.py` 也没有启动它们。当前实际运行的是简化协调器：IBVS新鲜时跟飞、CH8请求时切 LAND。

因此目前没有实机运行的“小车里程计融合、平台速度预测、会合、速度匹配、移动触地条件和丢标退出状态机”。

### P0-7：相机平移外参仍错误地假设位于机体中心

当前配置仍为：

```text
translation_m=[0,0,0]
status=orientation_measured_xy_translation_assumed_centered
flight_use_approved=false
```

用户已确认相机不在 QAV280底盘中心。该误差会直接成为最终对准的固定 X/Y偏差，必须测量机体系 FRD下的相机光心 X/Y/Z安装偏移。

### P0-8：电池安全链路缺失

`BATT_MONITOR=0`，旧配置虽然具有最低电压/剩余电量阈值，但当时也关闭了强制遥测；当前 ROS 2执行器完全没有电池门控。该状态不适合以“完整实机自主链路”验收。

## 六、重要但非立即阻断的问题（P1）

1. 旧程序最大跟飞速度 0.10 m/s、加速度限制 0.15 m/s²；当前 IBVS最大 0.25 m/s，且没有时间域加速度限制。这是明显行为变化，尚未形成实机批准记录。
2. 旧目标回显误差门限为 0.03 m/s；当前为 0.05 m/s。它只影响确认音，不影响指令发送。
3. 当前 Tag距离修正已切换为 QAV280 0.602 m单点比例，但两个 Tag 都标记为“待多距离、多图像位置验证”，并且 `flight_use_approved=false`；硬件输出配置却已设为 true，语义矛盾。
4. `moving_landing.prototype.json` 仍写着 `scope=offline_sitl_and_disarmed_bench`、RC候选通道7、发射机映射 pending、安全输出 false；而硬件 YAML实际启用了 CH6和飞控输出。运行时部分由 YAML覆盖，但配置文件已经不能作为可信的单一事实来源。
5. 树莓派为4核，审计时 load average约7.5、温度约75°C；`get_throttled=0x80000` 表示本次启动以来发生过软温度限制。视觉分析仍约10 Hz，但高负载可能加剧0.4秒候选超时和ROS发现延迟。
6. 网页预览与分析使用独立流，但硬件MJPEG始终编码；打开网页还会增加约6 Mbps网络发送。它不是串口故障根因，但正式试飞时关闭网页可以减少一个变量。
7. DataFlash MAV链路统计显示 channel 1约21包/s发送、20包/s接收，符合树莓派双向链路；channel 2约1.4包/s发送、200包/s接收，更像MTF-01P输入。当前 `MAV2_RC_CHAN=5` 是否真正控制树莓派所在链路仍未被单独证明；RC的5 Hz也可能来自MAVROS运行时申请。

## 七、有意变化，不属于迁移遗漏

- 跟飞授权从旧 CH7改为 CH6；CH7改为光流/GPS EKF源选择；CH8为降落。
- 旧程序不允许 LAND命令；当前新增 CH8→LAND＋AC_PrecLand。
- 提示音语义改为：Tag首次有效 C、GUIDED_ACTIVE每2秒C、回显确认 C-E-G、退出确认 G-E-C。
- 双 Tag、目标选择迟滞和不同质量门限是后续新增能力。
- 新机 IMU、罗盘、RC、动力和气压计标定不复制旧机数据是正确做法。
- 三个服务设置为开机自启动是用户明确要求，不是迁移错误。

## 八、建议的修复顺序

1. 先修 P0-4：保证 CH6关闭、候选丢失、owner丢失、MAVROS重连和服务重启后，飞控一定退出由伴随计算机进入的 GUIDED。
2. 恢复 P0-1/P0-2：移植已解锁、EKF/光流/高度/姿态门和飞手摇杆接管；保留用户要求的CH6/CH8操作方式。
3. 只针对 `POSITION_TARGET_LOCAL_NED` 增加启动及重连后的消息间隔申请，使 `FOLLOW_CONFIRMED`真正可触发。
4. 将 Elastic候选、平台估计器和完整监督器做成ROS 2节点并接入单写者仲裁；不要只运行简化协调器却称为完整融合链路。
5. 测量相机 FRD平移外参、MTF落地净空；完成双 Tag多距离修正。
6. 明确跟飞速度/加速度限制，建议先恢复旧机0.10 m/s和0.15 m/s²基线，再分阶段放宽。
7. 配置电池监测，解决散热和高负载，再进行装桨低空测试。
8. 合并硬件 YAML和 `moving_landing.prototype.json` 的冲突字段，生成唯一的“当前实机配置清单”。

## 九、当前验收边界

已验证：参数迁移、服务启动、RC 5 Hz、双 Tag逻辑、模式ACK状态机纯逻辑、GUIDED进入、LANDING_TARGET到达、CH8 LAND台架分支、提示音发送。

尚未验证：目标回显确认、退出后全故障路径回滚、驾驶员摇杆接管、真实水平跟飞响应、Elastic会合、平台速度融合、移动下降、脚架触地后的 LAND_COMPLETE、双 Tag近距离交接和完整移动平台降落。

