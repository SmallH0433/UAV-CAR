# 商业化生产就绪门禁

## 当前结论

本仓库的目标是形成可迁移、可审计、故障闭锁的软件生产候选基线。它不是仅用于演示的开环脚本；同时，在真实 Hunter、Pixhawk、传感器、锁止机构和物理安全回路完成 HIL/场地/法规证据前，也不得标称“工业安全认证完成”。

## 生产控制链

```text
Nav2 闭环输出 ─┐
               ├─ UGV 控制权仲裁 ─ Nav2 Collision Monitor ─ 底盘适配器 ─ 命令网关 ─ Hunter CAN
人工点动输出 ──┘          │                  │                   │             │
任务门控心跳 ─────────────┘                  └─ 雷达超时停车      └─ 门超时      └─ 命令超时
                                              system_supervisor ────────────────┘

UAV 导航/跟飞/对接 ─ UAV 优先级仲裁 ─ MAVLink 限幅/看门狗 ─ Pixhawk/ArduPilot 内环与 failsafe
                              │                         │
                       system emergency-stop       RC/GCS/battery/EKF/geofence

任务锁止请求 ─ 双路接触/锁舌互锁网关 ─ 锁止 MCU/PLC ─ 机械防脱
                    │                    │
             动作/反馈超时闭锁       独立硬件限位与失电安全
```

关键原则：

1. 所有运动接口上电默认关闭；
2. 任务、人工和自主控制只有一个有效控制权；
3. 发布者退出、消息过期、传感器过期和外部急停都关闭运动链；
4. 飞行中“急停”不切断电机，而是阻断伴随计算机速度命令、终止任务，并交由 ArduPilot/飞手执行 LAND、RTL 或手动接管；
5. 已锁住的无人机不会因通用 abort 自动释放；
6. 静止停靠必须先确认落地并解除武装再锁止；移动捕获只允许在低高度、低相对速度、低横摆角速度、目标航向对齐、`LAND` 模式、新鲜地图位姿和甲板相对测距组成的包线内发生；
7. 移动捕获后的解除武装有独立超时，超时即闭锁，禁止载车继续运动；
8. 复位必须满足物理急停健康、无人机未解锁、无人车静止、任务不活动和无当前关键故障。
9. 真实锁止命令必须通过双通道反馈网关，重复请求不得延长动作超时，软件退出不得自动释放。

## 发布门禁

| Gate | 通过条件 | 自动化程度 | 当前 |
|---|---|---:|---|
| G0 构建完整性 | ROS 包构建、前端构建、XML/YAML 解析 | 自动 | 可执行 |
| G1 单元/属性测试 | 控制适配、状态机、空域、融合、MAVLink、安全监督、审计 | 自动 | 可执行 |
| G2 安全配置基线 | `verify_production_baseline.py` 全通过 | 自动 | 可执行 |
| G3 SIL 完整任务 | 无人工介入完成远端降落、并行避障、静态/移动降落和联合停止 | 自动/录像 | 可执行 |
| G4 故障注入 | kill 节点、断雷达、断 MAVLink、网页断线、延迟/丢包、电池低、定位漂移 | 半自动 | HIL 待完成 |
| G5 控制器 HIL | Pixhawk + ArduPilot、Hunter CAN 控制器、双通道锁止 I/O | 硬件 | 待完成 |
| G6 整机封闭场地 | 制动距离、飞行中止包线、对接成功率、RC 接管 | 实测 | 待完成 |
| G7 安全/网络评审 | 风险评估、PLr/安全回路、威胁模型、渗透、密钥管理 | 独立评审 | 待完成 |
| G8 法规和量产 | 目标市场民航/机械/无线电/电气、制造一致性、追溯 | 合规 | 待完成 |

G0–G3 通过仅代表 SIL 发布候选；商业现场发布必须由产品负责人对 G4–G8 签字。

## 建议验收指标（需由风险评估最终确认）

- UGV 软件门在心跳消失后 250 ms 内关闭；最终轮端停止距离按最大载荷和地面附着实测；
- Collision Monitor 的真实雷达数据超过 300 ms 未更新即停车；
- 人工点动松手/网络断线 350 ms 内归零；
- UAV 速度命令超过 300–400 ms 未更新即发送零速度目标；
- 空中关键感知丢失触发系统闭锁和任务中止，不自动恢复原任务；
- 任务状态、关键故障和全部写命令具备 UTC 时间、操作者、请求 ID 和结果审计；
- 停靠捕获必须同时满足新鲜视觉、健康的甲板相对测距、位姿/速度包线和状态机许可；静止锁止前必须确认落地与解除武装；
- 联合减速必须以带动态时间戳检查的 TF `map→base_link` 计算地图帧剩余距离并生成 Nav2 上游限速，不得因事件驱动的 AMCL 消息停更、墙钟经过或局部规划停滞而自行降至不可控蠕动速度；
- Nav2 在完成内部恢复后返回拒绝/中止时，协同状态机立即关闭运动门并进入可审计故障态；
- 移动捕获后，实机解除武装确认超时不得大于 8 s；实际值需结合飞控日志、机构接触时间和风险评估收紧；
- 锁止执行超时从第一次请求起算；四路反馈任一超过 200 ms、双路不一致或锁止后失去接触均闭锁；最终阈值须由 I/O 周期和风险评估确认；
- 目标 Jetson 在最坏温度、传感器带宽和推理负载下保留 CPU/GPU/内存余量；
- 至少执行 72 h 连续运行、100 次完整任务和按风险分层的对接统计。

## 实机分阶段策略

1. 台架：轮胎离地、桨叶拆除，只验证 I/O、符号、超时和急停；
2. HIL：Pixhawk 接仿真动力学，Hunter 控制器接真实 CAN；
3. 低能量：无人车限速，无人机系留，锁止机构假负载；
4. 封闭场地：静止平台降落后再测试移动平台；
5. 任务包线扩展：每次只扩展一个速度、曲率、风或照明维度；
6. 试运营：有安全员、RC 最高优先级、完整日志和回滚版本；
7. 商业发布：仅使用签名镜像、冻结参数和已批准硬件 BOM。

## 上游机制依据

- [ROS 2 managed node lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- [Nav2 Collision Monitor](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html)
- [Nav2 Collision Monitor deployment chain](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
- [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ArduPilot Copter failsafes](https://ardupilot.org/copter/docs/failsafe-landing-page.html)
- [ArduPilot pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
