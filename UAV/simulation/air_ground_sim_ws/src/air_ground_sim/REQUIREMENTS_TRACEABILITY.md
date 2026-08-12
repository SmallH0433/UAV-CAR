# 空地协同系统需求追踪矩阵

本文把原始需求、实现位置、自动验证和仍需外部验证的证据绑定在一起。状态定义：

- `SIL-VERIFIED`：软件在环已自动验证；
- `IMPLEMENTED-HIL-REQUIRED`：软件接口完成，但必须通过真实控制器/传感器 HIL；
- `FIELD-REQUIRED`：必须在封闭场地、真实整机上留存测试证据；
- `CERTIFICATION-REQUIRED`：是否适用及合格结论必须由产品安全/法规负责人确认。

| ID | 需求 | 当前实现与证据 | 当前状态 | 商业发布前剩余证据 |
|---|---|---|---|---|
| SYS-001 | Jetson + Ubuntu + ROS 2 作为统一计算与通信架构 | `hardware_bringup.launch.py`、`real_interfaces.yaml` | IMPLEMENTED-HIL-REQUIRED | 目标 Jetson 型号、JetPack/Ubuntu 镜像冻结；72 h 负载与温度测试 |
| UGV-001 | Hunter 以 Ackermann 为主，保留差速/四轮转向适配器 | `chassis_adapters.py`、`ugv_chassis_adapter.py`、单元测试 | SIL-VERIFIED | Hunter CAN 实车转向符号、轮距、轴距、限幅和故障码验证 |
| UGV-002 | Nav2 闭环规划、避障与路径跟踪 | Hybrid-A*、Regulated Pure Pursuit、闭环 velocity smoother、行为树 | SIL-VERIFIED | 不同载荷/地面附着下的轨迹误差、制动距离和恢复行为 |
| UGV-003 | 独立紧急碰撞层 | Nav2 Collision Monitor 位于控制权仲裁之后、底盘适配器之前 | SIL-VERIFIED | 标定后的真实安全区、最大速度/负载下停止距离 |
| UGV-004 | 人工、任务、Nav2 不得争抢命令 | `ugv_control_mux.py` 使用任务状态、操作者心跳和超时进行唯一控制权仲裁 | SIL-VERIFIED | 遥控接管、网页断线、任务节点崩溃的 HIL 故障注入 |
| UAV-001 | 云台相机、双目、2D/3D LiDAR、六向超声 | UAV Gazebo 模型、桥接配置、感知状态和网页画面 | SIL-VERIFIED | 实机驱动、内外参、时间同步、振动/强光/雨雾边界 |
| UAV-002 | 多传感器三维避障 | `uav_perception.py` 融合扫描、点云、深度与 Range；`uav_navigation.py` 执行局部避障 | SIL-VERIFIED | 飞行包线内闭环延迟、最小可检测目标、盲区和降级策略 |
| UAV-003 | 禁飞区和限高区 | `airspace.py`、局部规划器与协同世界约束 | SIL-VERIFIED | 任务区域测绘、坐标基准、法规空域和飞控原生围栏交叉验证 |
| UAV-004 | 不绕过飞控 pre-arm，正常解锁/起飞/降落确认 | `uav_mavlink_bridge.py` 使用 ArduPilot 预检、ACK、landed state；无 force-arm | SIL-VERIFIED | Pixhawk HIL、RC/GCS/battery/EKF/geofence failsafe 参数评审 |
| DOCK-001 | AprilTag 引导静止和移动平台降落 | 下视相机、AprilTag 跟踪、粗定位/视觉切换、新鲜下降帧、健康甲板相对测距和位姿/速度捕获包线 | SIL-VERIFIED | 不同照明、污损、运动模糊、甲板振动和风扰下统计成功率 |
| DOCK-002 | 真实物理锁止语义 | Gazebo `DetachableJoint` 验证任务语义；`docking_hardware_gateway.py` 对双路接触/锁舌反馈、动作超时、误变位和安全释放执行独立互锁，并纳入实机 readiness | IMPLEMENTED-HIL-REQUIRED | 真实锁止执行器/MCU、承载确认、双通道故障注入、失电状态和误释放 FMEA |
| MISSION-001 | 远端飞来找车并降落，二者分离并行避障，目的地汇合 | `air_ground_mission.py` 的显式状态机和每状态超时 | SIL-VERIFIED | HIL 全任务、任务中途传感器/链路故障 |
| MISSION-002 | 跟随移动小车、移动降落、联合减速停止 | 持续运动、地图位姿新鲜度、航向/横摆角速度许可、视觉捕获、实体附着、按剩余距离生成 Nav2 限速、导航失败快速闭锁 | SIL-VERIFIED | 真实移动平台多速度/曲率/侧风下成功率、复飞与中止包线 |
| OPS-001 | 浏览器/平板显示状态、视觉、传感器并控制仿真 | `web_ground_station`、HTTP/SSE 网关、相机 JPEG、Gazebo 控制 | SIL-VERIFIED | iPad/工业平板弱网、断网恢复、触控松手停止测试 |
| OPS-002 | 商业控制面鉴权和审计 | 生产模式强令牌、操作者 ID、请求 ID、速率限制、JSONL 审计、TLS 反代边界 | IMPLEMENTED-HIL-REQUIRED | PKI/密钥轮换、渗透测试、日志集中存储与留存策略 |
| SAFE-001 | 故障后全系统闭锁 | `system_supervisor.py`、ROS diagnostics、持久安全事件、受保护复位 | SIL-VERIFIED | 物理急停回路的 PLr 评估；ROS 只作为监控/非安全控制层 |
| SAFE-002 | 任务节点退出后无人车必须自动停止 | 任务门心跳、控制权仲裁超时、适配器门超时、硬件网关看门狗四层防护 | SIL-VERIFIED | 实机测量从进程 kill 到轮端静止的总时间/距离 |
| DEPLOY-001 | 仿真配置可迁移实机且默认安全 | `real_interfaces.yaml` 写命令全部默认关闭；`real_mission.yaml` 未委任模板阻止仿真坐标误用于现场；实机 mission 不代替飞手解锁 | IMPLEMENTED-HIL-REQUIRED | 站点任务计划签署、驱动、设备规则、时间同步、标定、HIL 和现场验收 |
| QA-001 | 自动测试、配置门禁和二次完整任务回归 | `test/`、`verify_production_baseline.py`、CI 工作流 | SIL-VERIFIED | 目标硬件 nightly、故障矩阵、长稳和发布签名 |
| COMP-001 | 工业/机械安全和航空法规 | 安全生命周期及证据门禁已定义 | CERTIFICATION-REQUIRED | 适用市场、用途、区域决定 ISO/IEC/民航/无线电合规路径 |

## 结论规则

原始仿真功能只有在自动测试、生产基线检查和完整协同任务三项均通过时才标记 `SIL-VERIFIED`。任何 `HIL-REQUIRED`、`FIELD-REQUIRED` 或 `CERTIFICATION-REQUIRED` 项都不能由 Gazebo/SITL 结果替代。
