# 新执行器外围功能迁移

基准：树莓派上一版生产硬件配置，以及旧 `guided_executor` 的默认参数与提示音编码。现场发布目录不纳入公开仓库。

| 项目 | 新版实现 |
|---|---|
| 水平限速、加速度 | hardware / hardware-preview 显式设置 0.20 m/s、0.40 m/s² |
| 视觉候选有效期 | candidate_timeout_s → candidate_maximum_age_s，1.0 s |
| 跟飞丢标宽限 | follow_dropout_grace_s → follow_loss_grace_s，1.0 s |
| 实机测距来源 | /landing/sensor_range；新版服务依赖 ov9281-range-bridge.service |
| 模式重试 | 1.0 s；心跳确认时限沿用 2.0 s |
| 提示音 | 复用 FollowTonePolicy、legacy PLAY_TUNE（258）、原旋律；跟飞 3 s、降落 2 s |
| 提示音启用 | 旧版最后配置为 false；按本次恢复提示音要求在新版 hardware 开启，preview/offline 关闭 |
| 目标速度回传 | 订阅 /mavros/setpoint_raw/target_local，0.5 s 过期，记录数量、连续性、速度与同坐标系发送值差值 |
| 回传频率请求 | CommandLong 511，消息 85，200000 µs（5 Hz）；失败/无回复 2 s 重试，断链后重新请求 |
| 遗留 GUIDED | 启动/重连观察到已解锁 GUIDED 且无当前动作或保持输出时，等 1 s 后请求 LOITER，等待模式心跳；不会主动进入 LAND 或发送速度 |

模式回退不要求先重新建立 RC6 会话，否则无法处理程序重启后的无主 GUIDED。
飞手在恢复期间切到其他模式即结束恢复；启动时已在普通模式，则不把后来手动进入 GUIDED 当作遗留状态。
原生 LAND、已上锁、当前动作持有控制权、遥测不新鲜时不会触发该回退。

跟飞提示需要新鲜回传和实际发送设定值；GUIDED 跟踪下降使用降落提示，不误报为跟飞。
丢标零速度等待不会播报正在下降；LAND 本身由模式心跳确认。
频率请求 ACK 不代表收到回传，更不代表飞行器执行了对应运动。回传是飞控目标值，不是测得速度；目前沿用监测用途，不因单次回传丢失直接中止动作。

新增 `state_maximum_age_s: 2.0` 是 v2 的显式硬件适配：兼容通常 1 Hz 的飞控心跳，防止旧 0.5 s 默认值反复误判断链。它不是从旧版同名参数复制的值。
RC6/CH8 阈值和新降落动作的 0.10 m 交接、恢复等待及低空丢标 DISARM 策略保持既定设计。
旧的控制权协调器/Elastic 仲裁不重新启用，避免出现第二个动作控制出口。

所有外围写操作遵守 environment/flight_use_approved 与各自输出开关，preview/offline 不发送提示音、频率请求或回退模式命令。
状态话题与网页状态 API 均新增 target_echo_*、tone_*、orphaned_guided_* 字段供核对。

验证：本地执行器、提示音和外围测试；树莓派隔离容器构建、ROS 模拟回传/提示音、启动 GUIDED 回退及完整模式流程。实机声音可闻性和空中回退效果需单独验收。
