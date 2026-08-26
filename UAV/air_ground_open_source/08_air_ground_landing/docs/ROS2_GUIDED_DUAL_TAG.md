# ROS 2 GUIDED 执行与 OV9281 双标签说明

## 结论

项目正式基线为 Ubuntu 22.04 + ROS 2 Humble。`Elastic-Tracker` 和 `ibvs_sim` 原仓仍是 ROS 1 参考代码，生产路径使用新建的 `air_ground_landing_ros2` 适配层：

- Elastic 算法输出改接标准 ROS 2 `MultiDOFJointTrajectory`；
- ibvs_sim 只保留特征误差到水平速度修正的思想，读取当前 OV9281 标定和角点，不移植 PX4/MAVROS1 电机逻辑；
- GUIDED 执行器集中做控制权、RC、时效、模式确认和回滚，避免多节点并发写飞控。

## CH6 跟飞、CH7 定位源与 CH8 下降

`CH5` 保留为 ArduPilot 飞行模式通道；`CH6` 是跟飞总开关；`CH7` 通过 `RC7_OPTION=90` 选择 EKF 源组（低位光流，中/高位 GPS）；`CH8/SwD` 是仅在跟飞期间生效的下降开关。SwD 不是新的 ArduPilot 飞行模式开关，也不直接写飞控模式，因此不会替换 CH5 上的 LOITER/BRAKE/RTL 等模式。

执行顺序如下：

1. CH6 高位且 Elastic/IBVS 候选有效，执行器请求 GUIDED；只有 `/mavros/state` HEARTBEAT 确认 GUIDED 后，才视为跟飞已建立。
2. 每次跟飞建立后，SwD 必须先处于低位，再发生低到高的边沿；若启动跟飞时 SwD 已经在高位，状态为 `NEEDS_REARM`，不会意外下降。
3. SwD 高位发布 `/landing/descent_request=true`。监督器仍需确认目标、速度匹配和对准条件，随后协调器把唯一控制权切到 `AC_PRECLAND_LAND`，模式管理器才请求 LAND。
4. SwD 回到低位立即发布 false，监督器退回 `MATCH_VELOCITY`，协调器恢复 IBVS/Elastic 跟飞，模式管理器请求 LAND→GUIDED；原进入模式仍作为最终回滚点。
5. CH6 关闭、RC 数据陈旧、控制权超时或人工切换模式会撤销自动控制并走既有回滚/人工接管路径。

正式配置读取 `CH6` 与 `CH8` 原始 PWM，因此 `RC6_OPTION=0`、`RC8_OPTION=0`；`RC7_OPTION=90` 只负责在已经配置的三套 EKF 源之间切换。实机映射前应拆桨查看 `/mavros/rc/in`，确认各物理拨杆的通道和 PWM。

## 双标签

标签板使用同心嵌套 `tag36h11`：外层 ID 0 黑边 100.0 mm，内层 ID 1 黑边 20.0 mm，内层逆时针旋转 45 度。两者中心重合，因此切换标签不会引入目标点跳变。服务每帧分别用各自真实边长解算位姿：

可打印成品为 [`ov9281_nested_tag36h11_id0_100mm_id1_20mm.pdf`](../../../output/pdf/ov9281_nested_tag36h11_id0_100mm_id1_20mm.pdf)。

- 大于切换区间时优先 ID 0；
- 小于 0.30 m 时切到 ID 1；
- 0.30-0.40 m 为 0.05 m 滞回区，防止两个标签来回抖动；
- `/api/status.detections` 保存同帧所有结果，顶层字段只暴露当前主标签，兼容现有 `LANDING_TARGET` 桥。

打印必须选择“实际大小/100%”，关闭“适合页面”。打印后用卡尺量黑色外框边长，而不是纸张或白色静区。外层必须为 100.0 mm，内层必须为 20.0 mm；若实测不同，应按实测黑边修改配置，不能继续使用名义尺寸。

2 cm 标签的最终可用高度还受焦距、运动模糊、曝光和 `quad_decimate=2.0` 影响。合成图能证明编码共存，不能替代 OV9281 实拍距离测试；实机下降开关保持关闭，直到从 0.4 m 到触地高度完成逐级识别率和位姿跳变记录。
