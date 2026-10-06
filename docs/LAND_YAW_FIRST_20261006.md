# 先调机头，再下降（2026-10-06）

## 新流程

CH8 启动 LAND 任务后，先进入 `GUIDED_ALIGN_YAW`：保持高度目标，使用原有横向纠偏保持目标可见，并调整机头朝向。

航向误差在 4° 内、测得的航向转动速度不超过 3°/s，并保持 0.5 s 后，允许进入 `GUIDED_TRACK_DESCENT`。较短的观测缺口只暂停输出；两次有效对准证据间隔超过 0.3 s，会重新计稳定时间。有效观测报告航向超限或仍在较快转动，也重新计时。

进入下降后，对准完成标志锁存到本次任务结束，横向纠偏继续，主动转向速度固定为 0。目标短暂丢失、重获时不会重新追着 Tag 调头。新任务重新要求对准。

同样的转向/下降互斥约束也用于独立 `PRECISION_LAND` 动作。近地交接飞控 LAND、遥控撤销、人工接管与目标失效处理仍由原来的优先级路径负责。

本次按照“把调头与下降分开”的范围修改，没有新增横向偏差准入门槛；横向对准和下降包线可作为后续独立改动。这里的保持高度和零转向是控制目标，不代表真实机体不会受到惯性或估计误差影响。

## 修改文件

- `air_ground_landing/src/air_ground_landing/action_execution.py`：新增航向对准阶段、稳定判定及任务内锁存，禁止下降时发送转向。
- `air_ground_landing/ros2_ws/src/air_ground_landing_ros2/air_ground_landing_ros2/action_executor.py`：传入新增参数，状态中增加 `landing_turn_while_descending=false` 和 `landing_yaw_alignment_complete`。
- `air_ground_landing/ros2_ws/src/air_ground_landing_ros2/config/action_executor.hardware.yaml`：配置 0.5 s 稳定等待及 3°/s 转动速度门槛，继续使用原来的 4° 航向容差。

完整差异保存为 `yaw_first.patch`，修改前部署代码和配置保存于 `installed_before`；现场原始发布目录归档为 `deployed_release_before.tar`。

## 验证

运行 7 个相关离线测试模块，共 94 项。首轮 93 项通过；一项旧 RC 测试夹具缺少当前版本字段、并沿用过时的重授权断言，按部署前的实际行为修正后，该项单独复跑通过。没有因此修改 RC 控制逻辑。

新增行为测试覆盖：大航向误差不下降、稳定等待、仍在转动时不下降、误差或长观测间断重新计时、短缺口暂停输出、下降及重获时不再调头、新任务重新对准，以及遥控撤销和近地 LAND 交接优先级。

第一次加载后，18 s 内未收到完整运行状态，部署脚本已自动回退。旧版本重启也出现 20 多秒初始化时间，树莓派负载较高；将启动等待延长到 60 s 后重新加载。最终现场结果见 `deployment_result.json`。脚本在写入前检查实时地面、未解锁及任务空闲状态，并保留现场回退文件。

最终部署结果为 `ACTIVATED`。部署前后均确认飞控连接正常、STABILIZE、未解锁、ON_GROUND、任务 IDLE，且执行器持续发布新鲜状态；五个现场源文件/安装文件/配置文件的 SHA-256 均与补丁一致。

现场回退备份保留在实机，不纳入公开仓库。尚未进行实飞验证。
