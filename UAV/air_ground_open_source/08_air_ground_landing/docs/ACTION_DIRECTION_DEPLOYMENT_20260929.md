# 动作层降落方向修正部署记录（2026-09-29）

- 目标：无人机机载 Raspberry Pi，由 `ov9281-landing-ros2.service` 启动。
- 服务已切换为 `action_executor.launch.py`；旧 `guided_executor` 不再启动，保持单一 MAVROS 控制写入者。
- 本分支中的 systemd 单元是可恢复的公开配置，现场临时发布目录、主机地址和备份文件不纳入仓库。

切换前两次确认飞控连接、新鲜未解锁 `STABILIZE` 状态，且无跟随或降落动作。树莓派容器运行 83 项离线测试通过，ROS 包编译通过；隔离 ROS 域的 CH5→CH6→CH8、边下降边转向/居中、丢 Tag 恢复和退出模式模拟流程返回 `ROS_CH5_CH6_CH8_TAG_RECOVERY_FLOW_OK`。硬件参数配置在断网隔离容器中启动了视觉适配器、降落目标适配器、动作执行器和只读状态节点。

切换后网页 `/api/status` 连续四次返回 `executor_version=v2`、`action_state=IDLE`、飞控 `STABILIZE` 且未解锁；飞控状态年龄均小于 0.2 秒，图像帧年龄均小于 77 毫秒。`ov9281-landing-ros2.service` 为 active，容器中只有新动作执行器。此次未解锁、未发起飞行动作，未进行实飞验收；现场没有 AprilTag，尚未观测真实飞行中的方向修正效果。

如需回退，使用已备份的上一版 systemd 单元，执行 `systemctl --user daemon-reload` 和 `systemctl --user restart ov9281-landing-ros2.service`；回退前先确认飞控未解锁且无跟随/降落动作。
