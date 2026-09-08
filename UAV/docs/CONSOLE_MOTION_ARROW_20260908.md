# 网页运动指令箭头

鲜红色箭头从预览中心出发，表示树莓派发往 `/mavros/setpoint_raw/local` 的水平速度方向，不是识别目标位置、控制候选或实测运动。升降速度单独显示。

只读 `flight_status_http` 订阅实际速度目标和 MAVROS 位姿。ROS 输入的 LOCAL_NED/LOCAL_OFFSET_NED 消息数值按 ENU 解释，通过当前姿态转回机体 FLU；BODY_NED/BODY_OFFSET_NED 的 ROS 输入按 FLU 解释。屏蔽速度分量、未知坐标系或缺失姿态时不猜方向。

画面映射依据 2026-09-08 的现场轴向确认：画面上为机体前，画面右为机体右。因此显示向量为 `image_x=-body_flu_y, image_y=-body_flu_x`。此映射只用于网页，不修改控制器外参。

状态桥只提供 0.5 秒内的新指令；网页计入接收后时间，超过 0.7 秒隐藏。退出 GUIDED、链路断开、无指令、零水平速度时不画水平红箭头。LAND 模式没有新树莓派速度时明确提示飞控自主 LAND，不从旧指令猜测方向。

验证：`tools/test_motion_command_status.py` 覆盖五个航向、机体系输入、姿态缺失、屏蔽轴、未知帧、过期和模式退出；`tools/test_vision_direction_overlay.cjs` 覆盖画面方向、斜向、升降文字及箭头绘制。测试不会发布 ROS 消息或发出运动命令。

状态桥以独立 ROS 2 节点运行，只读订阅飞控状态、位姿和已发布的速度目标；它不发布控制消息。部署或更新前仍需保持未解锁，并保留可回滚的上一版服务配置。
