# ROS 2 遥控通道与移动精准降落部署记录（2026-08-18）

## 目标

最终遥控职责为：`CH5` 飞行模式、`CH6` 跟飞授权、`CH7` 光流/GPS 定位源、`CH8/SwD` 跟飞中的降落请求。树莓派通过 ROS 2/MAVROS 实现：

1. `CH6` 低到高且 AprilTag/IBVS 数据新鲜时，请求并确认 `GUIDED`；
2. 只有已确认处于 GUIDED 跟飞会话后，才接受 `RC8` 的低到高边沿；
3. RC8 开启时，持续向飞控发送质量门控后的 `LANDING_TARGET`，并请求 `LAND`；
4. ArduPilot `AC_PrecLand` 在 LAND 内完成移动目标水平跟随与下降；
5. RC8 关闭时请求返回 `GUIDED`，CH6 关闭或数据超时则按失败关闭逻辑退出。

`RC6_OPTION=0`、`RC8_OPTION=0` 是有意保留的配置：两个通道由伴随计算机读取原始 PWM。`RC7_OPTION=90` 只选择预先配置好的 EKF 源组，不触发 GUIDED 或 LAND。

## 树莓派运行方式

树莓派系统为 Debian 12 aarch64，没有原生 ROS 2 Humble。部署采用 rootless Podman 容器 `localhost/air-ground-landing-ros2:humble`，避免替换系统 Python 或影响 OV9281 相机服务。

实机镜像 ID 为 `ebd4486dccb`。MAVROS 与应用分为两个容器运行时，两者都使用 `--network host --ipc=host`；共享 IPC 是 Fast DDS 跨容器实际传输消息所必需的，仅共享主机网络只能发现节点，不能保证 RC/状态数据真正送达订阅者。

服务职责：

- `ov9281-vision.service`：宿主机相机唯一所有者，提供 `http://127.0.0.1:8765/api/status`；
- `ov9281-mavros.service`：容器内 MAVROS，是 `/dev/ttyAMA0` 的唯一串口所有者；
- `ov9281-landing-ros2-preview.service`：读取 CH6/CH8、视觉和飞控状态，但禁止模式、速度及 LANDING_TARGET 实机输出；
- `ov9281-landing-ros2.service`：通过所有台架门控后使用的正式执行服务；
- 旧 `ov9281-follow-props-off-manual.service` 与上述新服务互斥，不能同时占用串口。

## 飞控参数实机结果

精准降落写入前备份为 `/home/PI/air_ground_landing/config/precision_landing_before_20260818.json`。遥控职责变更前后的备份分别为 `rc_roles_before_20260818.json` 和 `rc_roles_after_20260818.json`。所有遥控/EKF 写入均在 CH6/CH7/CH8 为 1000、连续未解锁心跳成立时执行，随后安全重启飞控并回读：

```text
FLTMODE_CH=5
RC5_OPTION=0
RC6_OPTION=0
RC7_OPTION=90
RC8_OPTION=0
PLND_ENABLED=1
PLND_TYPE=1
PLND_OPTIONS=1
PLND_EST_TYPE=1
PLND_STRICT=1
PLND_ALT_MIN=0.75
PLND_ALT_MAX=8.0
PLND_XY_DIST_MAX=2.5
```

CH7 的三段源组映射为：

```text
低位 / source set 1: POSXY=None, VELXY=OpticalFlow, POSZ=Baro, VELZ=None, YAW=Compass
中位 / source set 2: POSXY=GPS,  VELXY=GPS,         POSZ=Baro, VELZ=GPS,  YAW=Compass
高位 / source set 3: POSXY=GPS,  VELXY=GPS,         POSZ=Baro, VELZ=GPS,  YAW=Compass
EK3_SRC_OPTIONS=0
```

中位和高位故意配置为相同的 GPS 源，因此二段或三段物理拨杆都不会切入未配置的第三源组。低位保持原来的室内光流工作方式。

其中 `PLND_OPTIONS=1` 只开启移动着陆目标位，没有开启快速最终下降。

树莓派重启后又通过 MAVROS 参数节点回读一次，以上 `PLND_*` 值保持不变，确认已经写入飞控持久参数而不是仅存在于临时进程中。

## 代码门控

- `guided_executor` 读取 `/mavros/rc/in` 的 CH6/CH8；
- CH8 必须先在已确认 GUIDED 跟飞期间出现低位，再接受低到高边沿；
- `simple_landing_coordinator` 只有在视觉目标新鲜、质量门控通过且 LANDING_TARGET 实际输出已启用时，才把控制权交给 `AC_PRECLAND_LAND`；
- `landing_target_adapter` 将 OV9281 相机坐标变换到 `BODY_FRD`，再预转换为 MAVROS 所要求的 ROS `base_link` FLU 表达，避免 Y/Z 二次翻转；
- MAVROS 是唯一 MAVLink 串口写入者，视觉服务不直接打开飞控串口。

## 验证与回滚

本地纯逻辑与坐标转换测试：19 项通过。ROS 2 包使用独立临时目录完成 `colcon build`，新节点及硬件配置均成功安装。

实机无动作验证结果：

```text
MAVROS connected=true
FC armed=false, mode=ALT_HOLD
RC6=1000, RC7=1000, RC8=1000
guided_executor execution_enabled=true
follow_rc_gate=ABORT
landing_requested=false
setpoint_transmitted=false
landing_target_adapter output_enabled=true
vision reason=TARGET_NOT_FOUND
mavlink_transmitted=false
```

这说明新程序已真实读取 CH6/CH8；在 CH6 未授权时不会请求 GUIDED/LAND，也不会发送速度设定值。CH7 低位由飞控选择光流源。相机当前未见 Tag，因此精准降落消息按设计不发送。

已执行一次树莓派重启验证。当前开机配置为：

```text
PI user Linger=yes
ov9281-vision.service              enabled + active
ov9281-mavros.service              enabled + active
ov9281-landing-ros2.service        enabled + active
ov9281-landing-ros2-preview.service disabled + inactive
ov9281-follow-props-off-manual.service disabled + inactive
```

部署和验证期间没有解锁飞控、切换飞行模式或发送电机命令。

快速状态检查：

```bash
systemctl --user status ov9281-mavros.service ov9281-landing-ros2.service
podman ps
podman exec ov9281-landing-ros2 bash -lc \
  'source /opt/ros/humble/setup.bash; source /opt/air_ground_landing/ros2_ws/install/setup.bash; ros2 topic echo --once /landing/guided_executor/status'
```

回滚为旧服务：

```bash
systemctl --user disable --now ov9281-landing-ros2.service ov9281-landing-ros2-preview.service ov9281-mavros.service
systemctl --user enable --now ov9281-follow-props-off-manual.service
```

飞控精准降落参数回滚工具：

```bash
/home/PI/venvs/landing/bin/python \
  /home/PI/air_ground_landing/pi_configure_precision_landing.py rollback
```

回滚工具同样要求连续确认飞控未解锁；执行后需重启飞控。
