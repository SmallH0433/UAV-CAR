# QAV280 Pixhawk 参数迁移结果

## 结果

- 迁移日期：2026-08-18
- 源飞控：Pixhawk1 / ArduCopter 4.7.0 / Git `1511f271`
- 目标飞控：Pixhawk1 / ArduCopter 4.7.0 / Git `1511f271`
- 目标机架：`FRAME_CLASS=1`、`FRAME_TYPE=1`，QUAD/X
- 最终迁移核验：108/108 项匹配，0 项缺失或不一致
- 最终状态：`STABILIZE`、未解锁
- 固件刷写：Erase、Program、Verify 均为 100%

源飞控与目标飞控的 UID 不同，已确认本次写入对象是新的 QAV280 飞控。

## 已迁移的功能配置

- CH5 飞行模式通道及旧机模式档位。
- CH6 跟飞原始开关、CH7 EKF/光流-GPS 数据源切换、CH8 自动降落原始开关。
- 室内 EKF 默认原点：北纬 22.13527870°、东经 113.54499817°、参考高度 0 m。
- EKF Source 1 光流方案，以及 Source 2/3 GPS 方案。
- TELEM1 树莓派 MAVLink 2（57600）与 TELEM2 MTF-01P MAVLink 1（115200）。
- MTF-01P 光流与 MAVLink 测距仪类型、向下安装方向和 0.01–8 m 使用范围。
- AC_PrecLand：启用、MAVLink `LANDING_TARGET`、移动目标选项、严格策略及高度/距离门限。
- LAND 速度、允许水平修正、`PILOT_THR_BHV=4`、`DISARM_DELAY=5`。
- 树莓派所需 MAVLink 遥测流、失控保护和日志策略。

## 明确没有迁移的项目

以下数据属于旧机实体，不适合直接用于 QAV280：

- IMU 加速度计/陀螺仪偏移与比例。
- 罗盘 ID、方向、偏移、软铁校准和优先级。
- 气压计地面压力及旧机水平姿态 Trim。
- 遥控器各通道 MIN/MAX/TRIM。
- SERVO 输出行程、电机输出细节和电调标定。
- `MOT_THST_HOVER=0.6875` 等旧机动力学习值。
- ATC/PSC PID、机体动态参数和自动调参结果。
- 电源模块、电池电压/电流比例参数。
- 新机尚未实测的测距仪离地高度和物理安装偏移。

## 当前禁止试飞的原因

最终飞控自检明确报告：

- `PreArm: RC not found`
- `PreArm: 3D Accel calibration needed`
- `PreArm: Compass not calibrated`
- `PreArm: Hardware safety switch`

这是恢复默认标定后的预期状态，也证明旧机传感器标定没有被错误复制。

## 上桨前必须完成

1. 无桨完成加速度计校准。
2. 在远离磁性物体处完成罗盘校准并核对外置/内置罗盘方向。
3. 给接收机供电并重新校准遥控器；核对 CH5、CH6、CH7、CH8 的低/高位。
4. 按新机安装实测 `RNGFND1_GNDCLR`；当前临时保留默认 0.10 m。
5. 配置并校验 QAV280 的电池与电源模块；当前未继承旧机电源标定。
6. 无桨核对电机序号、旋向、PWM 协议和失控保护。
7. 低空悬停重新学习悬停油门并调参，然后再测试静止 Tag 和移动平台降落。

## 归档

- `new_fc_before`：目标飞控原始 3.6.7 参数母本。
- `new_fc_post_firmware_pre_reset`：升级 4.7.0、清除参数前的母本。
- `new_fc_factory_defaults`：4.7.0 纯净默认参数。
- `new_fc_after_stage1`：启用动态参数组后的母本。
- `final`：最终迁移后完整参数与固件身份母本。
- `stage1_enable_interfaces.json`、`stage2_selected_migration.json`：可审计的迁移清单。
- `stage1_apply_result.json`、`stage2_apply_result.json`：逐项写入和回读结果。
