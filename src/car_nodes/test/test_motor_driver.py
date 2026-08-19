"""motor_driver 下行 (vx, vz) 换算单测（纯函数，不需要 ROS 图）。

回归目标：避障倒车脱困（v<0, ω≠0）时，下发给 STM32 的 vz 必须翻号，
否则固件按前进假设解算舵机角，前进/倒车共用同一舵机角，
小车沿同一弧线前后往复（实机观测到的 bug）。
"""

import math

import pytest

from car_nodes.motor_driver import ackermann_to_firmware_velocity
from car_nodes.sim_motor_bridge import twist_to_ackermann

R = 0.076       # 轮半径 m
TRACK = 0.32    # 后轮距 m
WHEELBASE = 0.31  # 轴距 m
MAX_STEER = 0.5   # 最大转向角 rad


def _firmware_velocities(v, w):
    """(v, w) → 底盘逆运动学 → 下行帧 (vx, vz)，走真机完整链路。"""
    speeds, steering = twist_to_ackermann(v, w, R, WHEELBASE, TRACK, MAX_STEER)
    return ackermann_to_firmware_velocity(speeds, steering, R, WHEELBASE)


def _firmware_steering(vx, vz):
    """固件按前进假设由 (vx, vz) 解算舵机角：δ = atan(vz·L/|vx|)。"""
    return math.atan(vz * WHEELBASE / abs(vx))


def test_forward_vz_passthrough():
    # 前进指令不受影响：vx/vz 原样下发
    vx, vz = _firmware_velocities(0.4, 0.3)
    assert vx == pytest.approx(0.4)
    assert vz == pytest.approx(0.3)


def test_reverse_flips_vz_sign():
    # 倒车脱困 v=-0.2, ω=+0.3（期望车头向左甩）：vz 必须翻为负，
    # 固件才会打出与前进相反的舵机角
    vx, vz = _firmware_velocities(-0.2, 0.3)
    assert vx == pytest.approx(-0.2)
    assert vz < 0.0
    # 固件解算出的实际角速度应与期望 ω 一致
    steering_fw = _firmware_steering(vx, vz)
    w_actual = vx * math.tan(steering_fw) / WHEELBASE
    assert w_actual == pytest.approx(0.3)


def test_reverse_steering_physically_flips():
    # 核心回归：同一 ω 指令，前进 vs 倒车时固机舵机角必须反向，
    # 否则前进/倒车切换转向轮不动，车沿同一弧线往复
    vx_fwd, vz_fwd = _firmware_velocities(0.2, 0.3)
    vx_rev, vz_rev = _firmware_velocities(-0.2, 0.3)
    steering_fwd = _firmware_steering(vx_fwd, vz_fwd)
    steering_rev = _firmware_steering(vx_rev, vz_rev)
    assert steering_fwd > 0.0
    assert steering_rev == pytest.approx(-steering_fwd)


def test_standstill_zero_vz():
    # v≈0 时 vz 强制为 0（阿克曼不能原地自旋）
    vx, vz = _firmware_velocities(0.0, 1.0)
    assert vx == pytest.approx(0.0)
    assert vz == pytest.approx(0.0)
