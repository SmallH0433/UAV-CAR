"""sim_motor_bridge 阿克曼运动学正逆换算单测（纯函数，不需要 ROS 图）。"""

import math

import pytest

from car_nodes.sim_motor_bridge import ackermann_to_twist, twist_to_ackermann

R = 0.076       # 轮半径 m
TRACK = 0.32    # 后轮距 m
WHEELBASE = 0.31  # 轴距 m
MAX_STEER = 0.5   # 最大转向角 rad


def test_straight_line_zero_steering():
    # 直行 v=0.38, w=0 → δ=0，左右后轮同速 5 rad/s
    speeds, steering = twist_to_ackermann(0.38, 0.0, R, WHEELBASE, TRACK, MAX_STEER)
    assert steering == pytest.approx(0.0)
    assert speeds[0] == pytest.approx(5.0)
    assert speeds[1] == pytest.approx(5.0)
    v, w = ackermann_to_twist(speeds, steering, R, WHEELBASE)
    assert v == pytest.approx(0.38)
    assert w == pytest.approx(0.0)


def test_turn_roundtrip():
    # 定半径转弯：v=0.4, w=0.6 → δ=atan(w·L/v)，往返一致
    v_in, w_in = 0.4, 0.6
    speeds, steering = twist_to_ackermann(
        v_in, w_in, R, WHEELBASE, TRACK, MAX_STEER)
    assert steering == pytest.approx(math.atan2(w_in * WHEELBASE, v_in))
    # 左转弯（w>0）时右后轮比左后轮快
    assert speeds[1] > speeds[0]
    v, w = ackermann_to_twist(speeds, steering, R, WHEELBASE)
    assert v == pytest.approx(v_in)
    assert w == pytest.approx(w_in)


def test_steering_clamped_and_consistent():
    # 大角速度指令使 δ 限幅：后轮差速应与限幅后的实际转弯半径一致
    v_in, w_in = 0.5, 100.0
    speeds, steering = twist_to_ackermann(
        v_in, w_in, R, WHEELBASE, TRACK, MAX_STEER)
    assert steering == pytest.approx(MAX_STEER)
    v, w = ackermann_to_twist(speeds, steering, R, WHEELBASE)
    assert v == pytest.approx(v_in)
    assert w == pytest.approx(v_in * math.tan(MAX_STEER) / WHEELBASE)


def test_reverse_turn_roundtrip():
    # 倒车转弯：v<0 时 δ 必须与 v 异号才能得到正确的 ω 方向
    # （atan2 会错误地把 δ 映射到限幅值另一侧，这里回归验证 atan 方案）
    v_in, w_in = -0.3, 0.5
    speeds, steering = twist_to_ackermann(
        v_in, w_in, R, WHEELBASE, TRACK, MAX_STEER)
    assert steering < 0.0  # 倒车左转（ω>0）需要负转向角
    v, w = ackermann_to_twist(speeds, steering, R, WHEELBASE)
    assert v == pytest.approx(v_in)
    assert w == pytest.approx(w_in)


def test_standstill_no_wheel_spin():
    # v=0, w≠0：阿克曼不能原地自旋——后轮速必须为 0，δ 按 w 方向打满（预打方向）
    speeds, steering = twist_to_ackermann(0.0, 1.0, R, WHEELBASE, TRACK, MAX_STEER)
    assert speeds == [0.0, 0.0]
    assert steering == pytest.approx(MAX_STEER)
    speeds, steering = twist_to_ackermann(0.0, -1.0, R, WHEELBASE, TRACK, MAX_STEER)
    assert speeds == [0.0, 0.0]
    assert steering == pytest.approx(-MAX_STEER)
    # 全零指令 → 全零输出
    speeds, steering = twist_to_ackermann(0.0, 0.0, R, WHEELBASE, TRACK, MAX_STEER)
    assert speeds == [0.0, 0.0]
    assert steering == pytest.approx(0.0)
