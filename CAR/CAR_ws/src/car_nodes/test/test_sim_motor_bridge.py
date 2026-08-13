"""sim_motor_bridge 运动学正逆换算单测（纯函数，不需要 ROS 图）。"""

import pytest

from car_nodes.sim_motor_bridge import twist_to_wheel_speeds, wheel_speeds_to_twist

R = 0.076      # 轮半径 m
TRACK = 0.32   # 轮距 m


def test_straight_line_wheels_map_to_pure_linear():
    # 四轮同速 5 rad/s → v = 5 * 0.076，w = 0
    v, w = wheel_speeds_to_twist([5.0, 5.0, 5.0, 5.0], R, TRACK)
    assert v == pytest.approx(0.38)
    assert w == pytest.approx(0.0)


def test_opposite_sides_map_to_pure_rotation():
    # 左 +2、右 -2 rad/s → v = 0，w = (右 - 左) * r / track，逆时针为负
    v, w = wheel_speeds_to_twist([2.0, -2.0, 2.0, -2.0], R, TRACK)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx((-2.0 - 2.0) * R / TRACK)


def test_twist_to_wheels_roundtrip():
    v, w = 0.4, 0.6
    speeds = twist_to_wheel_speeds(v, w, R, TRACK)
    assert len(speeds) == 4
    # 左侧两轮相等、右侧两轮相等（与 chassis_controller 约定一致）
    assert speeds[0] == pytest.approx(speeds[2])
    assert speeds[1] == pytest.approx(speeds[3])
    v2, w2 = wheel_speeds_to_twist(speeds, R, TRACK)
    assert v2 == pytest.approx(v)
    assert w2 == pytest.approx(w)
