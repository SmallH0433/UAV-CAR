import math

from air_ground_sim.docking_math import (
    body_feedforward_from_ugv,
    inside_capture_envelope,
    propagate_map_pose_with_odometry,
    visual_centering_velocity,
)


def test_visual_servo_matches_downward_camera_flu_mapping():
    forward, left = visual_centering_velocity(0.2, -0.3, 1.0, 1.0, 0.01)
    assert forward > 0.0
    assert left < 0.0


def test_moving_platform_feedforward_rotates_into_uav_frame():
    forward, left = body_feedforward_from_ugv(1.0, math.pi / 2.0, 0.0)
    assert abs(forward) < 1e-6
    assert abs(left - 1.0) < 1e-6


def test_sparse_map_fix_is_propagated_with_fresh_odometry():
    x, y, yaw = propagate_map_pose_with_odometry(
        map_anchor=(-9.0, -6.0, math.pi / 2.0),
        odom_anchor=(1.0, 2.0, 0.0),
        odom_current=(3.0, 2.0, math.pi / 4.0),
    )
    assert abs(x + 9.0) < 1e-6
    assert abs(y + 4.0) < 1e-6
    assert abs(yaw - 3.0 * math.pi / 4.0) < 1e-6


def test_capture_envelope_requires_all_independent_guards():
    assert inside_capture_envelope(True, 0.03, 0.04, 0.70, 0.72, 0.1, 2000, 1400)
    assert not inside_capture_envelope(False, 0.0, 0.0, 0.6, 0.72, 0.1, 3000, 1400)
    assert not inside_capture_envelope(True, 0.2, 0.0, 0.6, 0.72, 0.1, 3000, 1400)


def test_capture_envelope_accepts_healthy_deck_relative_range():
    assert inside_capture_envelope(
        True,
        0.02,
        -0.01,
        0.90,
        0.60,
        0.1,
        3000,
        1400,
        deck_range_m=0.48,
        maximum_deck_range_m=0.55,
    )
    assert not inside_capture_envelope(
        True,
        0.02,
        -0.01,
        0.90,
        0.60,
        0.1,
        3000,
        1400,
        deck_range_m=None,
        maximum_deck_range_m=0.55,
    )
