import math

import pytest

from air_ground_sim.chassis_adapters import (
    AckermannAdapter,
    DiffDriveAdapter,
    FourWheelSteeringAdapter,
    make_chassis_adapter,
)


def test_diff_drive_keeps_in_place_rotation():
    command = DiffDriveAdapter(1.0, 1.5).adapt(0.0, 0.8)
    assert command.linear_mps == 0.0
    assert command.angular_rps == 0.8


def test_ackermann_rejects_in_place_rotation():
    adapter = AckermannAdapter(1.0, 2.0, 0.65, 0.45)
    command = adapter.adapt(0.0, 0.8)
    assert command.linear_mps == 0.0
    assert command.angular_rps == 0.0
    assert command.reason == "in_place_turn_rejected"


def test_ackermann_preserves_low_speed_straight_motion():
    adapter = AckermannAdapter(1.0, 2.0, 0.65, 0.45, 0.03)
    command = adapter.adapt(0.02, 0.0)
    assert command.linear_mps == 0.02
    assert command.angular_rps == 0.0


def test_ackermann_starts_straight_before_low_speed_turn():
    adapter = AckermannAdapter(1.0, 2.0, 0.65, 0.45, 0.03)
    command = adapter.adapt(0.02, 0.4)
    assert command.linear_mps == 0.02
    assert command.angular_rps == 0.0
    assert command.reason == "turn_suppressed_below_min_speed"


def test_ackermann_clamps_curvature_to_steering_limit():
    adapter = AckermannAdapter(1.0, 2.0, 0.65, 0.45)
    command = adapter.adapt(1.0, 2.0)
    assert math.isclose(command.steering_angle_rad, 0.45)
    assert math.isclose(command.angular_rps, math.tan(0.45) / 0.65)
    assert command.saturated


def test_ackermann_reverse_uses_consistent_curvature_sign():
    adapter = AckermannAdapter(1.0, 2.0, 0.65, 0.45)
    command = adapter.adapt(-0.5, 0.2)
    assert command.curvature_per_m < 0.0
    assert command.steering_angle_rad < 0.0
    assert command.angular_rps > 0.0


def test_four_wheel_steering_exposes_twice_the_curvature():
    ackermann = AckermannAdapter(1.0, 3.0, 0.65, 0.45)
    four_wheel = FourWheelSteeringAdapter(1.0, 3.0, 0.65, 0.45)
    ackermann_command = ackermann.adapt(1.0, 3.0)
    four_wheel_command = four_wheel.adapt(1.0, 3.0)
    assert math.isclose(
        four_wheel_command.angular_rps, 2.0 * ackermann_command.angular_rps
    )


def test_adapter_factory_rejects_unknown_type():
    with pytest.raises(ValueError):
        make_chassis_adapter("skid_magic", 1.0, 1.0, 0.65, 0.45)
