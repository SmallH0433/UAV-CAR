import math

import pytest

from air_ground_sim.protocol import (
    VELOCITY_YAWRATE_TYPE_MASK,
    clamp,
    ros_flu_to_body_ned,
    tracking_velocity_from_image,
)


def test_clamp_limits_both_sides():
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def test_clamp_rejects_reversed_interval():
    with pytest.raises(ValueError):
        clamp(0.0, 1.0, -1.0)


def test_ros_flu_to_body_ned_signs():
    result = ros_flu_to_body_ned(1.0, 2.0, 3.0, 0.4)
    assert result.forward == 1.0
    assert result.right == -2.0
    assert result.down == -3.0
    assert math.isclose(result.yaw_rate_clockwise, -0.4)


def test_velocity_mask_activates_velocity_and_yaw_rate():
    for bit in (0, 1, 2, 6, 7, 8, 10):
        assert VELOCITY_YAWRATE_TYPE_MASK & (1 << bit)
    for bit in (3, 4, 5, 11):
        assert not VELOCITY_YAWRATE_TYPE_MASK & (1 << bit)


def test_downward_camera_error_maps_to_body_motion():
    command = tracking_velocity_from_image(0.5, -0.25, 1.0, 0.05, 0.4)
    assert command.forward == 0.25
    assert command.left == -0.4


def test_tracker_deadband_stops_small_image_error():
    command = tracking_velocity_from_image(0.02, -0.03, 1.0, 0.05, 0.4)
    assert command.forward == 0.0
    assert command.left == 0.0
