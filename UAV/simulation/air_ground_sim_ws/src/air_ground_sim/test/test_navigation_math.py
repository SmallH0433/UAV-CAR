import math

from air_ground_sim.navigation_math import (
    PlanarVelocity,
    apply_lidar_avoidance,
    goal_velocity_body,
    limit_planar_velocity,
    wrap_angle,
)


def test_world_goal_is_rotated_into_body_frame():
    velocity = goal_velocity_body(0.0, 2.0, math.pi / 2.0, 1.0, 3.0)
    assert math.isclose(velocity.forward, 2.0, abs_tol=1.0e-9)
    assert math.isclose(velocity.left, 0.0, abs_tol=1.0e-9)


def test_planar_limit_preserves_direction():
    velocity = limit_planar_velocity(3.0, 4.0, 1.0)
    assert math.isclose(velocity.forward, 0.6)
    assert math.isclose(velocity.left, 0.8)


def test_close_front_obstacle_removes_forward_motion():
    ranges = [math.inf] * 5
    ranges[2] = 0.5
    velocity = apply_lidar_avoidance(
        PlanarVelocity(1.0, 0.0),
        ranges,
        -0.2,
        0.1,
        3.0,
        1.0,
        1.2,
        1.0,
    )
    assert velocity.forward <= 0.0


def test_wrap_angle_uses_shortest_rotation():
    assert math.isclose(wrap_angle(3.0 * math.pi), -math.pi)
