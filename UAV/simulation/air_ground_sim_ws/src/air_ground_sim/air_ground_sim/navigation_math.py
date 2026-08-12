"""Dependency-free navigation helpers used by the UAV controller and tests."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from .protocol import clamp


@dataclass(frozen=True)
class PlanarVelocity:
    forward: float
    left: float


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return ROS ENU yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def world_vector_to_body(dx_world: float, dy_world: float, yaw_rad: float) -> PlanarVelocity:
    """Rotate an ENU world vector into a ROS FLU body frame."""
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return PlanarVelocity(
        forward=cosine * dx_world + sine * dy_world,
        left=-sine * dx_world + cosine * dy_world,
    )


def limit_planar_velocity(forward: float, left: float, maximum: float) -> PlanarVelocity:
    """Limit planar magnitude without changing direction."""
    if maximum <= 0.0:
        return PlanarVelocity(0.0, 0.0)
    magnitude = math.hypot(forward, left)
    if magnitude <= maximum or magnitude <= 1.0e-9:
        return PlanarVelocity(forward, left)
    scale = maximum / magnitude
    return PlanarVelocity(forward * scale, left * scale)


def goal_velocity_body(
    dx_world: float,
    dy_world: float,
    yaw_rad: float,
    gain: float,
    max_xy_mps: float,
) -> PlanarVelocity:
    """Create a bounded body-frame velocity toward an ENU goal."""
    body_error = world_vector_to_body(dx_world, dy_world, yaw_rad)
    return limit_planar_velocity(
        gain * body_error.forward,
        gain * body_error.left,
        max_xy_mps,
    )


def apply_lidar_avoidance(
    desired: PlanarVelocity,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    influence_distance_m: float,
    hard_stop_distance_m: float,
    repulsion_gain: float,
    max_xy_mps: float,
) -> PlanarVelocity:
    """Add bounded planar repulsion and remove motion into very close obstacles.

    Laser angles follow ROS conventions: zero is forward and positive is left.
    Invalid, zero and infinite measurements are ignored.
    """
    if influence_distance_m <= 0.0 or not ranges:
        return limit_planar_velocity(desired.forward, desired.left, max_xy_mps)

    repulsion_forward = 0.0
    repulsion_left = 0.0
    active_samples = 0
    closest_range = math.inf
    closest_angle = 0.0

    for index, measured_range in enumerate(ranges):
        if not math.isfinite(measured_range) or measured_range <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        if measured_range < closest_range:
            closest_range = measured_range
            closest_angle = angle
        if measured_range >= influence_distance_m:
            continue
        strength = (influence_distance_m - measured_range) / influence_distance_m
        repulsion_forward -= math.cos(angle) * strength
        repulsion_left -= math.sin(angle) * strength
        active_samples += 1

    forward = desired.forward
    left = desired.left
    if active_samples:
        forward += repulsion_gain * repulsion_forward / active_samples
        left += repulsion_gain * repulsion_left / active_samples

    if 0.0 < closest_range < hard_stop_distance_m:
        obstacle_forward = math.cos(closest_angle)
        obstacle_left = math.sin(closest_angle)
        motion_toward_obstacle = forward * obstacle_forward + left * obstacle_left
        if motion_toward_obstacle > 0.0:
            forward -= motion_toward_obstacle * obstacle_forward
            left -= motion_toward_obstacle * obstacle_left

    return limit_planar_velocity(forward, left, max_xy_mps)


def minimum_valid_range(ranges: Iterable[float]) -> float:
    valid = [value for value in ranges if math.isfinite(value) and value > 0.0]
    return min(valid) if valid else math.inf


def vertical_goal_velocity(error_z: float, gain: float, max_z_mps: float) -> float:
    return clamp(gain * error_z, -max_z_mps, max_z_mps)
