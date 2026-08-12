"""Pure control helpers for landing on stationary and moving ground vehicles."""

import math
from typing import Optional, Tuple

from .protocol import clamp


def visual_centering_velocity(
    error_x: float, error_y: float, gain: float, maximum: float, deadband: float
) -> Tuple[float, float]:
    """Map downward image error to FLU velocity (forward, left)."""
    horizontal = 0.0 if abs(error_x) <= deadband else float(error_x)
    vertical = 0.0 if abs(error_y) <= deadband else float(error_y)
    return (
        clamp(-float(gain) * vertical, -float(maximum), float(maximum)),
        clamp(-float(gain) * horizontal, -float(maximum), float(maximum)),
    )


def body_feedforward_from_ugv(
    ugv_forward_mps: float, ugv_yaw_rad: float, uav_yaw_rad: float
) -> Tuple[float, float]:
    """Rotate UGV longitudinal velocity into the UAV FLU frame."""
    world_x = float(ugv_forward_mps) * math.cos(float(ugv_yaw_rad))
    world_y = float(ugv_forward_mps) * math.sin(float(ugv_yaw_rad))
    cosine = math.cos(float(uav_yaw_rad))
    sine = math.sin(float(uav_yaw_rad))
    return (
        cosine * world_x + sine * world_y,
        -sine * world_x + cosine * world_y,
    )


def propagate_map_pose_with_odometry(
    map_anchor: Tuple[float, float, float],
    odom_anchor: Tuple[float, float, float],
    odom_current: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Propagate a sparse map localization fix with fresh local odometry.

    AMCL intentionally does not republish while a robot is stationary. A
    docking controller therefore treats the last map pose as an anchor and
    applies the SE(2) odometry delta, exactly as the map->odom TF chain does.
    """
    map_x, map_y, map_yaw = (float(value) for value in map_anchor)
    odom_x0, odom_y0, odom_yaw0 = (float(value) for value in odom_anchor)
    odom_x, odom_y, odom_yaw = (float(value) for value in odom_current)

    delta_world_x = odom_x - odom_x0
    delta_world_y = odom_y - odom_y0
    cosine_anchor = math.cos(odom_yaw0)
    sine_anchor = math.sin(odom_yaw0)
    delta_local_x = cosine_anchor * delta_world_x + sine_anchor * delta_world_y
    delta_local_y = -sine_anchor * delta_world_x + cosine_anchor * delta_world_y

    cosine_map = math.cos(map_yaw)
    sine_map = math.sin(map_yaw)
    propagated_x = map_x + cosine_map * delta_local_x - sine_map * delta_local_y
    propagated_y = map_y + sine_map * delta_local_x + cosine_map * delta_local_y
    propagated_yaw = map_yaw + (odom_yaw - odom_yaw0)
    propagated_yaw = math.atan2(math.sin(propagated_yaw), math.cos(propagated_yaw))
    return (propagated_x, propagated_y, propagated_yaw)


def inside_capture_envelope(
    tag_visible: bool,
    error_x: float,
    error_y: float,
    altitude_m: float,
    capture_altitude_m: float,
    maximum_normalized_error: float,
    tag_area_px: float,
    minimum_tag_area_px: float,
    deck_range_m: Optional[float] = None,
    maximum_deck_range_m: Optional[float] = None,
) -> bool:
    altitude = float(altitude_m)
    altitude_guard = (
        math.isfinite(altitude)
        and 0.0 <= altitude <= float(capture_altitude_m)
    )
    deck_range_guard = False
    if deck_range_m is not None and maximum_deck_range_m is not None:
        measured_range = float(deck_range_m)
        deck_range_guard = (
            math.isfinite(measured_range)
            and 0.0 <= measured_range <= float(maximum_deck_range_m)
        )
    return bool(tag_visible) and (
        math.hypot(float(error_x), float(error_y)) <= float(maximum_normalized_error)
        and (altitude_guard or deck_range_guard)
        and float(tag_area_px) >= float(minimum_tag_area_px)
    )
