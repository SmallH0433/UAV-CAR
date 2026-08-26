"""Frame helpers for messages that MAVROS converts before MAVLink output."""

from __future__ import annotations

import math


def body_frd_pose_to_ros_baselink(
    position_body_frd: tuple[float, float, float],
    orientation_body_frd_wxyz: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Represent a BODY_FRD pose in ROS base_link FLU coordinates.

    MAVROS' landing-target raw callback applies its base_link-to-aircraft
    conversion before encoding ``LANDING_TARGET``.  Therefore publishers must
    provide FLU values even when the message's MAV_FRAME field is BODY_FRD.
    """

    if len(position_body_frd) != 3 or len(orientation_body_frd_wxyz) != 4:
        raise ValueError("BODY_FRD pose requires xyz and wxyz")
    values = tuple(
        float(value)
        for value in (*position_body_frd, *orientation_body_frd_wxyz)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("BODY_FRD pose must be finite")
    x, y, z = position_body_frd
    qw, qx, qy, qz = orientation_body_frd_wxyz
    # Vector: FLU -> FRD inside MAVROS is (x, -y, -z), and is self-inverse.
    position_ros_flu = (float(x), -float(y), -float(z))
    # MAVROS computes q_frd = q_ros * q_roll_pi.  Multiplying the desired
    # q_frd by q_roll_pi gives an equivalent input (the output differs only by
    # the quaternion's irrelevant global sign).
    orientation_ros_flu = (-float(qx), float(qw), float(qz), -float(qy))
    return position_ros_flu, orientation_ros_flu
