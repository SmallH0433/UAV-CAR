#!/usr/bin/env python3
"""Encode bounded GUIDED velocity setpoints without opening a MAVLink link."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from pymavlink.dialects.v20 import common


MAV_FRAME_LOCAL_NED = 1
# Ignore position, acceleration/force and yaw; use vx/vy/vz and yaw_rate=0.
VELOCITY_AND_YAW_RATE_TYPE_MASK = 1479


@dataclass(frozen=True)
class GuidedVelocitySetpoint:
    time_boot_ms: int
    vx_mps: float
    vy_mps: float
    vz_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    frame: int = MAV_FRAME_LOCAL_NED
    type_mask: int = VELOCITY_AND_YAW_RATE_TYPE_MASK

    def as_dict(self) -> dict:
        return asdict(self)


def validate_setpoint(setpoint: GuidedVelocitySetpoint, max_speed_mps: float) -> None:
    values = (
        setpoint.vx_mps,
        setpoint.vy_mps,
        setpoint.vz_mps,
        setpoint.yaw_rate_rad_s,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("setpoint contains a non-finite value")
    if math.sqrt(
        setpoint.vx_mps * setpoint.vx_mps
        + setpoint.vy_mps * setpoint.vy_mps
        + setpoint.vz_mps * setpoint.vz_mps
    ) > max_speed_mps + 1e-9:
        raise ValueError("setpoint exceeds configured speed limit")
    if setpoint.frame != MAV_FRAME_LOCAL_NED:
        raise ValueError("first version only permits MAV_FRAME_LOCAL_NED")
    if setpoint.type_mask != VELOCITY_AND_YAW_RATE_TYPE_MASK:
        raise ValueError("unexpected position-target type mask")


def make_message(
    setpoint: GuidedVelocitySetpoint,
    *,
    target_system: int = 1,
    target_component: int = 1,
    max_speed_mps: float = 0.2,
) -> common.MAVLink_set_position_target_local_ned_message:
    validate_setpoint(setpoint, max_speed_mps)
    return common.MAVLink_set_position_target_local_ned_message(
        setpoint.time_boot_ms,
        target_system,
        target_component,
        setpoint.frame,
        setpoint.type_mask,
        0.0,
        0.0,
        0.0,
        setpoint.vx_mps,
        setpoint.vy_mps,
        setpoint.vz_mps,
        0.0,
        0.0,
        0.0,
        0.0,
        setpoint.yaw_rate_rad_s,
    )


def pack_message(
    setpoint: GuidedVelocitySetpoint,
    *,
    source_system: int = 191,
    source_component: int = 191,
    max_speed_mps: float = 0.2,
) -> bytes:
    mav = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
    return make_message(setpoint, max_speed_mps=max_speed_mps).pack(mav)
