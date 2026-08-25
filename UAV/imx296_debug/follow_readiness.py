#!/usr/bin/env python3
"""Pure readiness checks for the Raspberry Pi AprilTag monitor.

This module is deliberately transport-free.  It cannot send MAVLink messages.
"""

from __future__ import annotations

from dataclasses import dataclass


EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_POS_HORIZ_REL = 8
EKF_PRED_POS_HORIZ_REL = 256
REQUIRED_EKF_FLAGS = (
    EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_REL | EKF_PRED_POS_HORIZ_REL
)


@dataclass(frozen=True)
class ReadinessInputs:
    heartbeat_age_s: float | None
    armed: bool | None
    mode: str | None
    rc7_pwm: int | None
    rc_age_s: float | None
    ekf_flags: int | None
    ekf_age_s: float | None
    battery_voltage_v: float | None
    battery_remaining_pct: int | None
    battery_age_s: float | None
    range_m: float | None
    range_age_s: float | None
    flow_quality: int | None
    flow_age_s: float | None
    origin_valid: bool
    target_acquired: bool
    target_age_s: float | None
    camera_ok: bool


@dataclass(frozen=True)
class ReadinessResult:
    ready_for_follow_request: bool
    blockers: tuple[str, ...]
    warnings: tuple[str, ...]


def evaluate_readiness(
    inputs: ReadinessInputs,
    *,
    minimum_voltage_v: float = 21.6,
    minimum_remaining_pct: int = 20,
    battery_telemetry_required: bool = True,
    minimum_range_m: float = 0.55,
    maximum_range_m: float = 0.85,
    minimum_flow_quality: int = 80,
    telemetry_timeout_s: float = 0.75,
    target_timeout_s: float = 0.25,
    allowed_modes: tuple[str, ...] = ("LOITER", "GUIDED"),
) -> ReadinessResult:
    blockers: list[str] = []
    warnings: list[str] = []

    if not inputs.camera_ok:
        blockers.append("CAMERA_UNAVAILABLE")
    if inputs.heartbeat_age_s is None or inputs.heartbeat_age_s > telemetry_timeout_s:
        blockers.append("FLIGHT_CONTROLLER_HEARTBEAT_STALE")
    if inputs.armed is None:
        blockers.append("ARM_STATE_UNKNOWN")
    approved_modes = {mode.upper() for mode in allowed_modes}
    if not approved_modes:
        raise ValueError("allowed_modes must not be empty")
    if inputs.mode is None:
        blockers.append("FLIGHT_MODE_UNKNOWN")
    elif inputs.mode.upper() not in approved_modes:
        blockers.append("ENTRY_MODE_NOT_APPROVED")
    if inputs.rc7_pwm is None or inputs.rc_age_s is None or inputs.rc_age_s > telemetry_timeout_s:
        blockers.append("CH7_STALE")

    if inputs.ekf_flags is None or inputs.ekf_age_s is None or inputs.ekf_age_s > telemetry_timeout_s:
        blockers.append("EKF_STATUS_STALE")
    elif inputs.ekf_flags & REQUIRED_EKF_FLAGS != REQUIRED_EKF_FLAGS:
        blockers.append("EKF_RELATIVE_POSITION_INVALID")

    if not inputs.origin_valid:
        blockers.append("EKF_GLOBAL_ORIGIN_MISSING")

    if battery_telemetry_required:
        if (
            inputs.battery_voltage_v is None
            or inputs.battery_age_s is None
            or inputs.battery_age_s > telemetry_timeout_s
        ):
            blockers.append("BATTERY_TELEMETRY_STALE")
        elif inputs.battery_voltage_v < minimum_voltage_v:
            blockers.append("BATTERY_VOLTAGE_LOW")
        if (
            inputs.battery_remaining_pct is not None
            and inputs.battery_remaining_pct < minimum_remaining_pct
        ):
            blockers.append("BATTERY_REMAINING_LOW")
    else:
        warnings.append("BATTERY_CHECK_DISABLED")

    if inputs.range_m is None or inputs.range_age_s is None or inputs.range_age_s > telemetry_timeout_s:
        blockers.append("RANGEFINDER_STALE")
    elif not minimum_range_m <= inputs.range_m <= maximum_range_m:
        blockers.append("HEIGHT_OUTSIDE_FOLLOW_GATE")

    if inputs.flow_quality is None or inputs.flow_age_s is None or inputs.flow_age_s > telemetry_timeout_s:
        blockers.append("OPTICAL_FLOW_STALE")
    elif inputs.flow_quality < minimum_flow_quality:
        blockers.append("OPTICAL_FLOW_QUALITY_LOW")

    if not inputs.target_acquired:
        blockers.append("APRILTAG_NOT_ACQUIRED")
    elif inputs.target_age_s is None or inputs.target_age_s > target_timeout_s:
        blockers.append("APRILTAG_STALE")

    if inputs.armed is False:
        warnings.append("DISARMED_MONITORING_ONLY")
    if inputs.rc7_pwm is not None and inputs.rc7_pwm < 1800:
        warnings.append("CH7_FOLLOW_DISABLED")

    return ReadinessResult(not blockers, tuple(blockers), tuple(warnings))
