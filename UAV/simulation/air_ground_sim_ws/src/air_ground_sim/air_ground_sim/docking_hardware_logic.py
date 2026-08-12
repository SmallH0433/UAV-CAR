"""Pure interlocks for a dual-channel physical docking mechanism."""

from __future__ import annotations

from typing import Optional


STATIONARY_LATCH_STATES = frozenset({"LATCH_AT_START", "LATCH_STOPPED"})
RELEASE_STATES = frozenset(
    {"RELEASE_REMOTE_DOCK", "RELEASE_FOR_TRANSIT", "RELEASE_FOR_FOLLOW"}
)


def dual_channel_state(channel_a: bool, channel_b: bool) -> Optional[bool]:
    """Return agreed state, or None when redundant channels disagree."""

    return bool(channel_a) if bool(channel_a) == bool(channel_b) else None


def operation_command_decision(
    *,
    commanded_locked: Optional[bool],
    requested_locked: bool,
    feedback_locked: Optional[bool],
    operation_started_s: float,
    now_s: float,
) -> tuple[bool, float]:
    """Decide whether to publish and retain the first attempt's timeout origin.

    Mission orchestration may repeat an attach/detach request while waiting for
    hardware. A repeated request must never extend the actuator timeout.
    """

    requested = bool(requested_locked)
    if feedback_locked is not None and bool(feedback_locked) == requested:
        return False, 0.0
    if (
        commanded_locked is not None
        and bool(commanded_locked) == requested
        and float(operation_started_s) > 0.0
    ):
        return True, float(operation_started_s)
    return True, float(now_s)


def feedback_safe_for_enable(
    *, contact_state: Optional[bool], locked_state: Optional[bool]
) -> bool:
    """Require agreed channels and forbid a locked-without-contact state."""

    return (
        contact_state is not None
        and locked_state is not None
        and not (bool(locked_state) and not bool(contact_state))
    )


def physical_attach_authorized(
    *,
    mission_state: str,
    contact_a: bool,
    contact_b: bool,
    armed: bool,
    landed: Optional[bool],
    autopilot_mode: str,
    altitude_m: float,
    ugv_speed_mps: float,
    ugv_yaw_rate_rps: float,
    stationary_speed_limit_mps: float,
    moving_speed_limit_mps: float,
    moving_yaw_rate_limit_rps: float,
    moving_capture_max_altitude_m: float,
) -> bool:
    """Independently guard a physical lock command."""

    if not (bool(contact_a) and bool(contact_b)):
        return False
    state = str(mission_state).upper()
    speed = abs(float(ugv_speed_mps))
    if state in STATIONARY_LATCH_STATES:
        return (
            speed <= max(0.0, float(stationary_speed_limit_mps))
            and landed is True
            and not bool(armed)
        )
    if state != "LATCH_MOVING":
        return False
    if speed > max(0.0, float(moving_speed_limit_mps)):
        return False
    if abs(float(ugv_yaw_rate_rps)) > max(
        0.0, float(moving_yaw_rate_limit_rps)
    ):
        return False
    if landed is True and not bool(armed):
        return True
    return (
        bool(armed)
        and str(autopilot_mode).upper() == "LAND"
        and 0.0 <= float(altitude_m)
        <= max(0.0, float(moving_capture_max_altitude_m))
    )


def physical_release_authorized(
    *,
    mission_state: str,
    armed: bool,
    landed: Optional[bool],
    ugv_speed_mps: float,
    stationary_speed_limit_mps: float,
) -> bool:
    """Release only in explicit states with both vehicles in a safe state."""

    return (
        str(mission_state).upper() in RELEASE_STATES
        and landed is True
        and not bool(armed)
        and abs(float(ugv_speed_mps))
        <= max(0.0, float(stationary_speed_limit_mps))
    )
