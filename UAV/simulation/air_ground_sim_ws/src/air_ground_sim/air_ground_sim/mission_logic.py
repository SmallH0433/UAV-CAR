"""Deterministic state-transition rules for the air-ground mission."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping, Optional


class MissionState(str, Enum):
    IDLE = "IDLE"
    RELEASE_REMOTE_DOCK = "RELEASE_REMOTE_DOCK"
    WAIT_AUTOPILOT = "WAIT_AUTOPILOT"
    ARM_INITIAL = "ARM_INITIAL"
    TAKEOFF_INITIAL = "TAKEOFF_INITIAL"
    NAVIGATE_TO_START_DOCK = "NAVIGATE_TO_START_DOCK"
    DOCK_AT_START = "DOCK_AT_START"
    LATCH_AT_START = "LATCH_AT_START"
    DWELL_AT_START = "DWELL_AT_START"
    RELEASE_FOR_TRANSIT = "RELEASE_FOR_TRANSIT"
    ARM_FOR_TRANSIT = "ARM_FOR_TRANSIT"
    TAKEOFF_FOR_TRANSIT = "TAKEOFF_FOR_TRANSIT"
    PARALLEL_TRANSIT = "PARALLEL_TRANSIT"
    DOCK_STOPPED = "DOCK_STOPPED"
    LATCH_STOPPED = "LATCH_STOPPED"
    DWELL_STOPPED = "DWELL_STOPPED"
    RELEASE_FOR_FOLLOW = "RELEASE_FOR_FOLLOW"
    ARM_FOR_FOLLOW = "ARM_FOR_FOLLOW"
    TAKEOFF_FOR_FOLLOW = "TAKEOFF_FOR_FOLLOW"
    FOLLOW_MOVING_UGV = "FOLLOW_MOVING_UGV"
    DOCK_MOVING = "DOCK_MOVING"
    LATCH_MOVING = "LATCH_MOVING"
    RIDE_AND_DECELERATE = "RIDE_AND_DECELERATE"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    FAULT = "FAULT"


UGV_MOTION_STATES = frozenset(
    {
        MissionState.PARALLEL_TRANSIT,
        MissionState.FOLLOW_MOVING_UGV,
        MissionState.DOCK_MOVING,
        MissionState.LATCH_MOVING,
        MissionState.RIDE_AND_DECELERATE,
    }
)


def mission_state_allows_ugv_motion(state: MissionState) -> bool:
    """Keep the ground motion gate closed outside explicit drive states."""

    return state in UGV_MOTION_STATES


def mission_plan_is_commissioned(
    *, simulation_lifecycle: bool, validated: bool, plan_id: str
) -> bool:
    """Reject real missions unless a site-specific plan was commissioned."""

    if simulation_lifecycle:
        return True
    normalized = str(plan_id).strip()
    return (
        bool(validated)
        and len(normalized) >= 8
        and normalized.upper() not in {"UNCOMMISSIONED", "CHANGE_ME", "DEFAULT"}
    )


def parse_detachable_joint_state(value: str) -> Optional[bool]:
    """Map Gazebo DetachableJoint StringMsg state to detached semantics."""
    state = str(value).strip().lower()
    if state == "detached":
        return True
    if state == "attached":
        return False
    return None


def dock_attach_authorized(
    state: MissionState,
    *,
    armed: bool,
    landed: Optional[bool],
    autopilot_mode: str,
    altitude_m: float,
    moving_capture_max_altitude_m: float = 0.50,
) -> bool:
    """Authorize mechanical capture only in an explicit docking state.

    Stationary landings must be positively landed and disarmed before the
    latch closes. A moving-platform landing may close the capture mechanism
    during the final LAND phase so that the platform cannot move out from
    under the aircraft; the supervisor independently time-bounds this armed
    transition until normal autopilot disarm is confirmed.
    """

    if state in (MissionState.LATCH_AT_START, MissionState.LATCH_STOPPED):
        return landed is True and not bool(armed)
    if state != MissionState.LATCH_MOVING:
        return False
    if landed is True and not bool(armed):
        return True
    altitude_limit = max(0.0, float(moving_capture_max_altitude_m))
    return (
        bool(armed)
        and str(autopilot_mode).upper() == "LAND"
        and 0.0 <= float(altitude_m) <= altitude_limit
    )


def distance_speed_scale(
    cruise: float, final: float, remaining_m: float, slowdown_distance_m: float
) -> float:
    """Reduce speed near the goal without decaying while the robot is stalled."""

    high = max(0.0, min(1.0, float(cruise)))
    low = max(0.0, min(1.0, float(final)))
    distance = max(0.0, float(remaining_m))
    window = max(1.0e-6, float(slowdown_distance_m))
    progress = max(0.0, min(1.0, distance / window))
    return low + (high - low) * progress


def moving_deck_envelope(
    *,
    yaw_rad: Optional[float],
    yaw_rate_rps: float,
    target_yaw_rad: float,
    max_yaw_error_rad: float,
    max_yaw_rate_rps: float,
) -> bool:
    """Check that a moving deck is aligned and not executing a hard turn."""

    if yaw_rad is None:
        return False
    error = abs(
        math.atan2(
            math.sin(float(yaw_rad) - float(target_yaw_rad)),
            math.cos(float(yaw_rad) - float(target_yaw_rad)),
        )
    )
    return (
        error <= max(0.0, float(max_yaw_error_rad))
        and abs(float(yaw_rate_rps)) <= max(0.0, float(max_yaw_rate_rps))
    )


def navigation_goal_failed(status: str) -> bool:
    """Recognize terminal Nav2 failures while allowing its internal recovery."""

    normalized = str(status).strip().lower()
    return normalized == "rejected" or normalized.startswith(
        ("ended_", "send_error:", "result_error:")
    )


def transform_stamp_is_fresh(
    *, now_ns: int, stamp_ns: int, timeout_s: float
) -> bool:
    """Reject missing, stale, or pre-reset TF timestamps."""

    now = int(now_ns)
    stamp = int(stamp_ns)
    if now <= 0 or stamp <= 0:
        return False
    age_s = (now - stamp) / 1_000_000_000.0
    return -0.1 <= age_s <= max(0.0, float(timeout_s))


def update_sustained_since(
    condition: bool, since_s: float, now_s: float
) -> float:
    """Track when a continuously true safety condition first became true."""

    if not condition:
        return 0.0
    return float(since_s) if float(since_s) > 0.0 else float(now_s)


def sustained_for(
    condition: bool,
    since_s: float,
    now_s: float,
    hold_s: float,
) -> bool:
    """Reject one-sample readiness spikes before a hazardous transition."""

    return bool(
        condition
        and float(since_s) > 0.0
        and float(now_s) - float(since_s) >= max(float(hold_s), 0.0)
    )


def progress_watchdog_step(
    *,
    position_xy: Optional[tuple[float, float]],
    anchor_xy: Optional[tuple[float, float]],
    anchor_since_s: Optional[float],
    now_s: float,
    minimum_progress_m: float,
    timeout_s: float,
) -> tuple[Optional[tuple[float, float]], float, bool]:
    """Advance a distance-based progress watchdog.

    A long route may legitimately need a generous state timeout. This watchdog
    remains independent of that budget and detects a controller which is still
    reporting ``executing`` but has stopped making measurable map-frame
    progress. Missing pose data consumes the same bounded progress window.
    """

    now = float(now_s)
    since = now if anchor_since_s is None else float(anchor_since_s)
    anchor = anchor_xy
    if position_xy is not None:
        position = (float(position_xy[0]), float(position_xy[1]))
        if anchor is None or math.hypot(
            position[0] - float(anchor[0]), position[1] - float(anchor[1])
        ) >= max(float(minimum_progress_m), 0.0):
            return position, now, False
    return anchor, since, now - since >= max(float(timeout_s), 0.0)


def mission_terminal_reset_is_safe(
    state: MissionState,
    *,
    armed: bool,
    landed: Optional[bool],
    ugv_speed_mps: float,
    stopped_speed_mps: float,
) -> bool:
    """Permit mission-state reset only after both vehicles are positively safe."""

    return bool(
        state in (MissionState.COMPLETE, MissionState.ABORTED, MissionState.FAULT)
        and not armed
        and landed is True
        and abs(float(ugv_speed_mps)) <= max(float(stopped_speed_mps), 0.0)
    )


def mavlink_command_ack_outcome(acknowledgement, command_id: int):
    """Classify an ACK as accepted, in progress, failed, or unrelated.

    MAV_RESULT_ACCEPTED is 0 and MAV_RESULT_IN_PROGRESS is 5. Keeping these
    protocol constants in a pure helper avoids adding pymavlink to mission
    state-machine logic and makes retry pacing directly testable.
    """

    if not isinstance(acknowledgement, dict):
        return None
    try:
        if int(acknowledgement.get("command")) != int(command_id):
            return None
        result = int(acknowledgement.get("result"))
    except (TypeError, ValueError):
        return None
    if result == 0:
        return "accepted"
    if result == 5:
        return "in_progress"
    return "failed"


def acknowledged_retry_deadline(
    *, now_s: float, wall_now_s: float, ack_wall_s: float, delay_s: float
) -> float:
    """Map a wall-stamped FCU acknowledgement delay to a monotonic deadline.

    MAVLink telemetry carries a wall-clock receipt timestamp while mission
    watchdogs intentionally use a steady clock.  Accounting for ACK age here
    prevents callback and status-publication latency from extending the
    configured transaction window.
    """

    ack_age = max(0.0, float(wall_now_s) - float(ack_wall_s))
    remaining = max(0.0, float(delay_s) - ack_age)
    return float(now_s) + remaining


def failed_ack_retry_deadline(
    *, now_s: float, wall_now_s: float, ack_wall_s: float, cooldown_s: float
) -> float:
    """Backward-compatible failure-cooldown specialization."""

    return acknowledged_retry_deadline(
        now_s=now_s,
        wall_now_s=wall_now_s,
        ack_wall_s=ack_wall_s,
        delay_s=cooldown_s,
    )


def mission_start_is_safe(
    state: MissionState,
    *,
    armed: bool,
    landed: Optional[bool],
    ugv_speed_mps: float,
    stopped_speed_mps: float,
) -> bool:
    """Require a deliberate reset and a quiescent launch condition."""

    return bool(
        state == MissionState.IDLE
        and not armed
        and landed is True
        and abs(float(ugv_speed_mps)) <= max(float(stopped_speed_mps), 0.0)
    )


def split_speed_scale(requested: float) -> tuple[float, float, float]:
    """Route a speed envelope without fighting closed-loop smoothing.

    Nav2's ``SpeedLimit`` is applied before its closed-loop velocity smoother,
    while the chassis adapter remains a binary, downstream safety gate. A zero
    Nav2 speed limit means *no limit*, so a stop must close the safety gate.

    Returns ``(clamped_scale, nav2_percentage, chassis_safety_gate)``.
    """

    scale = max(0.0, min(1.0, float(requested)))
    if scale <= 0.0:
        return 0.0, 0.0, 0.0
    return scale, scale * 100.0, 1.0


@dataclass(frozen=True)
class MissionFacts:
    connected: bool = False
    flight_ready: bool = False
    armed: bool = False
    altitude_m: float = 0.0
    navigation_reached: bool = False
    docking_capture_ready: bool = False
    dock_detached: Optional[bool] = None
    landed: Optional[bool] = None
    ugv_goal_done: bool = False
    ugv_stopped_stable: bool = False
    ugv_moving: bool = False
    ugv_motion_envelope: bool = False
    docking_separation_m: Optional[float] = None


STATE_TIMEOUTS = {
    MissionState.RELEASE_REMOTE_DOCK: 8.0,
    MissionState.WAIT_AUTOPILOT: 70.0,
    MissionState.ARM_INITIAL: 30.0,
    MissionState.TAKEOFF_INITIAL: 45.0,
    MissionState.NAVIGATE_TO_START_DOCK: 120.0,
    MissionState.DOCK_AT_START: 100.0,
    MissionState.LATCH_AT_START: 12.0,
    MissionState.RELEASE_FOR_TRANSIT: 12.0,
    MissionState.ARM_FOR_TRANSIT: 30.0,
    MissionState.TAKEOFF_FOR_TRANSIT: 45.0,
    MissionState.PARALLEL_TRANSIT: 180.0,
    MissionState.DOCK_STOPPED: 100.0,
    MissionState.LATCH_STOPPED: 12.0,
    MissionState.RELEASE_FOR_FOLLOW: 12.0,
    MissionState.ARM_FOR_FOLLOW: 30.0,
    MissionState.TAKEOFF_FOR_FOLLOW: 45.0,
    # The forward-only Ackermann U-turn is route length limited, not merely a
    # short mode transition. Keep a conservative default and let commissioned
    # mission profiles provide explicit overrides.
    MissionState.FOLLOW_MOVING_UGV: 180.0,
    MissionState.DOCK_MOVING: 100.0,
    MissionState.LATCH_MOVING: 12.0,
    MissionState.RIDE_AND_DECELERATE: 150.0,
}


def next_state(
    state: MissionState,
    elapsed_s: float,
    facts: MissionFacts,
    timeout_scale: float = 1.0,
    timeout_overrides_s: Optional[Mapping[MissionState, float]] = None,
) -> MissionState:
    """Return the next state or the current state if its guard is not met."""
    timeout = STATE_TIMEOUTS.get(state)
    if timeout_overrides_s is not None and state in timeout_overrides_s:
        timeout = max(float(timeout_overrides_s[state]), 0.1)
    if timeout is not None and elapsed_s > timeout * max(float(timeout_scale), 0.1):
        return MissionState.FAULT

    if state == MissionState.RELEASE_REMOTE_DOCK:
        return MissionState.WAIT_AUTOPILOT if facts.dock_detached is True else state
    if state == MissionState.WAIT_AUTOPILOT:
        return MissionState.ARM_INITIAL if facts.connected and facts.flight_ready else state
    if state == MissionState.ARM_INITIAL:
        return MissionState.TAKEOFF_INITIAL if facts.armed else state
    if state == MissionState.TAKEOFF_INITIAL:
        return MissionState.NAVIGATE_TO_START_DOCK if facts.altitude_m >= 2.2 else state
    if state == MissionState.NAVIGATE_TO_START_DOCK:
        close = facts.docking_separation_m is not None and facts.docking_separation_m <= 3.2
        return MissionState.DOCK_AT_START if facts.navigation_reached or close else state
    if state == MissionState.DOCK_AT_START:
        return MissionState.LATCH_AT_START if facts.docking_capture_ready else state
    if state == MissionState.LATCH_AT_START:
        dock_confirmed = facts.dock_detached is False
        return (
            MissionState.DWELL_AT_START
            if dock_confirmed and facts.landed is True and not facts.armed
            else state
        )
    if state == MissionState.DWELL_AT_START:
        return MissionState.RELEASE_FOR_TRANSIT if elapsed_s >= 3.0 else state
    if state == MissionState.RELEASE_FOR_TRANSIT:
        release_settled = elapsed_s >= 2.0 * max(float(timeout_scale), 0.1)
        return (
            MissionState.ARM_FOR_TRANSIT
            if facts.dock_detached is True
            and facts.landed is True
            and release_settled
            else state
        )
    if state == MissionState.ARM_FOR_TRANSIT:
        return MissionState.TAKEOFF_FOR_TRANSIT if facts.armed else state
    if state == MissionState.TAKEOFF_FOR_TRANSIT:
        return MissionState.PARALLEL_TRANSIT if facts.altitude_m >= 2.2 else state
    if state == MissionState.PARALLEL_TRANSIT:
        return (
            MissionState.DOCK_STOPPED
            if facts.ugv_goal_done and facts.navigation_reached
            else state
        )
    if state == MissionState.DOCK_STOPPED:
        return MissionState.LATCH_STOPPED if facts.docking_capture_ready else state
    if state == MissionState.LATCH_STOPPED:
        dock_confirmed = facts.dock_detached is False
        return (
            MissionState.DWELL_STOPPED
            if dock_confirmed and facts.landed is True and not facts.armed
            else state
        )
    if state == MissionState.DWELL_STOPPED:
        return MissionState.RELEASE_FOR_FOLLOW if elapsed_s >= 3.0 else state
    if state == MissionState.RELEASE_FOR_FOLLOW:
        release_settled = elapsed_s >= 2.0 * max(float(timeout_scale), 0.1)
        return (
            MissionState.ARM_FOR_FOLLOW
            if facts.dock_detached is True
            and facts.landed is True
            and release_settled
            else state
        )
    if state == MissionState.ARM_FOR_FOLLOW:
        return MissionState.TAKEOFF_FOR_FOLLOW if facts.armed else state
    if state == MissionState.TAKEOFF_FOR_FOLLOW:
        return MissionState.FOLLOW_MOVING_UGV if facts.altitude_m >= 2.2 else state
    if state == MissionState.FOLLOW_MOVING_UGV:
        close = facts.docking_separation_m is not None and facts.docking_separation_m <= 4.0
        return (
            MissionState.DOCK_MOVING
            if elapsed_s >= 8.0
            and close
            and facts.ugv_moving
            and facts.ugv_motion_envelope
            else state
        )
    if state == MissionState.DOCK_MOVING:
        return (
            MissionState.LATCH_MOVING
            if facts.docking_capture_ready
            and facts.ugv_moving
            and facts.ugv_motion_envelope
            else state
        )
    if state == MissionState.LATCH_MOVING:
        dock_confirmed = facts.dock_detached is False
        return (
            MissionState.RIDE_AND_DECELERATE
            if dock_confirmed and facts.landed is True and not facts.armed
            else state
        )
    if state == MissionState.RIDE_AND_DECELERATE:
        return (
            MissionState.COMPLETE
            if facts.ugv_goal_done and facts.ugv_stopped_stable
            else state
        )
    return state
