"""Transport-free lifecycle for the single ROS 2 flight action executor.

The task layer sees one lifecycle.  Timing and phase memory needed to execute a
closed-loop action remains private to this class and never owns a MAVLink
transport.  All coordinates are ROS local ENU and vertical velocity is positive
up.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import math
from typing import Any, Mapping, Optional

from .guided_descent import DescentInput, GuidedDescent


class ActionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    DONE = "DONE"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    CANCELLED = "CANCELLED"


class ActionKind(str, Enum):
    MOVE = "MOVE"
    ROTATE = "ROTATE"
    HOVER = "HOVER"
    FOLLOW = "FOLLOW"
    PRECISION_LAND = "PRECISION_LAND"
    LAND = "LAND"
    DISARM = "DISARM"
    EMERGENCY_STOP = "EMERGENCY_STOP"


GUIDED_ACTIONS = frozenset({
    ActionKind.MOVE,
    ActionKind.ROTATE,
    ActionKind.HOVER,
    ActionKind.FOLLOW,
    ActionKind.PRECISION_LAND,
})
LAND_ACTIONS = frozenset({ActionKind.LAND})
FLIGHT_ACTIONS = GUIDED_ACTIONS | LAND_ACTIONS


@dataclass(frozen=True)
class ActionRequest:
    action_id: str
    kind: ActionKind
    timeout_s: float
    params: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "ActionRequest":
        if not isinstance(payload, Mapping):
            raise ValueError("request must be a JSON object")
        action_id = str(payload.get("action_id", "")).strip()
        if not action_id or len(action_id) > 128:
            raise ValueError("action_id must contain 1..128 characters")
        try:
            kind = ActionKind(str(payload.get("action", "")).strip().upper())
        except ValueError as exc:
            raise ValueError("unsupported action") from exc
        timeout_s = _finite_float(payload.get("timeout_s"), "timeout_s")
        if not 0.05 <= timeout_s <= 3600.0:
            raise ValueError("timeout_s must be in [0.05, 3600]")
        params = payload.get("params", {})
        if not isinstance(params, Mapping):
            raise ValueError("params must be a JSON object")
        return cls(action_id=action_id, kind=kind, timeout_s=timeout_s, params=dict(params))


@dataclass(frozen=True)
class VehicleSnapshot:
    now_s: float
    telemetry_fresh: bool
    connected: bool
    armed: bool
    mode: str
    authorized: bool
    ekf_healthy: bool
    home_set: bool
    landed: bool
    position_enu: tuple[float, float, float]
    velocity_enu: tuple[float, float, float]
    yaw_rad: float
    yaw_rate_rad_s: float = 0.0
    tilt_deg: float = 0.0
    candidate_velocity_flu: Optional[tuple[float, float]] = None
    candidate_fresh: bool = False
    target_aligned: bool = False
    landing_target_fresh: bool = False
    landing_target_output_enabled: bool = False
    landing_requested: bool = True
    landing_explicit_low: bool = False
    range_m: float = math.nan
    range_fresh: bool = False
    # Same-frame, quality-gated image-centre correction and calibrated pad yaw.
    landing_alignment_fresh: bool = False
    landing_velocity_flu: Optional[tuple[float, float]] = None
    landing_center_error_px: float = math.nan
    landing_heading_error_rad: float = math.nan  # ROS FLU/ENU: positive left


@dataclass(frozen=True)
class ActionCommand:
    desired_mode: Optional[str] = None
    velocity_enu: Optional[tuple[float, float, float]] = None
    yaw_rad: Optional[float] = None
    yaw_rate_rad_s: Optional[float] = None
    request_disarm: bool = False
    request_emergency_stop: bool = False


@dataclass(frozen=True)
class ActionStatus:
    action_id: Optional[str]
    action: Optional[str]
    state: ActionState
    reason: str
    detail: str
    elapsed_s: float
    control_retained: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "action_id": self.action_id,
            "action": self.action,
            "state": self.state.value,
            "reason": self.reason,
            "detail": self.detail,
            "elapsed_s": self.elapsed_s,
            "control_retained": self.control_retained,
        }


class ActionExecutor:
    """Run one closed-loop action at a time without owning any ROS transport."""

    def __init__(
        self,
        *,
        position_tolerance_m: float = 0.08,
        speed_tolerance_mps: float = 0.08,
        yaw_tolerance_rad: float = math.radians(4.0),
        completion_dwell_s: float = 0.3,
        follow_loss_grace_s: float = 0.5,
        maximum_horizontal_speed_mps: float = 0.5,
        maximum_vertical_speed_mps: float = 0.2,
        maximum_yaw_rate_rad_s: float = math.radians(45.0),
        maximum_horizontal_acceleration_mps2: float = 0.5,
        maximum_vertical_acceleration_mps2: float = 0.3,
        guided_mode: str = "GUIDED",
        land_mode: str = "LAND",
        fallback_mode: str = "LOITER",
        land_recovery_height_m: float = 0.10,
        land_recovery_timeout_s: float = 1.0,
        land_reacquire_dwell_s: float = 0.0,
        land_guided_descent_mps: float = 0.10,
        land_yaw_tolerance_rad: float = math.radians(4.0),
        land_yaw_gain_per_s: float = 0.8,
        land_maximum_yaw_rate_rad_s: float = math.radians(15.0),
    ) -> None:
        values = (
            position_tolerance_m,
            speed_tolerance_mps,
            yaw_tolerance_rad,
            completion_dwell_s,
            follow_loss_grace_s,
            maximum_horizontal_speed_mps,
            maximum_vertical_speed_mps,
            maximum_yaw_rate_rad_s,
            maximum_horizontal_acceleration_mps2,
            maximum_vertical_acceleration_mps2,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("executor limits must be finite and positive")
        self.position_tolerance_m = position_tolerance_m
        self.speed_tolerance_mps = speed_tolerance_mps
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.completion_dwell_s = completion_dwell_s
        self.follow_loss_grace_s = follow_loss_grace_s
        self.maximum_horizontal_speed_mps = maximum_horizontal_speed_mps
        self.maximum_vertical_speed_mps = maximum_vertical_speed_mps
        self.maximum_yaw_rate_rad_s = maximum_yaw_rate_rad_s
        self.maximum_horizontal_acceleration_mps2 = (
            maximum_horizontal_acceleration_mps2
        )
        self.maximum_vertical_acceleration_mps2 = (
            maximum_vertical_acceleration_mps2
        )
        self.guided_mode = str(guided_mode).strip().upper()
        self.land_mode = str(land_mode).strip().upper()
        self.fallback_mode = str(fallback_mode).strip().upper()
        if not self.guided_mode or not self.land_mode or not self.fallback_mode:
            raise ValueError("guided_mode, land_mode and fallback_mode are required")
        recovery_values = (land_recovery_height_m, land_recovery_timeout_s)
        if not all(math.isfinite(value) and value > 0.0 for value in recovery_values):
            raise ValueError("LAND recovery height and timeout must be positive")
        if not math.isfinite(land_reacquire_dwell_s) or land_reacquire_dwell_s < 0.0:
            raise ValueError("LAND reacquire dwell must be non-negative")
        self.land_recovery_height_m = land_recovery_height_m
        self.land_recovery_timeout_s = land_recovery_timeout_s
        self.land_reacquire_dwell_s = land_reacquire_dwell_s
        if not math.isfinite(land_guided_descent_mps) or not 0 < land_guided_descent_mps <= maximum_vertical_speed_mps:
            raise ValueError("GUIDED landing descent must fit the vertical speed limit")
        self.land_guided_descent_mps = land_guided_descent_mps
        alignment_limits = (land_yaw_tolerance_rad,
                            land_yaw_gain_per_s,
                            land_maximum_yaw_rate_rad_s)
        if not all(math.isfinite(v) and v > 0 for v in alignment_limits):
            raise ValueError("landing alignment limits must be finite and positive")
        if land_yaw_tolerance_rad >= math.pi / 2:
            raise ValueError("landing yaw tolerance must distinguish the tag's forward direction")
        self.land_yaw_tolerance_rad = land_yaw_tolerance_rad
        self.land_yaw_gain_per_s = land_yaw_gain_per_s
        self.land_maximum_yaw_rate_rad_s = min(land_maximum_yaw_rate_rad_s, maximum_yaw_rate_rad_s)
        self._descent = GuidedDescent(adjust_while_descending=True)
        self._seen_action_ids: set[str] = set()
        self.reset()

    def reset(self) -> None:
        self.request: Optional[ActionRequest] = None
        self.state = ActionState.IDLE
        self.reason = "READY"
        self.detail = "IDLE"
        self.started_s: Optional[float] = None
        self.last_tick_s: Optional[float] = None
        self.goal_position: Optional[tuple[float, float, float]] = None
        self.goal_yaw: Optional[float] = None
        self.completed_since: Optional[float] = None
        self.candidate_missing_since: Optional[float] = None
        self.land_phase = "INACTIVE"
        self.land_hold_started_s: Optional[float] = None
        self.land_reacquired_since_s: Optional[float] = None
        self.land_telemetry_loss_started_s: Optional[float] = None
        self.land_exit_mode: Optional[str] = None
        self.land_exit_reason = ""
        self._land_expected_mode = self.land_mode
        self._terminal_command: Optional[ActionCommand] = None
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self.last_velocity_command_s: Optional[float] = None
        self._descent.reset()

    @property
    def active(self) -> bool:
        return self.state == ActionState.RUNNING and self.request is not None

    @property
    def retaining_control(self) -> bool:
        return self._terminal_command is not None

    @property
    def expected_mode(self) -> Optional[str]:
        if self.active and self.request is not None:
            if self.request.kind in GUIDED_ACTIONS:
                return self.guided_mode
            if self.request.kind == ActionKind.LAND:
                return self._land_expected_mode
        if self._terminal_command is not None:
            return self._terminal_command.desired_mode
        return None

    @property
    def guided_landing(self) -> bool:
        return bool(self.request and self.request.kind == ActionKind.LAND
                    and self.request.params.get("guided_descent", self.request.params.get("precision", True)) is True)

    def start(
        self, request: ActionRequest, snapshot: VehicleSnapshot
    ) -> ActionStatus:
        self._validate_snapshot_clock(snapshot)
        if (
            self.request is not None
            and self.active
            and request.action_id == self.request.action_id
        ):
            return self._status(
                snapshot.now_s,
                reason="DUPLICATE_ACTIVE_REQUEST",
                detail=self.detail,
            )
        if request.action_id in self._seen_action_ids:
            return ActionStatus(
                action_id=request.action_id,
                action=request.kind.value,
                state=ActionState.FAILED,
                reason="REJECTED_ACTION_ID_REPLAY",
                detail="USE_A_NEW_ACTION_ID",
                elapsed_s=0.0,
            )
        if self.active and request.kind != ActionKind.EMERGENCY_STOP:
            self._seen_action_ids.add(request.action_id)
            return ActionStatus(
                action_id=request.action_id,
                action=request.kind.value,
                state=ActionState.FAILED,
                reason="REJECTED_BUSY",
                detail=f"ACTIVE_ACTION:{self.request.action_id}",
                elapsed_s=0.0,
            )
        if self.active:
            self.state = ActionState.CANCELLED
            self.reason = "PREEMPTED_BY_EMERGENCY_STOP"
            self._terminal_command = None
        self.reset()
        self.request = request
        self._seen_action_ids.add(request.action_id)
        self.started_s = snapshot.now_s
        self.last_tick_s = snapshot.now_s
        self.last_velocity_command_s = snapshot.now_s
        rejection = self._precondition_rejection(request, snapshot)
        if rejection is not None:
            self.state = ActionState.FAILED
            self.reason = rejection
            self.detail = "START_REJECTED"
            return self._status(snapshot.now_s)
        try:
            self._prepare(request, snapshot)
        except ValueError as exc:
            self.state = ActionState.FAILED
            self.reason = "REJECTED_INVALID_PARAMETERS"
            self.detail = str(exc)
            return self._status(snapshot.now_s)
        self.state = ActionState.RUNNING
        self.reason = "ACCEPTED"
        if request.kind in GUIDED_ACTIONS:
            self.detail = "WAITING_GUIDED"
        elif request.kind in LAND_ACTIONS:
            self.detail = "WAITING_LAND"
        else:
            self.detail = "EXECUTING"
        return self._status(snapshot.now_s)

    def cancel(
        self,
        action_id: Optional[str],
        now_s: float,
        reason: str = "TASK_CANCELLED",
    ) -> ActionStatus:
        if not self.active and not self.retaining_control:
            return self._status(now_s, reason="REJECTED_NO_ACTIVE_ACTION", detail="IDLE")
        if self.request is not None and self.request.kind == ActionKind.EMERGENCY_STOP:
            return self._status(
                now_s,
                reason="REJECTED_EMERGENCY_STOP_NOT_CANCELLABLE",
                detail="FCU_ESTOP_REMAINS_LATCHED",
            )
        if action_id is not None and action_id != self.request.action_id:
            return self._status(
                now_s,
                reason="REJECTED_ACTION_ID_MISMATCH",
                detail=action_id,
            )
        self.state = ActionState.CANCELLED
        self.reason = reason
        self.detail = "CONTROL_RELEASED"
        self._terminal_command = None
        return self._status(now_s)

    def fail(self, now_s: float, reason: str, detail: str = "") -> ActionStatus:
        if not self.active:
            return self._status(now_s)
        self._finish(ActionState.FAILED, reason, detail or "FAILED", retain_hold=False)
        return self._status(now_s)

    def tick(
        self, snapshot: VehicleSnapshot
    ) -> tuple[ActionStatus, Optional[ActionCommand]]:
        if not self.active:
            if self._terminal_command is not None:
                terminal_mode = self._terminal_command.desired_mode
                if terminal_mode == self.guided_mode:
                    release = not self._can_command_guided(snapshot)
                else:
                    release = bool(
                        not snapshot.connected
                        or snapshot.mode.strip().upper() == terminal_mode
                    )
                if release:
                    self._terminal_command = None
                    self.detail = "CONTROL_RELEASED"
            return self._status(snapshot.now_s), self._terminal_command
        assert self.request is not None and self.started_s is not None
        if (not math.isfinite(snapshot.now_s)
                or (self.last_tick_s is not None and snapshot.now_s < self.last_tick_s)):
            self._finish(
                ActionState.FAILED,
                "INVALID_CLOCK",
                "CONTROL_RELEASED",
                retain_hold=False,
            )
            return self._status(snapshot.now_s), None
        self.last_tick_s = snapshot.now_s
        if self.request.kind == ActionKind.DISARM and not snapshot.armed:
            command = self._tick_disarm(snapshot)
            return self._status(snapshot.now_s), command
        if self.request.kind == ActionKind.LAND and not snapshot.armed:
            self._finish(
                ActionState.DONE,
                "LAND_AND_DISARM_CONFIRMED",
                "FCU_LAND_COMPLETE",
                retain_hold=False,
            )
            return self._status(snapshot.now_s), None
        if (not snapshot.authorized
                and self.request.kind not in {
                    ActionKind.EMERGENCY_STOP,
                    ActionKind.LAND,
                }):
            self._finish(
                ActionState.CANCELLED,
                "AUTHORIZATION_WITHDRAWN",
                "RETURNING_LOITER",
                retain_hold=False,
            )
            self._terminal_command = ActionCommand(desired_mode=self.fallback_mode)
            return self._status(snapshot.now_s), self._terminal_command
        if snapshot.now_s - self.started_s >= self.request.timeout_s:
            can_hold = (
                (self.request.kind in GUIDED_ACTIONS or self.guided_landing)
                and self._can_command_guided(snapshot)
            )
            detail = "ZERO_VELOCITY_HOLD" if can_hold else "CONTROL_RELEASED"
            if self.request.kind == ActionKind.LAND and not can_hold:
                detail = "FCU_LAND_CONTINUES"
            self._finish(
                ActionState.TIMEOUT,
                "ACTION_TIMEOUT",
                detail,
                retain_hold=can_hold,
            )
            return self._status(snapshot.now_s), self._terminal_command
        if self.request.kind == ActionKind.LAND:
            # A fresh range or FCU ground indication is sufficient to hand the
            # final centimetres to native LAND. Pose/velocity callbacks can
            # briefly lag even while FCU state and range remain current.
            near_ground = bool(
                snapshot.connected and snapshot.authorized and
                (snapshot.landed or self.land_phase in {
                    "HYBRID_LAND_COMMITTED", "NEAR_GROUND_DISARM", "LANDED_DISARM"
                } or (
                    snapshot.range_fresh and math.isfinite(snapshot.range_m)
                    and round(snapshot.range_m, 6) <= self.land_recovery_height_m
                ))
            )
            if near_ground and (not snapshot.telemetry_fresh or not snapshot.ekf_healthy):
                command = self._tick_land(snapshot)
                return self._status(snapshot.now_s), command
            if snapshot.telemetry_fresh:
                self.land_telemetry_loss_started_s = None
            elif snapshot.connected and snapshot.authorized:
                if self.land_telemetry_loss_started_s is None:
                    self.land_telemetry_loss_started_s = snapshot.now_s
                if snapshot.now_s - self.land_telemetry_loss_started_s < 1.0:
                    self.detail = "FLIGHT_TELEMETRY_GRACE_HOLD"
                    return self._status(snapshot.now_s), None
        rejection = self._runtime_rejection(self.request.kind, snapshot)
        if rejection is not None:
            self._finish(
                ActionState.FAILED,
                rejection,
                "CONTROL_RELEASED",
                retain_hold=False,
            )
            return self._status(snapshot.now_s), None
        if self.request.kind == ActionKind.LAND:
            command = self._tick_land(snapshot)
            return self._status(snapshot.now_s), command
        if (
            self.request.kind in FLIGHT_ACTIONS
            and snapshot.mode.strip().upper() != self._desired_mode(self.request.kind)
        ):
            desired_mode = self._desired_mode(self.request.kind)
            self.detail = f"WAITING_{desired_mode}_HEARTBEAT"
            return self._status(snapshot.now_s), ActionCommand(desired_mode=desired_mode)
        handler = {
            ActionKind.MOVE: self._tick_move,
            ActionKind.ROTATE: self._tick_rotate,
            ActionKind.HOVER: self._tick_hover,
            ActionKind.FOLLOW: self._tick_follow,
            ActionKind.PRECISION_LAND: self._tick_precision_land,
            ActionKind.DISARM: self._tick_disarm,
            ActionKind.EMERGENCY_STOP: self._tick_emergency_stop,
        }[self.request.kind]
        command = handler(snapshot)
        return self._status(snapshot.now_s), command

    def _prepare(self, request: ActionRequest, snapshot: VehicleSnapshot) -> None:
        params = request.params
        if request.kind == ActionKind.MOVE:
            direction = _vector3(params.get("direction"), "direction")
            norm = math.sqrt(sum(value * value for value in direction))
            if norm <= 1e-9:
                raise ValueError("direction must be non-zero")
            distance = _positive(params.get("distance_m"), "distance_m")
            speed = _positive(params.get("speed_mps"), "speed_mps")
            unit = tuple(value / norm for value in direction)
            self.goal_position = tuple(
                snapshot.position_enu[index] + unit[index] * distance for index in range(3)
            )
            if str(params.get("frame", "LOCAL_ENU")).upper() != "LOCAL_ENU":
                raise ValueError("MOVE currently supports frame=LOCAL_ENU only")
            horizontal_speed = speed * math.hypot(unit[0], unit[1])
            vertical_speed = speed * abs(unit[2])
            if horizontal_speed > self.maximum_horizontal_speed_mps:
                raise ValueError("MOVE exceeds executor horizontal speed limit")
            if vertical_speed > self.maximum_vertical_speed_mps:
                raise ValueError("MOVE exceeds executor vertical speed limit")
        elif request.kind == ActionKind.ROTATE:
            if "angle_deg" in params:
                angle = math.radians(_finite_float(params["angle_deg"], "angle_deg"))
                self.goal_yaw = _wrap_angle(snapshot.yaw_rad + angle)
            elif "heading_deg" in params:
                self.goal_yaw = _wrap_angle(math.radians(
                    _finite_float(params["heading_deg"], "heading_deg")
                ))
            elif "yaw_rate_deg_s" in params:
                rate = _finite_float(params["yaw_rate_deg_s"], "yaw_rate_deg_s")
                if not (
                    0.1
                    <= abs(math.radians(rate))
                    <= self.maximum_yaw_rate_rad_s
                ):
                    raise ValueError("yaw_rate_deg_s exceeds executor yaw-rate limit")
            else:
                raise ValueError("ROTATE needs angle_deg, heading_deg or yaw_rate_deg_s")
        elif request.kind == ActionKind.HOVER:
            if "duration_s" in params:
                duration = _finite_float(params["duration_s"], "duration_s")
                if not 0.0 < duration < request.timeout_s:
                    raise ValueError("duration_s must be positive and < timeout_s")
        elif request.kind == ActionKind.FOLLOW:
            maximum = _positive(
                params.get("maximum_speed_mps", 0.5),
                "maximum_speed_mps",
            )
            if maximum > self.maximum_horizontal_speed_mps:
                raise ValueError("FOLLOW exceeds executor horizontal speed limit")
        elif request.kind == ActionKind.LAND:
            precision = params.get("precision", True)
            if not isinstance(precision, bool):
                raise ValueError("LAND precision must be a boolean")
            rc_managed = params.get("rc_managed", False)
            if not isinstance(rc_managed, bool):
                raise ValueError("LAND rc_managed must be a boolean")
            loss_recovery = params.get("loss_recovery", True)
            if not isinstance(loss_recovery, bool):
                raise ValueError("LAND loss_recovery must be a boolean")
            self.land_phase = "REQUEST_LAND"
            self._land_expected_mode = self.land_mode
            guided_descent = params.get("guided_descent", precision)
            if not isinstance(guided_descent, bool):
                raise ValueError("LAND guided_descent must be a boolean")
            if guided_descent:
                self.land_phase = "GUIDED_TRACK_DESCENT"
                self._land_expected_mode = self.guided_mode

    def _tick_move(self, snapshot: VehicleSnapshot) -> ActionCommand:
        assert self.request is not None and self.goal_position is not None
        error = tuple(
            self.goal_position[i] - snapshot.position_enu[i]
            for i in range(3)
        )
        distance = math.sqrt(sum(value * value for value in error))
        speed = math.sqrt(sum(value * value for value in snapshot.velocity_enu))
        within_goal = (
            distance <= self.position_tolerance_m
            and speed <= self.speed_tolerance_mps
        )
        if self._dwell_complete(snapshot.now_s, within_goal):
            self._finish(
                ActionState.DONE,
                "GOAL_REACHED",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        maximum = _positive(self.request.params["speed_mps"], "speed_mps")
        commanded = min(maximum, max(0.05, distance))
        if distance <= 1e-9:
            velocity = (0.0, 0.0, 0.0)
        else:
            velocity = tuple(commanded * value / distance for value in error)
        self.detail = "MOVING"
        return self._limited_velocity_command(snapshot.now_s, velocity)

    def _tick_rotate(self, snapshot: VehicleSnapshot) -> ActionCommand:
        assert self.request is not None
        if self.goal_yaw is None:
            rate = math.radians(_finite_float(
                self.request.params["yaw_rate_deg_s"],
                "yaw_rate_deg_s",
            ))
            self.detail = "ROTATING_CONTINUOUS"
            return ActionCommand(
                desired_mode=self.guided_mode,
                velocity_enu=(0.0, 0.0, 0.0),
                yaw_rate_rad_s=rate,
            )
        error = abs(_wrap_angle(self.goal_yaw - snapshot.yaw_rad))
        if self._dwell_complete(snapshot.now_s, error <= self.yaw_tolerance_rad):
            self._finish(
                ActionState.DONE,
                "HEADING_REACHED",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        self.detail = "ROTATING_TO_HEADING"
        return ActionCommand(
            desired_mode=self.guided_mode,
            velocity_enu=(0.0, 0.0, 0.0),
            yaw_rad=self.goal_yaw,
        )

    def _tick_hover(self, snapshot: VehicleSnapshot) -> ActionCommand:
        assert self.request is not None and self.started_s is not None
        speed = math.sqrt(sum(value * value for value in snapshot.velocity_enu))
        duration_value = self.request.params.get("duration_s")
        if (duration_value is not None
                and snapshot.now_s - self.started_s
                >= _finite_float(duration_value, "duration_s")
                and speed <= self.speed_tolerance_mps):
            self._finish(
                ActionState.DONE,
                "HOVER_COMPLETE",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        self.detail = "HOLDING_ZERO_VELOCITY"
        return ActionCommand(
            desired_mode=self.guided_mode,
            velocity_enu=(0.0, 0.0, 0.0),
        )

    def _tick_follow(self, snapshot: VehicleSnapshot) -> ActionCommand:
        assert self.request is not None
        if not snapshot.candidate_fresh or snapshot.candidate_velocity_flu is None:
            if self.candidate_missing_since is None:
                self.candidate_missing_since = snapshot.now_s
            if snapshot.now_s - self.candidate_missing_since > self.follow_loss_grace_s:
                self._finish(
                    ActionState.FAILED,
                    "FOLLOW_TARGET_LOST",
                    "ZERO_VELOCITY_HOLD",
                    retain_hold=True,
                )
                return self._terminal_command or ActionCommand()
            self.detail = "FOLLOW_TARGET_GRACE_HOLD"
            return ActionCommand(
                desired_mode=self.guided_mode,
                velocity_enu=(0.0, 0.0, 0.0),
            )
        self.candidate_missing_since = None
        vx, vy = snapshot.candidate_velocity_flu
        if not all(math.isfinite(v) for v in (vx, vy, snapshot.yaw_rad)):
            self._finish(
                ActionState.FAILED,
                "INVALID_FOLLOW_CANDIDATE",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        east = math.cos(snapshot.yaw_rad) * vx - math.sin(snapshot.yaw_rad) * vy
        north = math.sin(snapshot.yaw_rad) * vx + math.cos(snapshot.yaw_rad) * vy
        maximum = _positive(
            self.request.params.get("maximum_speed_mps", 0.5),
            "maximum_speed_mps",
        )
        magnitude = math.hypot(east, north)
        if magnitude > maximum:
            scale = maximum / magnitude
            east, north = east * scale, north * scale
        self.detail = "FOLLOWING_TARGET"
        return self._limited_velocity_command(
            snapshot.now_s,
            (east, north, 0.0),
        )

    def _tick_precision_land(self, snapshot: VehicleSnapshot) -> ActionCommand:
        if snapshot.landed:
            self._finish(
                ActionState.DONE,
                "LANDING_DETECTED",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        horizontal_speed = math.hypot(snapshot.velocity_enu[0], snapshot.velocity_enu[1])
        output = self._descent.update(DescentInput(
            now=snapshot.now_s,
            authorized=snapshot.authorized,
            armed=snapshot.armed,
            guided_confirmed=snapshot.mode.strip().upper() == self.guided_mode,
            requested=True,
            landed=snapshot.landed,
            telemetry_fresh=snapshot.telemetry_fresh,
            range_m=snapshot.range_m,
            range_fresh=snapshot.range_fresh,
            tag_fresh=self._landing_observation_valid(snapshot),
            aligned=snapshot.target_aligned,
            horizontal_speed=horizontal_speed,
            tilt_deg=snapshot.tilt_deg,
        ))
        if output.phase == "FAULT_HOLD":
            self._finish(
                ActionState.FAILED,
                "PRECISION_LAND_FAULT",
                "ZERO_VELOCITY_HOLD",
                retain_hold=True,
            )
            return self._terminal_command or ActionCommand()
        self.detail = output.phase
        if not output.track_tag:
            return self._guided_land_hold(snapshot, output.phase)
        return self._landing_tracking_command(snapshot, output.up_mps)

    def _tick_land(self, snapshot: VehicleSnapshot) -> ActionCommand:
        if snapshot.landed:
            if self.guided_landing:
                self.land_phase = "LANDED_DISARM"
                self._land_expected_mode = self.land_mode
                if snapshot.mode.strip().upper() != self.land_mode:
                    self.detail = "LANDED_REQUEST_LAND"
                    return ActionCommand(desired_mode=self.land_mode)
                self.detail = "LANDED_WAIT_DISARM"
                return ActionCommand(desired_mode=self.land_mode, request_disarm=True)
            self._finish(
                ActionState.DONE,
                "LANDING_DETECTED",
                "FCU_LAND_COMPLETE",
                retain_hold=False,
            )
            return ActionCommand()
        if self.guided_landing and self.land_phase == "LANDED_DISARM":
            self._land_expected_mode = self.land_mode
            self.detail = "LAND_COMMITTED_WAIT_LANDED"
            return ActionCommand(desired_mode=self.land_mode)
        if (self.request is not None
                and self.guided_landing
                and self.land_phase == "NEAR_GROUND_DISARM"
                and snapshot.authorized):
            return self._tick_guided_land(snapshot)
        assert self.request is not None
        params = self.request.params
        rc_managed = params.get("rc_managed", False) is True
        loss_recovery = params.get("loss_recovery", True) is True
        mode = snapshot.mode.strip().upper()

        if self.land_phase.startswith("EXIT_"):
            assert self.land_exit_mode is not None
            if mode != self.land_exit_mode:
                self.detail = f"WAITING_{self.land_exit_mode}_HEARTBEAT"
                # Keep zero velocity/yaw flowing while a GUIDED -> LOITER
                # handoff awaits the FCU heartbeat. The transport only sends
                # setpoints while the FCU still reports GUIDED.
                return ActionCommand(
                    desired_mode=self.land_exit_mode,
                    velocity_enu=(0.0, 0.0, 0.0),
                    yaw_rate_rad_s=0.0,
                )
            retain_guided = self.land_exit_mode == self.guided_mode
            self._finish(
                ActionState.DONE,
                self.land_exit_reason,
                f"LAND_EXITED_TO_{self.land_exit_mode}",
                retain_hold=retain_guided,
            )
            return self._terminal_command or ActionCommand()

        if not snapshot.authorized:
            return self._begin_land_exit(
                self.fallback_mode,
                "LAND_EXIT_CH6_AUTHORIZATION_WITHDRAWN",
            )
        if self.guided_landing:
            return self._tick_guided_land(snapshot)
        if rc_managed and (
            snapshot.landing_explicit_low or not snapshot.landing_requested
        ):
            exit_mode = (
                self.guided_mode if snapshot.candidate_fresh
                else self.fallback_mode
            )
            return self._begin_land_exit(exit_mode, "LAND_EXIT_CH8_RELEASED")

        if self.land_phase == "REQUEST_LAND":
            if mode != self.land_mode:
                self.detail = "WAITING_LAND_HEARTBEAT"
                return ActionCommand(desired_mode=self.land_mode)
            self.land_phase = "LAND_ACTIVE"

        if self.land_phase in {
            "REQUEST_GUIDED_HOLD",
            "GUIDED_REACQUIRE_HOLD",
        }:
            if snapshot.candidate_fresh:
                if self.land_reacquired_since_s is None:
                    self.land_reacquired_since_s = snapshot.now_s
                if (
                    snapshot.now_s - self.land_reacquired_since_s
                    >= self.land_reacquire_dwell_s
                ):
                    self.land_phase = "REQUEST_LAND"
                    self.land_hold_started_s = None
                    self.land_reacquired_since_s = None
                    self._land_expected_mode = self.land_mode
                    self.detail = "TAG_REACQUIRED_REQUEST_LAND"
                    return ActionCommand(desired_mode=self.land_mode)
            else:
                self.land_reacquired_since_s = None

            if mode != self.guided_mode:
                self.land_phase = "REQUEST_GUIDED_HOLD"
                self._land_expected_mode = self.guided_mode
                self.detail = "TAG_LOST_REQUEST_GUIDED_HOLD"
                return ActionCommand(desired_mode=self.guided_mode)
            if (
                snapshot.range_fresh
                and math.isfinite(snapshot.range_m)
                and snapshot.range_m <= self.land_recovery_height_m
            ):
                # A GUIDED request may already be accepted by the FCU when the
                # range crosses the commit height.  First observe that pending
                # heartbeat, then immediately return to LAND; otherwise its
                # late arrival could leave the vehicle in GUIDED unexpectedly.
                self.land_phase = "REQUEST_LAND"
                self.land_hold_started_s = None
                self.land_reacquired_since_s = None
                self._land_expected_mode = self.land_mode
                self.detail = "RECOVERY_ABORTED_BELOW_COMMIT_HEIGHT"
                return ActionCommand(desired_mode=self.land_mode)
            if self.land_hold_started_s is None:
                self.land_hold_started_s = snapshot.now_s
            if (
                snapshot.now_s - self.land_hold_started_s
                >= self.land_recovery_timeout_s
            ):
                exit_mode = (
                    self.guided_mode if snapshot.candidate_fresh
                    else self.fallback_mode
                )
                return self._begin_land_exit(
                    exit_mode,
                    "LAND_EXIT_TAG_REACQUIRE_TIMEOUT",
                )
            self.land_phase = "GUIDED_REACQUIRE_HOLD"
            self._land_expected_mode = self.guided_mode
            self.detail = "TAG_LOST_GUIDED_HOLD"
            return ActionCommand(
                desired_mode=self.guided_mode,
                velocity_enu=(0.0, 0.0, 0.0),
            )

        if mode != self.land_mode:
            self._land_expected_mode = self.land_mode
            self.detail = "WAITING_LAND_HEARTBEAT"
            return ActionCommand(desired_mode=self.land_mode)

        if self.land_phase == "LAND_COMMITTED":
            self.detail = "LAND_COMMITTED"
            return ActionCommand(desired_mode=self.land_mode)

        if (
            loss_recovery
            and not snapshot.candidate_fresh
            and snapshot.range_fresh
            and math.isfinite(snapshot.range_m)
            and snapshot.range_m > self.land_recovery_height_m
        ):
            self.land_phase = "REQUEST_GUIDED_HOLD"
            self.land_hold_started_s = None
            self.land_reacquired_since_s = None
            self._land_expected_mode = self.guided_mode
            self.detail = "TAG_LOST_REQUEST_GUIDED_HOLD"
            return ActionCommand(desired_mode=self.guided_mode)

        if (
            snapshot.range_fresh
            and math.isfinite(snapshot.range_m)
            and snapshot.range_m <= self.land_recovery_height_m
        ):
            self.land_phase = "LAND_COMMITTED"
            self.detail = "LAND_COMMITTED"
        else:
            self.land_phase = "LAND_ACTIVE"
            self.detail = "FCU_LAND_ACTIVE"
        self._land_expected_mode = self.land_mode
        return ActionCommand(desired_mode=self.land_mode)

    def _begin_land_exit(self, mode: str, reason: str) -> ActionCommand:
        self.land_exit_mode = mode
        self.land_exit_reason = reason
        self.land_phase = f"EXIT_{mode}"
        self._land_expected_mode = mode
        self.detail = f"WAITING_{mode}_HEARTBEAT"
        return ActionCommand(
            desired_mode=mode,
            velocity_enu=(0.0, 0.0, 0.0),
            yaw_rate_rad_s=0.0,
        )

    def _guided_land_hold(self, snapshot: VehicleSnapshot, detail: str) -> ActionCommand:
        # Reset limiter memory as well as output: recovery must not reuse descent.
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self.last_velocity_command_s = snapshot.now_s
        self.detail = detail
        self._land_expected_mode = self.guided_mode
        return ActionCommand(desired_mode=self.guided_mode, velocity_enu=(0.0, 0.0, 0.0), yaw_rate_rad_s=0.0)

    @staticmethod
    def _landing_observation_valid(snapshot: VehicleSnapshot) -> bool:
        velocity = snapshot.landing_velocity_flu
        return bool(snapshot.candidate_fresh and snapshot.landing_alignment_fresh
                    and velocity is not None and len(velocity) == 2
                    and all(math.isfinite(v) for v in velocity)
                    and math.isfinite(snapshot.landing_heading_error_rad)
                    and math.isfinite(snapshot.landing_center_error_px)
                    and snapshot.landing_center_error_px >= 0)

    def _landing_tracking_command(self, snapshot: VehicleSnapshot, up_mps: float) -> ActionCommand:
        """Simultaneously centre, turn toward tag top, and descend (ROS ENU)."""
        vx, vy = snapshot.landing_velocity_flu
        east = math.cos(snapshot.yaw_rad) * vx - math.sin(snapshot.yaw_rad) * vy
        north = math.sin(snapshot.yaw_rad) * vx + math.cos(snapshot.yaw_rad) * vy
        magnitude = math.hypot(east, north)
        if magnitude > self.maximum_horizontal_speed_mps:
            scale = self.maximum_horizontal_speed_mps / magnitude
            east, north = east * scale, north * scale
        error = _wrap_angle(snapshot.landing_heading_error_rad)
        rate_limit = self.land_maximum_yaw_rate_rad_s
        yaw_rate = max(-rate_limit, min(rate_limit, self.land_yaw_gain_per_s * error))
        # Small yaw deadband prevents chatter; neither tolerance gates descent.
        if abs(error) <= self.land_yaw_tolerance_rad:
            yaw_rate = 0.0
        if up_mps == 0.0:
            self.last_velocity_command = (*self.last_velocity_command[:2], 0.0)
        command = self._limited_velocity_command(snapshot.now_s, (east, north, up_mps))
        return replace(command, yaw_rate_rad_s=yaw_rate)

    def _tick_guided_land(self, snapshot: VehicleSnapshot) -> ActionCommand:
        """One CH8 action: GUIDED track/descent, bounded recovery, native LAND."""
        assert self.request is not None
        mode = snapshot.mode.strip().upper()
        tag_valid = bool(
            snapshot.candidate_fresh
            and snapshot.candidate_velocity_flu is not None
            and all(math.isfinite(v) for v in snapshot.candidate_velocity_flu)
        )
        correction_valid = tag_valid and self._landing_observation_valid(snapshot)
        range_valid = bool(snapshot.range_fresh and math.isfinite(snapshot.range_m)
                           and snapshot.range_m >= 0.0)
        # ROS Range is float32: nominal 0.10 arrives as 0.10000000149.
        # Micrometre normalization avoids missing the handoff at that boundary.
        range_m = round(snapshot.range_m, 6) if range_valid else math.nan
        released = bool(self.request.params.get("rc_managed", False) and
                        (snapshot.landing_explicit_low or not snapshot.landing_requested))

        # Strictly BELOW the threshold, using fresh range evidence. Once sent,
        # the disarm phase persists until the FCU armed=false heartbeat.
        if (self.land_phase == "NEAR_GROUND_DISARM" or
                (range_valid and range_m < self.land_recovery_height_m
                 and not tag_valid)):
            self.land_phase = "NEAR_GROUND_DISARM"
            self._land_expected_mode = self.land_mode
            self.detail = "NEAR_GROUND_TAG_LOST_WAIT_DISARM"
            return ActionCommand(desired_mode=self.land_mode, request_disarm=True)

        # Near-ground handoff is latched; CH8 cannot restart FOLLOW here.
        if self.land_phase == "HYBRID_LAND_COMMITTED":
            self._land_expected_mode = self.land_mode
            self.detail = "LAND_COMMITTED"
            return ActionCommand(desired_mode=self.land_mode)

        # At the handoff height, native LAND must win over Tag-pose recovery.
        # The camera often loses a usable landing pose at touchdown even while
        # it still detects the Tag, so recovery here can strand FCU in GUIDED.
        if range_valid and range_m <= self.land_recovery_height_m:
            self.land_phase = "HYBRID_LAND_COMMITTED"
            self._land_expected_mode = self.land_mode
            self.detail = "GUIDED_DESCENT_COMPLETE_REQUEST_LAND"
            return ActionCommand(desired_mode=self.land_mode)

        if self.land_phase == "GUIDED_REACQUIRE_HOLD":
            assert self.land_hold_started_s is not None
            # Deadline wins even if the first reacquired frame arrives this tick.
            if snapshot.now_s - self.land_hold_started_s >= self.land_recovery_timeout_s:
                return self._begin_land_exit(
                    self.guided_mode if tag_valid else self.fallback_mode,
                    "LAND_EXIT_TAG_REACQUIRE_TIMEOUT",
                )
            if correction_valid:
                if self.land_reacquired_since_s is None:
                    self.land_reacquired_since_s = snapshot.now_s
                if snapshot.now_s - self.land_reacquired_since_s >= self.land_reacquire_dwell_s:
                    self.land_phase = "GUIDED_TRACK_DESCENT"
                    self.land_hold_started_s = self.land_reacquired_since_s = None
            else:
                self.land_reacquired_since_s = None
            if self.land_phase == "GUIDED_REACQUIRE_HOLD":
                return self._guided_land_hold(snapshot, "LANDING_POSE_INVALID_GUIDED_HOLD" if tag_valid else "TAG_LOST_GUIDED_HOLD")

        if released:
            return self._begin_land_exit(
                self.guided_mode if tag_valid else self.fallback_mode,
                "LAND_EXIT_CH8_RELEASED",
            )
        if not range_valid:
            return self._guided_land_hold(snapshot, "RANGE_INVALID_GUIDED_HOLD")
        if tag_valid and not correction_valid:
            self.land_phase = "GUIDED_REACQUIRE_HOLD"
            self.land_hold_started_s = snapshot.now_s
            self.land_reacquired_since_s = None
            return self._guided_land_hold(snapshot, "LANDING_POSE_INVALID_GUIDED_HOLD")
        if not tag_valid:
            self.land_phase = "GUIDED_REACQUIRE_HOLD"
            self.land_hold_started_s = snapshot.now_s
            self.land_reacquired_since_s = None
            return self._guided_land_hold(snapshot, "TAG_LOST_GUIDED_HOLD")
        if mode != self.guided_mode:
            return self._guided_land_hold(snapshot, "WAITING_GUIDED_HEARTBEAT")
        self.detail = "GUIDED_TRACK_DESCENT"
        self._land_expected_mode = self.guided_mode
        return self._landing_tracking_command(snapshot, -self.land_guided_descent_mps)

    def _tick_disarm(self, snapshot: VehicleSnapshot) -> ActionCommand:
        if not snapshot.armed:
            self._finish(
                ActionState.DONE,
                "DISARM_CONFIRMED",
                "CONTROL_RELEASED",
                retain_hold=False,
            )
            return ActionCommand()
        self.detail = "WAITING_DISARM_HEARTBEAT"
        return ActionCommand(request_disarm=True)

    def _tick_emergency_stop(self, snapshot: VehicleSnapshot) -> ActionCommand:
        self.detail = "EMERGENCY_STOP_REQUESTED_UNVERIFIED"
        return ActionCommand(request_emergency_stop=True)

    def _precondition_rejection(
        self, request: ActionRequest, snapshot: VehicleSnapshot
    ) -> Optional[str]:
        kind = request.kind
        if not snapshot.connected:
            return "REJECTED_NO_FRESH_FCU_STATE"
        if not snapshot.authorized and kind != ActionKind.EMERGENCY_STOP:
            return "REJECTED_NOT_AUTHORIZED"
        if kind in FLIGHT_ACTIONS:
            if not snapshot.telemetry_fresh:
                return "REJECTED_STALE_FLIGHT_TELEMETRY"
            if not all(math.isfinite(value) for value in (
                *snapshot.position_enu,
                *snapshot.velocity_enu,
                snapshot.yaw_rad,
                snapshot.tilt_deg,
            )):
                return "REJECTED_NONFINITE_FLIGHT_TELEMETRY"
            if not snapshot.armed:
                return "REJECTED_NOT_ARMED"
            if not snapshot.ekf_healthy:
                return "REJECTED_EKF_UNHEALTHY"
            if not snapshot.home_set:
                return "REJECTED_HOME_UNSET"
        if kind == ActionKind.LAND:
            if snapshot.mode.strip().upper() != self.guided_mode:
                return "REJECTED_LAND_REQUIRES_GUIDED"
            if request.params.get("precision", True) is True:
                if not snapshot.landing_target_output_enabled:
                    return "REJECTED_LANDING_TARGET_OUTPUT_DISABLED"
                if not snapshot.landing_target_fresh:
                    return "REJECTED_LANDING_TARGET_UNHEALTHY"
        if kind == ActionKind.DISARM:
            if not snapshot.armed:
                return "REJECTED_ALREADY_DISARMED"
            if not snapshot.landed:
                return "REJECTED_NOT_LANDED"
        return None

    def _runtime_rejection(self, kind: ActionKind, snapshot: VehicleSnapshot) -> Optional[str]:
        if not snapshot.connected:
            return "FCU_STATE_LOST"
        if kind in FLIGHT_ACTIONS:
            if not snapshot.telemetry_fresh:
                return "FLIGHT_TELEMETRY_LOST"
            if not all(math.isfinite(value) for value in (
                *snapshot.position_enu,
                *snapshot.velocity_enu,
                snapshot.yaw_rad,
                snapshot.tilt_deg,
            )):
                return "NONFINITE_FLIGHT_TELEMETRY"
            if not snapshot.armed:
                return "UNEXPECTED_DISARM"
            if not snapshot.ekf_healthy:
                return "EKF_BECAME_UNHEALTHY"
        if kind == ActionKind.DISARM and not snapshot.landed:
            return "LANDING_EVIDENCE_WITHDRAWN"
        return None

    def _can_command_guided(self, snapshot: VehicleSnapshot) -> bool:
        return bool(snapshot.telemetry_fresh and snapshot.connected and snapshot.armed
                    and snapshot.authorized and snapshot.ekf_healthy
                    and snapshot.mode.strip().upper() == self.guided_mode)

    def _desired_mode(self, kind: ActionKind) -> str:
        if kind in GUIDED_ACTIONS:
            return self.guided_mode
        if kind in LAND_ACTIONS:
            return self._land_expected_mode
        raise ValueError(f"{kind.value} has no flight mode")

    def _dwell_complete(self, now_s: float, within: bool) -> bool:
        if not within:
            self.completed_since = None
            return False
        if self.completed_since is None:
            self.completed_since = now_s
        return now_s - self.completed_since >= self.completion_dwell_s

    def _limited_velocity_command(
        self,
        now_s: float,
        requested: tuple[float, float, float],
    ) -> ActionCommand:
        previous = self.last_velocity_command
        if self.last_velocity_command_s is None:
            delta_s = 0.0
        else:
            delta_s = min(0.2, max(0.0, now_s - self.last_velocity_command_s))
        dx = requested[0] - previous[0]
        dy = requested[1] - previous[1]
        horizontal_delta = math.hypot(dx, dy)
        maximum_horizontal_delta = (
            self.maximum_horizontal_acceleration_mps2 * delta_s
        )
        if horizontal_delta > maximum_horizontal_delta > 0.0:
            scale = maximum_horizontal_delta / horizontal_delta
            dx, dy = dx * scale, dy * scale
        elif maximum_horizontal_delta <= 0.0:
            dx = dy = 0.0
        dz = requested[2] - previous[2]
        maximum_vertical_delta = self.maximum_vertical_acceleration_mps2 * delta_s
        dz = max(-maximum_vertical_delta, min(maximum_vertical_delta, dz))
        limited = (previous[0] + dx, previous[1] + dy, previous[2] + dz)
        self.last_velocity_command = limited
        self.last_velocity_command_s = now_s
        return ActionCommand(desired_mode=self.guided_mode, velocity_enu=limited)

    def _finish(self, state: ActionState, reason: str, detail: str, *, retain_hold: bool) -> None:
        self.state = state
        self.reason = reason
        self.detail = detail
        self._terminal_command = (
            ActionCommand(
                desired_mode=self.guided_mode,
                velocity_enu=(0.0, 0.0, 0.0),
                yaw_rate_rad_s=0.0,
            )
            if retain_hold else None
        )

    def _status(self, now_s: float, *, reason: Optional[str] = None,
                detail: Optional[str] = None) -> ActionStatus:
        elapsed = 0.0
        if self.started_s is not None and math.isfinite(now_s):
            elapsed = max(0.0, now_s - self.started_s)
        request = self.request
        return ActionStatus(
            action_id=request.action_id if request else None,
            action=request.kind.value if request else None,
            state=self.state,
            reason=self.reason if reason is None else reason,
            detail=self.detail if detail is None else detail,
            elapsed_s=elapsed,
            control_retained=self._terminal_command is not None,
        )

    @staticmethod
    def _validate_snapshot_clock(snapshot: VehicleSnapshot) -> None:
        if not math.isfinite(snapshot.now_s):
            raise ValueError("snapshot clock must be finite")


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(value: Any, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain three numbers")
    return tuple(_finite_float(item, name) for item in value)  # type: ignore[return-value]


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
