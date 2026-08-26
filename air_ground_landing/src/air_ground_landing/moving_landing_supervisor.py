"""Fail-closed coordinator for rendezvous, tracking, descent and touchdown."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Mapping, Optional

from .math3d import horizontal_norm, subtract
from .models import MovingPadEstimate, UavState, UgvState


class LandingState(str, Enum):
    IDLE = "IDLE"
    RENDEZVOUS = "RENDEZVOUS"
    TRACK_PAD = "TRACK_PAD"
    MATCH_VELOCITY = "MATCH_VELOCITY"
    DESCEND = "DESCEND"
    FINAL_APPROACH = "FINAL_APPROACH"
    TOUCHDOWN = "TOUCHDOWN"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class SupervisorConfig:
    minimum_pad_quality: float = 0.45
    maximum_state_age_s: float = 0.50
    landing_target_timeout_s: float = 0.35
    landing_target_abort_s: float = 0.70
    rendezvous_xy_m: float = 1.00
    tracking_xy_m: float = 0.50
    velocity_match_xy_m: float = 0.25
    velocity_match_error_mps: float = 0.15
    velocity_match_hold_s: float = 0.60
    descent_xy_m: float = 0.18
    descent_relative_speed_mps: float = 0.12
    final_approach_height_m: float = 0.25
    final_xy_m: float = 0.08
    final_relative_speed_mps: float = 0.06
    allow_moving_touchdown: bool = False
    maximum_moving_touchdown_speed_mps: float = 0.15
    maximum_touchdown_yaw_rate_rps: float = 0.10
    stopped_speed_mps: float = 0.03
    require_independent_uav_velocity_for_moving: bool = True
    require_close_range_tag_for_moving: bool = True
    allow_range_only_final_when_stopped: bool = True
    range_only_max_height_m: float = 0.16
    minimum_rangefinder_m: float = 0.02
    maximum_rangefinder_m: float = 8.0

    @classmethod
    def from_mapping(cls, root: Mapping[str, Any]) -> "SupervisorConfig":
        values = root.get("moving_landing_supervisor", {})
        return cls(
            minimum_pad_quality=float(values.get("minimum_pad_quality", 0.45)),
            maximum_state_age_s=float(values.get("maximum_state_age_s", 0.50)),
            landing_target_timeout_s=float(values.get("landing_target_timeout_s", 0.35)),
            landing_target_abort_s=float(values.get("landing_target_abort_s", 0.70)),
            rendezvous_xy_m=float(values.get("rendezvous_xy_m", 1.00)),
            tracking_xy_m=float(values.get("tracking_xy_m", 0.50)),
            velocity_match_xy_m=float(values.get("velocity_match_xy_m", 0.25)),
            velocity_match_error_mps=float(values.get("velocity_match_error_mps", 0.15)),
            velocity_match_hold_s=float(values.get("velocity_match_hold_s", 0.60)),
            descent_xy_m=float(values.get("descent_xy_m", 0.18)),
            descent_relative_speed_mps=float(values.get("descent_relative_speed_mps", 0.12)),
            final_approach_height_m=float(values.get("final_approach_height_m", 0.25)),
            final_xy_m=float(values.get("final_xy_m", 0.08)),
            final_relative_speed_mps=float(values.get("final_relative_speed_mps", 0.06)),
            allow_moving_touchdown=bool(values.get("allow_moving_touchdown", False)),
            maximum_moving_touchdown_speed_mps=float(values.get("maximum_moving_touchdown_speed_mps", 0.15)),
            maximum_touchdown_yaw_rate_rps=float(values.get("maximum_touchdown_yaw_rate_rps", 0.10)),
            stopped_speed_mps=float(values.get("stopped_speed_mps", 0.03)),
            require_independent_uav_velocity_for_moving=bool(values.get("require_independent_uav_velocity_for_moving", True)),
            require_close_range_tag_for_moving=bool(values.get("require_close_range_tag_for_moving", True)),
            allow_range_only_final_when_stopped=bool(values.get("allow_range_only_final_when_stopped", True)),
            range_only_max_height_m=float(values.get("range_only_max_height_m", 0.16)),
            minimum_rangefinder_m=float(values.get("minimum_rangefinder_m", 0.02)),
            maximum_rangefinder_m=float(values.get("maximum_rangefinder_m", 8.0)),
        )


@dataclass(frozen=True)
class SupervisorInputs:
    timestamp_s: float
    mission_enabled: bool
    operator_authorized: bool
    pilot_override: bool
    descent_requested: bool
    uav: UavState
    ugv: UgvState
    pad: Optional[MovingPadEstimate]
    landing_target_age_s: Optional[float]
    rangefinder_distance_m: Optional[float] = None
    close_range_tag_visible: bool = False
    contact_confirmed: bool = False


@dataclass(frozen=True)
class SupervisorDecision:
    state: LandingState
    reason: str
    publish_landing_target: bool
    request_land_mode: bool
    request_hold_mode: bool
    request_ugv_stop: bool
    descent_authorized: bool
    abort_action: Optional[str]
    horizontal_error_m: Optional[float]
    relative_speed_mps: Optional[float]
    height_above_pad_m: Optional[float]
    descent_requested: bool = False

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["state"] = self.state.value
        return result


class MovingLandingSupervisor:
    """State machine that emits requests but never actuates either vehicle."""

    def __init__(self, config: SupervisorConfig) -> None:
        self.config = config
        self.state = LandingState.IDLE
        self._state_entered_s = 0.0
        self._velocity_matched_since_s: Optional[float] = None
        self._last_timestamp_s: Optional[float] = None
        self._last_reason = "NOT_STARTED"

    def reset(self) -> None:
        self.__init__(self.config)

    def step(self, inputs: SupervisorInputs) -> SupervisorDecision:
        now_s = float(inputs.timestamp_s)
        if self._last_timestamp_s is not None and now_s < self._last_timestamp_s:
            if self.state != LandingState.IDLE:
                self._transition(LandingState.ABORT, now_s, "TIME_MOVED_BACKWARD")
            return self._decision(inputs, "TIME_MOVED_BACKWARD")
        self._last_timestamp_s = now_s

        if self.state == LandingState.ABORT:
            if (
                not inputs.mission_enabled
                and inputs.uav.landed is True
                and not inputs.uav.armed
            ):
                self._transition(LandingState.IDLE, now_s, "ABORT_RESET_SAFE")
            return self._decision(inputs, self._last_reason)

        if inputs.pilot_override:
            self._transition(LandingState.ABORT, now_s, "PILOT_OVERRIDE")
            return self._decision(inputs, self._last_reason)

        active = self.state not in (LandingState.IDLE, LandingState.COMPLETE)
        if active and (not inputs.mission_enabled or not inputs.operator_authorized):
            self._transition(LandingState.ABORT, now_s, "MISSION_AUTHORIZATION_REMOVED")
            return self._decision(inputs, self._last_reason)

        health_fault = self._health_fault(inputs)
        if health_fault is not None:
            if active:
                self._transition(LandingState.ABORT, now_s, health_fault)
            else:
                self._last_reason = f"WAIT_{health_fault}"
            return self._decision(inputs, self._last_reason)

        if self.state == LandingState.COMPLETE:
            if (
                not inputs.mission_enabled
                and inputs.uav.landed is True
                and not inputs.uav.armed
            ):
                self._transition(LandingState.IDLE, now_s, "MISSION_RESET_SAFE")
            return self._decision(inputs, self._last_reason)

        if self.state == LandingState.IDLE:
            if not inputs.mission_enabled or not inputs.operator_authorized:
                self._last_reason = "WAIT_MISSION_AUTHORIZATION"
                return self._decision(inputs, self._last_reason)
            if not inputs.uav.armed:
                self._last_reason = "WAIT_UAV_ARMED"
                return self._decision(inputs, self._last_reason)
            self._transition(LandingState.RENDEZVOUS, now_s, "MISSION_ACCEPTED")
            return self._decision(inputs, self._last_reason)

        metrics = self._metrics(inputs)
        pad_fresh = metrics[3]
        vision_fresh = self._vision_fresh(inputs)
        horizontal_error, relative_speed, height_above_pad, _ = metrics
        state_at_start = self.state

        if state_at_start == LandingState.RENDEZVOUS:
            if pad_fresh and horizontal_error is not None and horizontal_error <= self.config.rendezvous_xy_m:
                self._transition(LandingState.TRACK_PAD, now_s, "PAD_ACQUIRED")
            else:
                self._last_reason = "RENDEZVOUS_TO_PAD"

        elif state_at_start == LandingState.TRACK_PAD:
            if not pad_fresh:
                self._transition(LandingState.RENDEZVOUS, now_s, "PAD_ESTIMATE_LOST")
            elif horizontal_error is not None and horizontal_error <= self.config.tracking_xy_m:
                self._transition(LandingState.MATCH_VELOCITY, now_s, "POSITION_TRACK_ESTABLISHED")
            else:
                self._last_reason = "TRACKING_PAD_POSITION"

        elif state_at_start == LandingState.MATCH_VELOCITY:
            matched = bool(
                pad_fresh
                and vision_fresh
                and horizontal_error is not None
                and relative_speed is not None
                and horizontal_error <= self.config.velocity_match_xy_m
                and relative_speed <= self.config.velocity_match_error_mps
            )
            if not pad_fresh:
                self._transition(LandingState.TRACK_PAD, now_s, "PAD_ESTIMATE_LOST")
            elif matched:
                if self._velocity_matched_since_s is None:
                    self._velocity_matched_since_s = now_s
                if not inputs.descent_requested:
                    self._last_reason = "WAIT_SWD_DESCENT_REQUEST"
                elif now_s - self._velocity_matched_since_s >= self.config.velocity_match_hold_s:
                    self._transition(LandingState.DESCEND, now_s, "VELOCITY_MATCH_STABLE")
                else:
                    self._last_reason = "VERIFYING_VELOCITY_MATCH"
            else:
                self._velocity_matched_since_s = None
                self._last_reason = "MATCHING_PAD_VELOCITY"

        elif state_at_start == LandingState.DESCEND:
            if not inputs.descent_requested:
                self._transition(
                    LandingState.MATCH_VELOCITY,
                    now_s,
                    "SWD_DESCENT_CANCELLED_RESUME_FOLLOW",
                )
            elif not pad_fresh:
                self._transition(LandingState.ABORT, now_s, "PAD_ESTIMATE_STALE_DURING_DESCENT")
            elif not vision_fresh:
                age = math.inf if inputs.landing_target_age_s is None else inputs.landing_target_age_s
                if age > self.config.landing_target_abort_s:
                    self._transition(LandingState.ABORT, now_s, "LANDING_TARGET_LOST_DURING_DESCENT")
                else:
                    self._last_reason = "LANDING_TARGET_TEMPORARILY_LOST_HOLD"
            elif height_above_pad is not None and height_above_pad <= self.config.final_approach_height_m:
                self._transition(LandingState.FINAL_APPROACH, now_s, "ENTER_FINAL_APPROACH")
            elif self._descent_alignment(horizontal_error, relative_speed):
                self._last_reason = "DESCENT_AUTHORIZED"
            else:
                self._last_reason = "DESCENT_PAUSED_FOR_ALIGNMENT"

        elif state_at_start == LandingState.FINAL_APPROACH:
            if inputs.contact_confirmed or inputs.uav.landed is True:
                self._transition(LandingState.TOUCHDOWN, now_s, "TOUCHDOWN_CONFIRMED")
            elif not inputs.descent_requested:
                self._transition(
                    LandingState.MATCH_VELOCITY,
                    now_s,
                    "SWD_DESCENT_CANCELLED_RESUME_FOLLOW",
                )
            elif not pad_fresh:
                self._transition(LandingState.ABORT, now_s, "PAD_ESTIMATE_STALE_IN_FINAL")
            elif self._final_descent_ready(inputs, horizontal_error, relative_speed, height_above_pad):
                self._last_reason = "FINAL_DESCENT_AUTHORIZED"
            else:
                self._last_reason = "FINAL_APPROACH_HOLD"

        elif state_at_start == LandingState.TOUCHDOWN:
            if inputs.uav.landed is True and not inputs.uav.armed:
                self._transition(LandingState.COMPLETE, now_s, "LANDED_AND_DISARMED")
            else:
                self._last_reason = "WAIT_AUTOPILOT_DISARM"

        return self._decision(inputs, self._last_reason)

    def _health_fault(self, inputs: SupervisorInputs) -> Optional[str]:
        now_s = float(inputs.timestamp_s)
        if not inputs.uav.link_healthy:
            return "UAV_LINK_UNHEALTHY"
        if not inputs.ugv.healthy:
            return "UGV_STATE_UNHEALTHY"
        if inputs.ugv.emergency_stop:
            return "UGV_EMERGENCY_STOP"
        if now_s - inputs.uav.timestamp_s > self.config.maximum_state_age_s:
            return "UAV_STATE_STALE"
        if now_s - inputs.ugv.timestamp_s > self.config.maximum_state_age_s:
            return "UGV_STATE_STALE"
        return None

    def _metrics(
        self,
        inputs: SupervisorInputs,
    ) -> tuple[Optional[float], Optional[float], Optional[float], bool]:
        pad = inputs.pad
        if pad is None:
            return None, None, None, False
        source_fresh = bool(pad.sources) and (
            (pad.vision_age_s is not None and pad.vision_age_s <= self.config.maximum_state_age_s)
            or (pad.ugv_age_s is not None and pad.ugv_age_s <= self.config.maximum_state_age_s)
        )
        pad_fresh = pad.quality >= self.config.minimum_pad_quality and source_fresh
        relative_position = subtract(pad.position_ned_m, inputs.uav.position_ned_m)
        relative_velocity = subtract(pad.velocity_ned_mps, inputs.uav.velocity_ned_mps)
        return (
            horizontal_norm(relative_position),
            horizontal_norm(relative_velocity),
            relative_position[2],
            pad_fresh,
        )

    def _vision_fresh(self, inputs: SupervisorInputs) -> bool:
        age = inputs.landing_target_age_s
        return age is not None and 0.0 <= age <= self.config.landing_target_timeout_s

    def _descent_alignment(
        self,
        horizontal_error: Optional[float],
        relative_speed: Optional[float],
    ) -> bool:
        return bool(
            horizontal_error is not None
            and relative_speed is not None
            and horizontal_error <= self.config.descent_xy_m
            and relative_speed <= self.config.descent_relative_speed_mps
        )

    def _rangefinder_valid(self, distance_m: Optional[float]) -> bool:
        return bool(
            distance_m is not None
            and math.isfinite(distance_m)
            and self.config.minimum_rangefinder_m <= distance_m <= self.config.maximum_rangefinder_m
        )

    def _final_descent_ready(
        self,
        inputs: SupervisorInputs,
        horizontal_error: Optional[float],
        relative_speed: Optional[float],
        height_above_pad: Optional[float],
    ) -> bool:
        aligned = bool(
            horizontal_error is not None
            and relative_speed is not None
            and horizontal_error <= self.config.final_xy_m
            and relative_speed <= self.config.final_relative_speed_mps
        )
        if not aligned or not self._rangefinder_valid(inputs.rangefinder_distance_m):
            return False
        if self.config.allow_moving_touchdown:
            if inputs.ugv.horizontal_speed_mps > self.config.maximum_moving_touchdown_speed_mps:
                return False
            if abs(inputs.ugv.yaw_rate_rps) > self.config.maximum_touchdown_yaw_rate_rps:
                return False
            if (
                self.config.require_independent_uav_velocity_for_moving
                and not inputs.uav.velocity_source_independent_of_deck
            ):
                return False
            if self.config.require_close_range_tag_for_moving and not inputs.close_range_tag_visible:
                return False
            return self._vision_fresh(inputs)

        if inputs.ugv.horizontal_speed_mps > self.config.stopped_speed_mps:
            return False
        if self._vision_fresh(inputs):
            return True
        return bool(
            self.config.allow_range_only_final_when_stopped
            and height_above_pad is not None
            and 0.0 <= height_above_pad <= self.config.range_only_max_height_m
        )

    def _transition(self, state: LandingState, now_s: float, reason: str) -> None:
        if state != self.state:
            self.state = state
            self._state_entered_s = float(now_s)
            if state != LandingState.MATCH_VELOCITY:
                self._velocity_matched_since_s = None
        self._last_reason = reason

    def _decision(self, inputs: SupervisorInputs, reason: str) -> SupervisorDecision:
        horizontal_error, relative_speed, height_above_pad, pad_fresh = self._metrics(inputs)
        vision_fresh = self._vision_fresh(inputs)
        descent_authorized = False
        request_hold = False
        if self.state == LandingState.DESCEND:
            descent_authorized = bool(
                pad_fresh
                and vision_fresh
                and self._descent_alignment(horizontal_error, relative_speed)
            )
            request_hold = not descent_authorized
        elif self.state == LandingState.FINAL_APPROACH:
            descent_authorized = bool(
                pad_fresh
                and self._final_descent_ready(
                    inputs,
                    horizontal_error,
                    relative_speed,
                    height_above_pad,
                )
            )
            request_hold = not descent_authorized
        elif self.state == LandingState.ABORT:
            request_hold = True

        request_ugv_stop = self.state in {
            LandingState.ABORT,
            LandingState.TOUCHDOWN,
            LandingState.COMPLETE,
        } or (
            self.state == LandingState.FINAL_APPROACH
            and not self.config.allow_moving_touchdown
        )
        request_land = bool(
            self.state in {LandingState.DESCEND, LandingState.FINAL_APPROACH}
            and not request_hold
            and inputs.uav.mode.upper() != "LAND"
        )
        abort_action = None
        if self.state == LandingState.ABORT:
            abort_action = "PILOT_CONTROL" if inputs.pilot_override else "HOLD_OR_STATIC_LAND"
        return SupervisorDecision(
            state=self.state,
            reason=reason,
            publish_landing_target=vision_fresh,
            request_land_mode=request_land,
            request_hold_mode=request_hold,
            request_ugv_stop=request_ugv_stop,
            descent_authorized=descent_authorized,
            abort_action=abort_action,
            horizontal_error_m=horizontal_error,
            relative_speed_mps=relative_speed,
            height_above_pad_m=height_above_pad,
            descent_requested=inputs.descent_requested,
        )
