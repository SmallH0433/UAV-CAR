#!/usr/bin/env python3
"""Safety state machine for an RC-authorized AprilTag follow controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FollowState(str, Enum):
    DISABLED = "DISABLED"
    OBSERVE = "OBSERVE"
    ACQUIRE = "ACQUIRE"
    FOLLOW_XY = "FOLLOW_XY"
    PREDICT_DECEL = "PREDICT_DECEL"
    HOLD = "HOLD"
    PILOT_OVERRIDE = "PILOT_OVERRIDE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class FollowInputs:
    timestamp_s: float
    armed: bool
    mode: str
    rc_enable: bool
    ekf_position_ok: bool
    battery_ok: bool
    altitude_ok: bool
    target_acquired: bool
    target_age_s: float
    critical_fault: bool = False


@dataclass(frozen=True)
class FollowDecision:
    state: FollowState
    may_send_velocity: bool
    velocity_scale: float
    reason: str


class FollowSafetyStateMachine:
    def __init__(self, *, predict_s: float = 0.25, hold_s: float = 0.7) -> None:
        if predict_s <= 0 or hold_s <= predict_s:
            raise ValueError("invalid target-loss timing")
        self.predict_s = predict_s
        self.hold_s = hold_s
        self.state = FollowState.DISABLED
        self._hold_latched = False

    def update(self, inputs: FollowInputs) -> FollowDecision:
        if inputs.critical_fault:
            self.state = FollowState.ABORT
            return self._decision(False, 0.0, "CRITICAL_FAULT")
        if not inputs.rc_enable:
            self.state = FollowState.DISABLED
            self._hold_latched = False
            return self._decision(False, 0.0, "RC_DISABLED")
        if inputs.armed and inputs.mode.upper() != "GUIDED":
            self.state = FollowState.PILOT_OVERRIDE
            return self._decision(False, 0.0, "MODE_NOT_GUIDED")
        if not inputs.armed:
            self.state = FollowState.OBSERVE
            return self._decision(False, 0.0, "DISARMED_OBSERVE_ONLY")
        if not inputs.ekf_position_ok:
            self.state = FollowState.ABORT
            return self._decision(False, 0.0, "EKF_POSITION_INVALID")
        if not inputs.battery_ok:
            self.state = FollowState.ABORT
            return self._decision(False, 0.0, "BATTERY_INVALID")
        if not inputs.altitude_ok:
            self.state = FollowState.ABORT
            return self._decision(False, 0.0, "ALTITUDE_OUT_OF_RANGE")
        if self._hold_latched:
            self.state = FollowState.HOLD
            return self._decision(True, 0.0, "HOLD_REQUIRES_RC_REENABLE")
        if not inputs.target_acquired:
            self.state = FollowState.ACQUIRE
            return self._decision(True, 0.0, "TARGET_NOT_ACQUIRED")
        if inputs.target_age_s <= self.predict_s:
            self.state = FollowState.FOLLOW_XY
            return self._decision(True, 1.0, "TARGET_FRESH")
        if inputs.target_age_s <= self.hold_s:
            self.state = FollowState.PREDICT_DECEL
            scale = 1.0 - (
                (inputs.target_age_s - self.predict_s)
                / (self.hold_s - self.predict_s)
            )
            return self._decision(True, max(0.0, min(1.0, scale)), "TARGET_STALE_DECEL")
        self._hold_latched = True
        self.state = FollowState.HOLD
        return self._decision(True, 0.0, "TARGET_LOST")

    def _decision(self, may_send: bool, scale: float, reason: str) -> FollowDecision:
        return FollowDecision(self.state, may_send, scale, reason)
