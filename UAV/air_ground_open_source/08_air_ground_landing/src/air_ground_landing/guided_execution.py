"""Fail-closed mode transition and RC authorization helpers.

The helpers in this module have no ROS or MAVLink dependency.  ROS 2 adapters
use them to distinguish a MAVROS ``mode_sent`` response from the authoritative
mode acknowledgement carried by the next vehicle heartbeat.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Iterable, Optional


class RcGateState(str, Enum):
    MISSING = "MISSING"
    STALE = "STALE"
    ABORT = "ABORT"
    STANDBY = "STANDBY"
    AUTHORIZED = "AUTHORIZED"


@dataclass(frozen=True)
class RcGateConfig:
    channel: int = 8
    abort_below_pwm: int = 1300
    authorize_above_pwm: int = 1800
    maximum_age_s: float = 0.5

    def validate(self) -> None:
        if self.channel < 1:
            raise ValueError("RC channel is one-based and must be positive")
        if not 800 <= self.abort_below_pwm < self.authorize_above_pwm <= 2200:
            raise ValueError("RC abort/authorize PWM thresholds are invalid")
        if self.maximum_age_s <= 0.0:
            raise ValueError("RC maximum age must be positive")


@dataclass(frozen=True)
class RcGateResult:
    state: RcGateState
    pwm: Optional[int]
    age_s: Optional[float]

    @property
    def authorized(self) -> bool:
        return self.state == RcGateState.AUTHORIZED

    @property
    def abort_requested(self) -> bool:
        return self.state in (RcGateState.ABORT, RcGateState.MISSING, RcGateState.STALE)


class RcAuthorizationGate:
    """Interpret a spare RC channel as a companion-computer permission gate."""

    def __init__(self, config: RcGateConfig) -> None:
        config.validate()
        self.config = config

    def evaluate(
        self,
        channels: Optional[Iterable[int]],
        *,
        received_time_s: Optional[float],
        now_s: float,
    ) -> RcGateResult:
        if channels is None or received_time_s is None:
            return RcGateResult(RcGateState.MISSING, None, None)
        values = tuple(int(value) for value in channels)
        index = self.config.channel - 1
        if index >= len(values):
            return RcGateResult(RcGateState.MISSING, None, None)
        age_s = max(0.0, float(now_s) - float(received_time_s))
        pwm = values[index]
        if age_s > self.config.maximum_age_s:
            return RcGateResult(RcGateState.STALE, pwm, age_s)
        if pwm <= self.config.abort_below_pwm:
            return RcGateResult(RcGateState.ABORT, pwm, age_s)
        if pwm >= self.config.authorize_above_pwm:
            return RcGateResult(RcGateState.AUTHORIZED, pwm, age_s)
        return RcGateResult(RcGateState.STANDBY, pwm, age_s)


@dataclass(frozen=True)
class HorizontalVelocityLimitConfig:
    maximum_speed_mps: float = 0.10
    maximum_acceleration_mps2: float = 0.15

    def validate(self) -> None:
        if self.maximum_speed_mps <= 0.0:
            raise ValueError("maximum horizontal speed must be positive")
        if self.maximum_acceleration_mps2 <= 0.0:
            raise ValueError("maximum horizontal acceleration must be positive")


class HorizontalVelocityLimiter:
    """Limit the final horizontal velocity vector and its time derivative."""

    def __init__(self, config: HorizontalVelocityLimitConfig) -> None:
        config.validate()
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._velocity = (0.0, 0.0)
        self._updated_s: Optional[float] = None

    def apply(self, vx: float, vy: float, *, now_s: float) -> tuple[float, float]:
        requested = (float(vx), float(vy))
        if not all(math.isfinite(value) for value in requested):
            self.reset()
            return (0.0, 0.0)

        speed = math.hypot(*requested)
        if speed > self.config.maximum_speed_mps:
            ratio = self.config.maximum_speed_mps / speed
            requested = (requested[0] * ratio, requested[1] * ratio)

        if self._updated_s is None:
            self._updated_s = float(now_s)
            return self._velocity

        dt_s = max(0.0, float(now_s) - self._updated_s)
        self._updated_s = float(now_s)
        delta = (
            requested[0] - self._velocity[0],
            requested[1] - self._velocity[1],
        )
        delta_norm = math.hypot(*delta)
        maximum_delta = self.config.maximum_acceleration_mps2 * dt_s
        if delta_norm > maximum_delta and delta_norm > 1.0e-9:
            ratio = maximum_delta / delta_norm
            delta = (delta[0] * ratio, delta[1] * ratio)
        self._velocity = (
            self._velocity[0] + delta[0],
            self._velocity[1] + delta[1],
        )
        return self._velocity


class LandingSwitchState(str, Enum):
    MISSING = "MISSING"
    STALE = "STALE"
    FOLLOW_INACTIVE = "FOLLOW_INACTIVE"
    NEEDS_REARM = "NEEDS_REARM"
    READY = "READY"
    REQUESTED = "REQUESTED"


@dataclass(frozen=True)
class LandingSwitchConfig:
    channel: int = 8
    off_below_pwm: int = 1200
    on_above_pwm: int = 1800
    maximum_age_s: float = 0.5

    def validate(self) -> None:
        if self.channel < 1:
            raise ValueError("landing switch channel is one-based and must be positive")
        if not 800 <= self.off_below_pwm < self.on_above_pwm <= 2200:
            raise ValueError("landing switch PWM thresholds are invalid")
        if self.maximum_age_s <= 0.0:
            raise ValueError("landing switch maximum age must be positive")


@dataclass(frozen=True)
class LandingSwitchResult:
    state: LandingSwitchState
    pwm: Optional[int]
    age_s: Optional[float]

    @property
    def requested(self) -> bool:
        return self.state == LandingSwitchState.REQUESTED


class RcLandingRequestGate:
    """Convert a two-position SwD channel into a fail-closed descent request.

    A high switch is accepted only after follow has become active and the
    operator has first presented a valid low value.  This prevents a switch
    left high across startup or follow re-entry from causing an immediate
    descent.  Returning the switch low cancels the request without revoking
    the independent RC6 follow authorization.
    """

    def __init__(self, config: LandingSwitchConfig) -> None:
        config.validate()
        self.config = config
        self._armed_for_rising_edge = False
        self._requested = False

    def reset(self) -> None:
        self._armed_for_rising_edge = False
        self._requested = False

    @property
    def requested(self) -> bool:
        return self._requested

    def evaluate(
        self,
        channels: Optional[Iterable[int]],
        *,
        received_time_s: Optional[float],
        now_s: float,
        follow_active: bool,
    ) -> LandingSwitchResult:
        if channels is None or received_time_s is None:
            self.reset()
            return LandingSwitchResult(LandingSwitchState.MISSING, None, None)
        values = tuple(int(value) for value in channels)
        index = self.config.channel - 1
        if index >= len(values):
            self.reset()
            return LandingSwitchResult(LandingSwitchState.MISSING, None, None)
        age_s = max(0.0, float(now_s) - float(received_time_s))
        pwm = values[index]
        if age_s > self.config.maximum_age_s:
            self.reset()
            return LandingSwitchResult(LandingSwitchState.STALE, pwm, age_s)
        if not follow_active:
            self.reset()
            return LandingSwitchResult(
                LandingSwitchState.FOLLOW_INACTIVE,
                pwm,
                age_s,
            )
        if pwm <= self.config.off_below_pwm:
            self._armed_for_rising_edge = True
            self._requested = False
            return LandingSwitchResult(LandingSwitchState.READY, pwm, age_s)
        if pwm >= self.config.on_above_pwm:
            if self._requested:
                return LandingSwitchResult(LandingSwitchState.REQUESTED, pwm, age_s)
            if self._armed_for_rising_edge:
                self._requested = True
                return LandingSwitchResult(LandingSwitchState.REQUESTED, pwm, age_s)
        self._requested = False
        self._armed_for_rising_edge = False
        return LandingSwitchResult(LandingSwitchState.NEEDS_REARM, pwm, age_s)


class ModeTransitionPhase(str, Enum):
    IDLE = "IDLE"
    REQUESTING_TARGET = "REQUESTING_TARGET"
    WAITING_TARGET_HEARTBEAT = "WAITING_TARGET_HEARTBEAT"
    ACTIVE = "ACTIVE"
    REQUESTING_ROLLBACK = "REQUESTING_ROLLBACK"
    WAITING_ROLLBACK_HEARTBEAT = "WAITING_ROLLBACK_HEARTBEAT"
    FAULT = "FAULT"


@dataclass(frozen=True)
class ModeTransitionConfig:
    target_ack_timeout_s: float = 2.0
    rollback_ack_timeout_s: float = 2.0
    rollback_retry_interval_s: float = 1.0
    fallback_mode: str = "LOITER"
    previous_mode_allowlist: tuple[str, ...] = ("LOITER", "BRAKE", "POSHOLD", "ALT_HOLD")

    def validate(self) -> None:
        if self.target_ack_timeout_s <= 0.0 or self.rollback_ack_timeout_s <= 0.0:
            raise ValueError("mode acknowledgement timeouts must be positive")
        if self.rollback_retry_interval_s <= 0.0:
            raise ValueError("rollback retry interval must be positive")
        if not self.fallback_mode.strip():
            raise ValueError("fallback mode is required")


@dataclass(frozen=True)
class ModeRequest:
    sequence: int
    mode: str
    rollback: bool
    reason: str


@dataclass(frozen=True)
class ModeTransitionStatus:
    phase: ModeTransitionPhase
    desired_mode: Optional[str]
    target_mode: Optional[str]
    rollback_mode: Optional[str]
    current_mode: str
    mavros_service_ack: Optional[bool]
    heartbeat_ack: bool
    reason: str

    @property
    def setpoint_stream_authorized(self) -> bool:
        return self.phase == ModeTransitionPhase.ACTIVE and self.heartbeat_ack

    def as_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["phase"] = self.phase.value
        return result


class ModeTransitionManager:
    """Request, confirm and roll back a vehicle mode transition.

    A successful MAVROS service response is a transport acknowledgement only.
    ``ACTIVE`` is entered exclusively after the vehicle state heartbeat reports
    the requested mode.
    """

    def __init__(self, config: ModeTransitionConfig) -> None:
        config.validate()
        self.config = config
        self.phase = ModeTransitionPhase.IDLE
        self.desired_mode: Optional[str] = None
        self.target_mode: Optional[str] = None
        self.rollback_mode: Optional[str] = None
        self.current_mode = "UNKNOWN"
        self.service_ack: Optional[bool] = None
        self.reason = "NO_MODE_REQUEST"
        self._deadline_s: Optional[float] = None
        self._sequence = 0
        self._outstanding_sequence: Optional[int] = None
        self._retry_at_s: Optional[float] = None

    @staticmethod
    def _mode(value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        normalized = str(value).strip().upper()
        return normalized or None

    def update(
        self,
        *,
        now_s: float,
        current_mode: str,
        desired_mode: Optional[str],
    ) -> Optional[ModeRequest]:
        self.current_mode = self._mode(current_mode) or "UNKNOWN"
        self.desired_mode = self._mode(desired_mode)

        if self.phase == ModeTransitionPhase.FAULT:
            if self.current_mode == self.rollback_mode:
                self._reset("ROLLBACK_HEARTBEAT_CONFIRMED_AFTER_FAULT")
                return None
            if self.target_mode is not None and self.current_mode != self.target_mode:
                self._reset("EXTERNAL_MODE_OVERRIDE_AFTER_ROLLBACK_FAULT")
                return None
            if (
                self.desired_mode is None
                and self.rollback_mode is not None
                and self._retry_at_s is not None
                and float(now_s) >= self._retry_at_s
            ):
                return self._start_rollback(now_s, "RETRY_ROLLBACK_AFTER_FAULT")
            return None

        if self.phase in (
            ModeTransitionPhase.REQUESTING_ROLLBACK,
            ModeTransitionPhase.WAITING_ROLLBACK_HEARTBEAT,
        ):
            if self.current_mode == self.rollback_mode:
                self._reset("ROLLBACK_HEARTBEAT_CONFIRMED")
                return None
            if self._expired(now_s):
                self._enter_fault("ROLLBACK_HEARTBEAT_TIMEOUT", now_s)
            return None

        if self.phase == ModeTransitionPhase.ACTIVE:
            if self.current_mode != self.target_mode:
                # A pilot or another higher-level safety action changed mode.
                # Do not fight it by switching back to GUIDED.
                self._reset("EXTERNAL_MODE_OVERRIDE")
                return None
            if self.desired_mode is None:
                return self._start_rollback(now_s, "AUTHORITY_REVOKED_OR_TARGET_CHANGED")
            if self.desired_mode != self.target_mode:
                return self._start_target_change(now_s, self.desired_mode)
            return None

        if self.phase in (
            ModeTransitionPhase.REQUESTING_TARGET,
            ModeTransitionPhase.WAITING_TARGET_HEARTBEAT,
        ):
            if self.current_mode == self.target_mode:
                self.phase = ModeTransitionPhase.ACTIVE
                self.reason = "TARGET_HEARTBEAT_ACK"
                self._deadline_s = None
                self._outstanding_sequence = None
                return None
            if self.desired_mode is None:
                return self._start_rollback(now_s, "AUTHORITY_REVOKED_DURING_TRANSITION")
            if self.desired_mode != self.target_mode:
                return self._start_target_change(now_s, self.desired_mode)
            if self._expired(now_s):
                return self._start_rollback(now_s, "TARGET_HEARTBEAT_TIMEOUT")
            return None

        if self.desired_mode is None:
            self.reason = "NO_MODE_REQUEST"
            return None

        self.target_mode = self.desired_mode
        previous = self.current_mode
        self.rollback_mode = (
            previous
            if previous in {mode.upper() for mode in self.config.previous_mode_allowlist}
            else self.config.fallback_mode.upper()
        )
        if self.current_mode == self.target_mode:
            self.phase = ModeTransitionPhase.ACTIVE
            self.reason = "TARGET_ALREADY_CONFIRMED_BY_HEARTBEAT"
            return None
        return self._new_request(
            self.target_mode,
            rollback=False,
            reason="REQUEST_TARGET_MODE",
            now_s=now_s,
        )

    def on_service_result(
        self,
        *,
        sequence: int,
        mode_sent: bool,
        now_s: float,
    ) -> Optional[ModeRequest]:
        if sequence != self._outstanding_sequence:
            return None
        self._outstanding_sequence = None
        self.service_ack = bool(mode_sent)
        if self.phase == ModeTransitionPhase.REQUESTING_TARGET:
            if not mode_sent:
                return self._start_rollback(now_s, "MAVROS_TARGET_MODE_NACK")
            self.phase = ModeTransitionPhase.WAITING_TARGET_HEARTBEAT
            self.reason = "MAVROS_TARGET_SENT_WAIT_HEARTBEAT"
            self._deadline_s = now_s + self.config.target_ack_timeout_s
            return None
        if self.phase == ModeTransitionPhase.REQUESTING_ROLLBACK:
            if not mode_sent:
                self._enter_fault("MAVROS_ROLLBACK_MODE_NACK", now_s)
                return None
            self.phase = ModeTransitionPhase.WAITING_ROLLBACK_HEARTBEAT
            self.reason = "MAVROS_ROLLBACK_SENT_WAIT_HEARTBEAT"
            self._deadline_s = now_s + self.config.rollback_ack_timeout_s
        return None

    def status(self) -> ModeTransitionStatus:
        heartbeat_ack = bool(
            self.target_mode is not None
            and self.current_mode == self.target_mode
            and self.phase == ModeTransitionPhase.ACTIVE
        )
        return ModeTransitionStatus(
            phase=self.phase,
            desired_mode=self.desired_mode,
            target_mode=self.target_mode,
            rollback_mode=self.rollback_mode,
            current_mode=self.current_mode,
            mavros_service_ack=self.service_ack,
            heartbeat_ack=heartbeat_ack,
            reason=self.reason,
        )

    def _new_request(
        self,
        mode: str,
        *,
        rollback: bool,
        reason: str,
        now_s: float,
    ) -> ModeRequest:
        self._sequence += 1
        self._outstanding_sequence = self._sequence
        self.service_ack = None
        self.phase = (
            ModeTransitionPhase.REQUESTING_ROLLBACK
            if rollback
            else ModeTransitionPhase.REQUESTING_TARGET
        )
        self.reason = reason
        timeout = (
            self.config.rollback_ack_timeout_s
            if rollback
            else self.config.target_ack_timeout_s
        )
        self._deadline_s = now_s + timeout
        self._retry_at_s = None
        return ModeRequest(self._sequence, mode, rollback, reason)

    def _start_rollback(self, now_s: float, reason: str) -> Optional[ModeRequest]:
        rollback_mode = self.rollback_mode or self.config.fallback_mode.upper()
        if self.current_mode == rollback_mode:
            self._reset(f"{reason}_ALREADY_IN_ROLLBACK_MODE")
            return None
        return self._new_request(
            rollback_mode,
            rollback=True,
            reason=reason,
            now_s=now_s,
        )

    def _start_target_change(self, now_s: float, mode: str) -> Optional[ModeRequest]:
        self.target_mode = mode
        if self.current_mode == mode:
            self.phase = ModeTransitionPhase.ACTIVE
            self.service_ack = None
            self._deadline_s = None
            self._outstanding_sequence = None
            self.reason = "CHANGED_TARGET_ALREADY_CONFIRMED_BY_HEARTBEAT"
            return None
        return self._new_request(
            mode,
            rollback=False,
            reason="REQUEST_CHANGED_TARGET_MODE",
            now_s=now_s,
        )

    def _expired(self, now_s: float) -> bool:
        return self._deadline_s is not None and float(now_s) >= self._deadline_s

    def _enter_fault(self, reason: str, now_s: float) -> None:
        self.phase = ModeTransitionPhase.FAULT
        self.reason = reason
        self._deadline_s = None
        self._outstanding_sequence = None
        self._retry_at_s = float(now_s) + self.config.rollback_retry_interval_s

    def _reset(self, reason: str) -> None:
        self.phase = ModeTransitionPhase.IDLE
        self.target_mode = None
        self.rollback_mode = None
        self.service_ack = None
        self._deadline_s = None
        self._outstanding_sequence = None
        self._retry_at_s = None
        self.reason = reason
