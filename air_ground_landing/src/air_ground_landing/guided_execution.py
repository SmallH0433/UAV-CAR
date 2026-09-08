"""Fail-closed mode transition and RC authorization helpers.

The helpers in this module have no ROS or MAVLink dependency.  ROS 2 adapters
use them to distinguish a MAVROS ``mode_sent`` response from the authoritative
mode acknowledgement carried by the next vehicle heartbeat.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from enum import Enum
import math
import statistics
from typing import Iterable, Optional


FOLLOW_ENTRY_MODES = frozenset({"ALT_HOLD", "LOITER"})


class PilotSessionGate:
    """Operator session, separate from retryable mode transactions.

    A new session requires fresh low RC samples spanning the dwell, then high.
    External mode changes invalidate it even between executor timer ticks.
    """

    def __init__(self, low_dwell_s: float = 0.4) -> None:
        if not math.isfinite(low_dwell_s) or low_dwell_s <= 0:
            raise ValueError("RC rearm dwell must be finite and positive")
        self.low_dwell_s = low_dwell_s
        self.enabled = False
        self.reason = "STARTUP_REARM_REQUIRED"
        self.session_sequence = 0
        self.mode = None
        self.low_since = None
        self.low_last = None
        self.low_ready = False
        self.boundary = -math.inf
        self.last_update_s: Optional[float] = None

    def invalidate(self, reason: str, now_s: float) -> None:
        self.enabled = False
        self.reason = reason
        self.low_since = self.low_last = None
        self.low_ready = False
        self.boundary = now_s

    def observe_mode(self, mode: str, *, expected_mode: Optional[str], now_s: float,
                     watch_transition: bool = False) -> bool:
        mode = str(mode).strip().upper()
        expected_mode = None if expected_mode is None else str(expected_mode).strip().upper()
        override = ((self.enabled or watch_transition or self.low_since is not None)
                    and self.mode is not None and mode != self.mode
                    and mode != expected_mode)
        self.mode = mode
        if override:
            self.invalidate("PILOT_OVERRIDE_LOCKOUT", now_s)
        return bool(override)

    def update(self, *, now_s: float, healthy: bool, rc: "RcGateResult",
               received_s: Optional[float], entry_allowed: bool) -> bool:
        if (not math.isfinite(now_s)
                or (self.last_update_s is not None and now_s < self.last_update_s)):
            self.invalidate("INVALID_CLOCK_REARM_REQUIRED",
                            self.last_update_s if self.last_update_s is not None else 0.0)
            return False
        self.last_update_s = now_s
        if not healthy or rc.state in (RcGateState.MISSING, RcGateState.STALE):
            self.invalidate("LINK_OR_STATE_REARM_REQUIRED", now_s)
            return False
        if (received_s is None or not math.isfinite(received_s)
                or not 0.0 <= now_s - received_s <= 0.5
                or (self.low_last is not None and received_s < self.low_last)):
            self.invalidate("INVALID_RC_TIME_REARM_REQUIRED", now_s)
            return False
        if rc.state == RcGateState.ABORT:
            self.enabled = False
            self.reason = "RC_LOW_REARMING"
            if received_s is not None and received_s > self.boundary:
                if self.low_last is not None and received_s - self.low_last > 0.5:
                    self.low_since = None
                    self.low_ready = False
                if self.low_since is None:
                    self.low_since = received_s
                self.low_last = received_s
                self.low_ready = received_s - self.low_since >= self.low_dwell_s
            return False
        if rc.state != RcGateState.AUTHORIZED:
            self.invalidate("RC_NEUTRAL_REARM_REQUIRED", now_s)
            return False
        # The high edge itself must be new and adjacent to the verified low
        # stream. A scheduler pause must not preserve an old authorization.
        fresh_high_edge = (self.low_last is not None
                           and 0.0 < received_s - self.low_last <= 0.5)
        if not self.enabled and self.low_ready and fresh_high_edge and entry_allowed:
            self.enabled = True
            self.session_sequence += 1
            self.reason = "SESSION_AUTHORIZED"
        # An early high edge cannot be queued until the mode becomes eligible.
        self.low_since = self.low_last = None
        self.low_ready = False
        return self.enabled


def follow_mode_allowed(current_mode: str, guided_mode: str = "GUIDED", land_mode: str = "LAND") -> bool:
    """Permit entry from pilot altitude/position hold, or continuation of follow/LAND."""
    return str(current_mode).strip().upper() in FOLLOW_ENTRY_MODES | {guided_mode, land_mode}


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


class FollowContinuityState(str, Enum):
    """State of one operator-authorized GUIDED follow session."""

    IDLE = "IDLE"
    WAITING_CANDIDATE = "WAITING_CANDIDATE"
    ACTIVE = "ACTIVE"
    GRACE_ZERO_HOLD = "GRACE_ZERO_HOLD"
    REACQUIRE_WAIT = "REACQUIRE_WAIT"
    REACQUIRING = "REACQUIRING"


@dataclass(frozen=True)
class FollowContinuityConfig:
    dropout_grace_s: float = 1.0
    reacquire_dwell_s: float = 0.0

    def validate(self) -> None:
        if self.dropout_grace_s <= 0.0 or self.reacquire_dwell_s < 0.0:
            raise ValueError(
                "follow dropout grace must be positive and reacquire dwell non-negative"
            )


@dataclass(frozen=True)
class FollowContinuityResult:
    state: FollowContinuityState
    keep_guided: bool
    live_candidate_allowed: bool
    zero_velocity_hold: bool
    reacquire_pending: bool
    dropout_age_s: Optional[float]
    reacquire_stable_age_s: Optional[float]
    reason: str

    @property
    def session_available(self) -> bool:
        return self.keep_guided


class FollowContinuityGuard:
    """Prevent brief vision stalls from creating a GUIDED/LOITER oscillator.

    A fresh candidate starts or refreshes a session.  A short candidate loss
    keeps GUIDED authorized, but only with a zero-velocity hold setpoint.  A
    longer loss first rolls back, then allows a new GUIDED request only after
    the RC authorization remains high and the candidate stream is continuously
    healthy for the configured reacquisition dwell.

    Missing/stale RC data and a MAVROS disconnect fail closed.  An explicit RC
    low value always cancels the session immediately.
    """

    def __init__(self, config: FollowContinuityConfig) -> None:
        config.validate()
        self.config = config
        self._session_started = False
        self._last_fresh_s: Optional[float] = None
        self._reacquire_required = False
        self._reacquire_fresh_since_s: Optional[float] = None
        self._latched_reason = "NONE"

    @property
    def reacquire_pending(self) -> bool:
        return self._reacquire_required

    def reset_from_explicit_rc_low(self) -> None:
        self._session_started = False
        self._last_fresh_s = None
        self._reacquire_required = False
        self._reacquire_fresh_since_s = None
        self._latched_reason = "NONE"

    def require_reacquire(
        self,
        *,
        now_s: float,
        reason: str,
    ) -> FollowContinuityResult:
        dropout_age_s = self._dropout_age(now_s)
        self._session_started = False
        self._reacquire_required = True
        self._reacquire_fresh_since_s = None
        self._latched_reason = str(reason).strip().upper() or "REACQUIRE_WAIT"
        return self._result(
            FollowContinuityState.REACQUIRE_WAIT,
            reason=self._latched_reason,
            dropout_age_s=dropout_age_s,
        )

    def update(
        self,
        *,
        now_s: float,
        connected: bool,
        rc_authorized: bool,
        rc_explicit_low: bool,
        fresh_control_signal: bool,
    ) -> FollowContinuityResult:
        now_s = float(now_s)

        if rc_explicit_low:
            self.reset_from_explicit_rc_low()
            return self._result(
                FollowContinuityState.IDLE,
                reason="RC_LOW_SESSION_RESET",
            )

        if not connected:
            return self.require_reacquire(now_s=now_s, reason="MAVROS_DISCONNECTED")

        if not rc_authorized:
            if self._session_started or self._reacquire_required:
                return self.require_reacquire(now_s=now_s, reason="RC_NOT_AUTHORIZED")
            return self._result(
                FollowContinuityState.IDLE,
                reason="RC_NOT_AUTHORIZED",
            )

        if self._reacquire_required:
            if not fresh_control_signal:
                self._reacquire_fresh_since_s = None
                return self._result(
                    FollowContinuityState.REACQUIRE_WAIT,
                    reason=self._latched_reason,
                    dropout_age_s=self._dropout_age(now_s),
                )
            if self._reacquire_fresh_since_s is None:
                self._reacquire_fresh_since_s = now_s
            stable_age_s = max(0.0, now_s - self._reacquire_fresh_since_s)
            if stable_age_s < self.config.reacquire_dwell_s:
                return self._result(
                    FollowContinuityState.REACQUIRING,
                    reason="HIGH_RC_WAITING_FOR_STABLE_CANDIDATE",
                    dropout_age_s=self._dropout_age(now_s),
                    reacquire_stable_age_s=stable_age_s,
                )
            self._reacquire_required = False
            self._reacquire_fresh_since_s = None
            self._session_started = True
            self._last_fresh_s = now_s
            return self._result(
                FollowContinuityState.ACTIVE,
                keep_guided=True,
                live_candidate_allowed=True,
                reason="HIGH_RC_STABLE_CANDIDATE_REACQUIRED",
                dropout_age_s=0.0,
                reacquire_stable_age_s=stable_age_s,
            )

        if fresh_control_signal:
            self._session_started = True
            self._last_fresh_s = now_s
            return self._result(
                FollowContinuityState.ACTIVE,
                keep_guided=True,
                live_candidate_allowed=True,
                reason="FRESH_CONTROL_SIGNAL",
                dropout_age_s=0.0,
            )

        if not self._session_started or self._last_fresh_s is None:
            return self._result(
                FollowContinuityState.WAITING_CANDIDATE,
                reason="WAITING_FOR_FIRST_CANDIDATE",
            )

        dropout_age_s = self._dropout_age(now_s)
        if dropout_age_s is not None and dropout_age_s <= self.config.dropout_grace_s:
            return self._result(
                FollowContinuityState.GRACE_ZERO_HOLD,
                keep_guided=True,
                zero_velocity_hold=True,
                reason="SHORT_DROPOUT_ZERO_VELOCITY_HOLD",
                dropout_age_s=dropout_age_s,
            )

        return self.require_reacquire(
            now_s=now_s,
            reason="DROPOUT_TIMEOUT_AUTO_REACQUIRE_WAIT",
        )

    def _dropout_age(self, now_s: float) -> Optional[float]:
        if self._last_fresh_s is None:
            return None
        return max(0.0, float(now_s) - self._last_fresh_s)

    @staticmethod
    def _result(
        state: FollowContinuityState,
        *,
        keep_guided: bool = False,
        live_candidate_allowed: bool = False,
        zero_velocity_hold: bool = False,
        reason: str,
        dropout_age_s: Optional[float] = None,
        reacquire_stable_age_s: Optional[float] = None,
    ) -> FollowContinuityResult:
        return FollowContinuityResult(
            state=state,
            keep_guided=keep_guided,
            live_candidate_allowed=live_candidate_allowed,
            zero_velocity_hold=zero_velocity_hold,
            reacquire_pending=state in {
                FollowContinuityState.REACQUIRE_WAIT,
                FollowContinuityState.REACQUIRING,
            },
            dropout_age_s=dropout_age_s,
            reacquire_stable_age_s=reacquire_stable_age_s,
            reason=reason,
        )


@dataclass(frozen=True)
class SlidingDistanceConfig:
    window_s: float = 0.50
    maximum_age_s: float = 0.30
    minimum_m: float = 0.02
    maximum_m: float = 8.0

    def validate(self) -> None:
        if self.window_s <= 0.0 or self.maximum_age_s <= 0.0:
            raise ValueError("distance window and maximum age must be positive")
        if not 0.0 <= self.minimum_m < self.maximum_m:
            raise ValueError("distance limits are invalid")


@dataclass(frozen=True)
class SlidingDistanceResult:
    healthy: bool
    median_m: Optional[float]
    age_s: Optional[float]
    sample_count: int


class SlidingDistanceMedian:
    """Maintain a fresh, health-gated median for a noisy range stream."""

    def __init__(self, config: SlidingDistanceConfig) -> None:
        config.validate()
        self.config = config
        self._samples: deque[tuple[float, float]] = deque()
        self._last_update_s: Optional[float] = None
        self._latest_sample_healthy = False

    def reset(self) -> None:
        self._samples.clear()
        self._last_update_s = None
        self._latest_sample_healthy = False

    def update(
        self,
        *,
        now_s: float,
        distance_m: float,
        sensor_healthy: bool,
    ) -> SlidingDistanceResult:
        now_s = float(now_s)
        distance_m = float(distance_m)
        self._last_update_s = now_s
        self._latest_sample_healthy = bool(
            sensor_healthy
            and math.isfinite(distance_m)
            and self.config.minimum_m <= distance_m <= self.config.maximum_m
        )
        if self._latest_sample_healthy:
            self._samples.append((now_s, distance_m))
        self._prune(now_s)
        return self.status(now_s)

    def status(self, now_s: float) -> SlidingDistanceResult:
        now_s = float(now_s)
        self._prune(now_s)
        age_s = (
            None
            if self._last_update_s is None
            else max(0.0, now_s - self._last_update_s)
        )
        healthy = bool(
            self._latest_sample_healthy
            and age_s is not None
            and age_s <= self.config.maximum_age_s
            and self._samples
        )
        median_m = (
            statistics.median(value for _, value in self._samples)
            if healthy
            else None
        )
        return SlidingDistanceResult(healthy, median_m, age_s, len(self._samples))

    def _prune(self, now_s: float) -> None:
        first_allowed_s = float(now_s) - self.config.window_s
        while self._samples and self._samples[0][0] < first_allowed_s:
            self._samples.popleft()


class TerminalLandState(str, Enum):
    IDLE = "IDLE"
    VERIFYING = "VERIFYING"
    LATCHED = "LATCHED"


@dataclass(frozen=True)
class TerminalLandConfig:
    rangefinder_threshold_m: float = 0.15
    inner_tag_threshold_m: float = 0.15
    evidence_maximum_age_s: float = 0.50
    dwell_s: float = 0.40

    def validate(self) -> None:
        if self.rangefinder_threshold_m <= 0.0:
            raise ValueError("terminal LAND rangefinder threshold must be positive")
        if self.inner_tag_threshold_m <= 0.0:
            raise ValueError("terminal LAND inner-tag threshold must be positive")
        if self.evidence_maximum_age_s <= 0.0 or self.dwell_s <= 0.0:
            raise ValueError("terminal LAND evidence age and dwell must be positive")


@dataclass(frozen=True)
class TerminalLandResult:
    state: TerminalLandState
    latched: bool
    reason: str
    candidate_source: Optional[str]
    candidate_age_s: Optional[float]
    rangefinder_qualifies: bool
    inner_tag_qualifies: bool


class TerminalLandLatch:
    """Keep an owned LAND session after stable, close-range confirmation.

    Entry is deliberately stricter than continuation: the vehicle must already
    be armed in heartbeat-confirmed LAND with fresh RC6 authorization and a
    fresh, high SwD request.  Once latched, vision/landing-target loss does not
    return the vehicle to GUIDED or LOITER.  Disarm, an explicit CH8 low, an
    explicit RC6 low, a link loss, or a pilot-selected non-LAND mode clears the
    latch.
    """

    def __init__(self, config: TerminalLandConfig) -> None:
        config.validate()
        self.config = config
        self.state = TerminalLandState.IDLE
        self._candidate_since_s: Optional[float] = None
        self._candidate_source: Optional[str] = None
        self._last_reason = "NOT_LATCHED"

    def reset(self, reason: str = "RESET") -> TerminalLandResult:
        self.state = TerminalLandState.IDLE
        self._candidate_since_s = None
        self._candidate_source = None
        self._last_reason = str(reason).strip().upper() or "RESET"
        return self._result(False, False, None)

    def update(
        self,
        *,
        now_s: float,
        connected: bool,
        armed: bool,
        current_mode: str,
        land_mode_confirmed: bool,
        rc_authorized: bool,
        rc_explicit_low: bool,
        swd_high_and_fresh: bool,
        rangefinder_healthy: bool,
        rangefinder_median_m: Optional[float],
        inner_tag_vertical_m: Optional[float],
        inner_tag_age_s: Optional[float],
        swd_explicit_low: bool = False,
    ) -> TerminalLandResult:
        now_s = float(now_s)
        mode = str(current_mode).strip().upper()

        if self.state == TerminalLandState.LATCHED:
            if not connected:
                return self.reset("MAVROS_DISCONNECTED")
            if not armed:
                return self.reset("DISARMED_LATCH_COMPLETE")
            if rc_explicit_low:
                return self.reset("RC6_EXPLICIT_LOW_OVERRIDE")
            if swd_explicit_low:
                return self.reset("CH8_EXPLICIT_LOW_OVERRIDE")
            if mode != "LAND":
                return self.reset("PILOT_MODE_OVERRIDE")
            self._last_reason = "TERMINAL_LAND_LATCHED"
            return self._result(True, False, None)

        entry_ready = bool(
            connected
            and armed
            and mode == "LAND"
            and land_mode_confirmed
            and rc_authorized
            and swd_high_and_fresh
        )
        if not entry_ready:
            reason = "WAIT_TERMINAL_LAND_ENTRY_CONDITIONS"
            if rc_explicit_low:
                reason = "RC6_EXPLICIT_LOW_OVERRIDE"
            return self.reset(reason)

        rangefinder_qualifies = bool(
            rangefinder_healthy
            and rangefinder_median_m is not None
            and math.isfinite(float(rangefinder_median_m))
            and float(rangefinder_median_m) <= self.config.rangefinder_threshold_m
        )
        inner_tag_qualifies = bool(
            inner_tag_vertical_m is not None
            and inner_tag_age_s is not None
            and 0.0 <= float(inner_tag_age_s) <= self.config.evidence_maximum_age_s
            and math.isfinite(float(inner_tag_vertical_m))
            and 0.0 < float(inner_tag_vertical_m) <= self.config.inner_tag_threshold_m
        )
        source = (
            "RANGEFINDER+INNER_TAG"
            if rangefinder_qualifies and inner_tag_qualifies
            else "RANGEFINDER"
            if rangefinder_qualifies
            else "INNER_TAG"
            if inner_tag_qualifies
            else None
        )
        if source is None:
            self.state = TerminalLandState.IDLE
            self._candidate_since_s = None
            self._candidate_source = None
            self._last_reason = "WAIT_CLOSE_RANGE_EVIDENCE"
            return self._result(False, False, None)

        if self._candidate_since_s is None:
            self._candidate_since_s = now_s
            self._candidate_source = source
        else:
            self._candidate_source = source
        candidate_age_s = max(0.0, now_s - self._candidate_since_s)
        if candidate_age_s < self.config.dwell_s:
            self.state = TerminalLandState.VERIFYING
            self._last_reason = "VERIFYING_CLOSE_RANGE_DWELL"
            return self._result(
                rangefinder_qualifies,
                inner_tag_qualifies,
                candidate_age_s,
            )

        self.state = TerminalLandState.LATCHED
        self._last_reason = "TERMINAL_LAND_LATCHED"
        return self._result(
            rangefinder_qualifies,
            inner_tag_qualifies,
            candidate_age_s,
        )

    def _result(
        self,
        rangefinder_qualifies: bool,
        inner_tag_qualifies: bool,
        candidate_age_s: Optional[float],
    ) -> TerminalLandResult:
        return TerminalLandResult(
            state=self.state,
            latched=self.state == TerminalLandState.LATCHED,
            reason=self._last_reason,
            candidate_source=self._candidate_source,
            candidate_age_s=candidate_age_s,
            rangefinder_qualifies=rangefinder_qualifies,
            inner_tag_qualifies=inner_tag_qualifies,
        )


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
    explicit_low: bool = False

    @property
    def requested(self) -> bool:
        return self.state == LandingSwitchState.REQUESTED


class RcLandingRequestGate:
    """Request descent on a fresh high level while follow is active.

    No preceding low/high edge is required. Low, neutral, stale RC and loss
    of follow cancel the request; session authorization is checked upstream.
    """

    def __init__(self, config: LandingSwitchConfig) -> None:
        config.validate()
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._requested = False
        self._last_received_s: Optional[float] = None
        self._last_now_s: Optional[float] = None

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
        age_s = float(now_s) - float(received_time_s)
        pwm = values[index]
        if (not math.isfinite(now_s) or not math.isfinite(received_time_s)
                or not 0.0 <= age_s <= self.config.maximum_age_s
                or (self._last_now_s is not None and now_s < self._last_now_s)
                or (self._last_received_s is not None and received_time_s < self._last_received_s)):
            self.reset()
            return LandingSwitchResult(LandingSwitchState.STALE, pwm, age_s)
        if not follow_active:
            self.reset()
            return LandingSwitchResult(
                LandingSwitchState.FOLLOW_INACTIVE,
                pwm,
                age_s,
                pwm <= self.config.off_below_pwm,
            )
        self._last_received_s = received_time_s
        self._last_now_s = now_s
        if pwm <= self.config.off_below_pwm:
            self._requested = False
            return LandingSwitchResult(
                LandingSwitchState.READY,
                pwm,
                age_s,
                True,
            )
        if pwm >= self.config.on_above_pwm:
            self._requested = True
            return LandingSwitchResult(LandingSwitchState.REQUESTED, pwm, age_s)
        self._requested = False
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

    def release_to_pilot(self, current_mode: str) -> None:
        """Cancel queued transitions and ignore their late callbacks without a rollback."""
        self.current_mode = self._mode(current_mode) or "UNKNOWN"
        self.desired_mode = None
        self._reset("PILOT_MODE_NOT_FOLLOW_ELIGIBLE")

    def request_is_current(self, action: ModeRequest) -> bool:
        """A cancelled/superseded request must never reach the transport."""
        return action.sequence == self._outstanding_sequence and (
            (self.phase == ModeTransitionPhase.REQUESTING_TARGET
             and not action.rollback and action.mode == self.target_mode)
            or (self.phase == ModeTransitionPhase.REQUESTING_ROLLBACK
                and action.rollback and action.mode == self.rollback_mode)
        )

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
