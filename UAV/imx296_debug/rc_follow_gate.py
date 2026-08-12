#!/usr/bin/env python3
"""Fail-closed RC authorization gate for companion-computer follow control."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RcGateStatus:
    enabled: bool
    pwm: int | None
    fresh: bool
    age_s: float | None
    reason: str


class RcFollowGate:
    """Convert one RC PWM channel into a fail-closed authorization signal."""

    def __init__(
        self,
        *,
        channel: int = 7,
        enable_pwm_min: int = 1800,
        disable_pwm_max: int = 1200,
        timeout_s: float = 0.5,
    ) -> None:
        if not 1 <= channel <= 18:
            raise ValueError("RC channel must be between 1 and 18")
        if disable_pwm_max >= enable_pwm_min:
            raise ValueError("disable threshold must be below enable threshold")
        if timeout_s <= 0:
            raise ValueError("RC timeout must be positive")
        self.channel = channel
        self.enable_pwm_min = enable_pwm_min
        self.disable_pwm_max = disable_pwm_max
        self.timeout_s = timeout_s
        self._pwm: int | None = None
        self._timestamp_s: float | None = None

    def update(self, pwm: int, timestamp_s: float) -> RcGateStatus:
        self._pwm = int(pwm)
        self._timestamp_s = float(timestamp_s)
        return self.status(timestamp_s)

    def update_from_rc_channels(self, message, timestamp_s: float) -> RcGateStatus:
        field = f"chan{self.channel}_raw"
        if not hasattr(message, field):
            self._pwm = None
            self._timestamp_s = None
            return RcGateStatus(False, None, False, None, "CHANNEL_MISSING")
        return self.update(int(getattr(message, field)), timestamp_s)

    def status(self, timestamp_s: float) -> RcGateStatus:
        if self._pwm is None or self._timestamp_s is None:
            return RcGateStatus(False, None, False, None, "NO_RC_SAMPLE")
        age_s = max(0.0, float(timestamp_s) - self._timestamp_s)
        if age_s > self.timeout_s:
            return RcGateStatus(False, self._pwm, False, age_s, "RC_SAMPLE_TIMEOUT")
        if self._pwm >= self.enable_pwm_min:
            return RcGateStatus(True, self._pwm, True, age_s, "RC_ENABLED")
        if self._pwm <= self.disable_pwm_max:
            return RcGateStatus(False, self._pwm, True, age_s, "RC_DISABLED")
        return RcGateStatus(False, self._pwm, True, age_s, "RC_PWM_AMBIGUOUS")
