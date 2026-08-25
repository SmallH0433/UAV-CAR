#!/usr/bin/env python3
"""Fail-closed CH7-to-GUIDED mode arbitration for AprilTag follow.

This module is deliberately transport-free.  It decides when a companion
computer may request a mode change, but it never opens MAVLink, arms, takes
off, lands, or writes parameters.  The caller must confirm mode changes from
flight-controller HEARTBEAT messages before sending movement setpoints.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping


class ModeManagerState(str, Enum):
    DISABLED = "DISABLED"
    WAIT_PREREQUISITES = "WAIT_PREREQUISITES"
    WAIT_GUIDED = "WAIT_GUIDED"
    ACTIVE = "ACTIVE"
    RESTORE_MODE = "RESTORE_MODE"
    PILOT_OVERRIDE_LOCKOUT = "PILOT_OVERRIDE_LOCKOUT"
    FAULT_LOCKOUT = "FAULT_LOCKOUT"


@dataclass(frozen=True)
class ModeManagerInputs:
    timestamp_s: float
    armed: bool
    current_mode: str
    rc_enable: bool
    prerequisites_ok: bool
    pilot_stick_override: bool = False
    mode_sample_id: int | None = None


@dataclass(frozen=True)
class ModeManagerDecision:
    state: ModeManagerState
    request_mode: str | None
    allow_follow_velocity: bool
    send_zero_velocity: bool
    lockout: bool
    reason: str


class FollowModeManager:
    """Turn CH7 authorization into a confirmed, reversible GUIDED session.

    A mode change made by the pilot while follow is active always wins.  The
    manager then latches out and will not request GUIDED again until CH7 has
    first returned low.  The same latch is used for a deliberate stick
    override and for failed mode-entry/safety checks.
    """

    def __init__(
        self,
        *,
        guided_confirmations: int = 3,
        mode_request_timeout_s: float = 3.0,
        mode_request_retry_s: float = 0.5,
        allowed_entry_modes: tuple[str, ...] = ("ALT_HOLD", "LOITER", "POSHOLD"),
        allow_preexisting_guided: bool = True,
    ) -> None:
        if guided_confirmations < 1:
            raise ValueError("guided_confirmations must be positive")
        if mode_request_timeout_s <= 0 or mode_request_retry_s <= 0:
            raise ValueError("mode request timing must be positive")
        self.guided_confirmations = guided_confirmations
        self.mode_request_timeout_s = mode_request_timeout_s
        self.mode_request_retry_s = mode_request_retry_s
        self.allowed_entry_modes = tuple(mode.upper() for mode in allowed_entry_modes)
        self.allow_preexisting_guided = bool(allow_preexisting_guided)
        self.state = ModeManagerState.DISABLED
        self._entry_mode: str | None = None
        self._owns_guided = False
        self._request_started_s: float | None = None
        self._last_request_s: float | None = None
        self._guided_confirmations = 0
        self._last_guided_sample_id: int | None = None
        self._was_active = False

    def update(self, inputs: ModeManagerInputs) -> ModeManagerDecision:
        now = float(inputs.timestamp_s)
        mode = inputs.current_mode.upper()

        if not inputs.rc_enable:
            return self._disabled_or_restore(now, mode)

        if self.state in (
            ModeManagerState.PILOT_OVERRIDE_LOCKOUT,
            ModeManagerState.FAULT_LOCKOUT,
        ):
            request = self._restore_request(now, mode)
            return self._decision(
                request_mode=request,
                allow_follow=False,
                send_zero=self._was_active,
                lockout=True,
                reason=(
                    "PILOT_OVERRIDE_LATCHED"
                    if self.state == ModeManagerState.PILOT_OVERRIDE_LOCKOUT
                    else "FAULT_LATCHED"
                ),
            )

        if inputs.pilot_stick_override and self.state == ModeManagerState.ACTIVE:
            self.state = ModeManagerState.PILOT_OVERRIDE_LOCKOUT
            request = self._restore_request(now, mode, force=True)
            return self._decision(
                request_mode=request,
                allow_follow=False,
                send_zero=True,
                lockout=True,
                reason="PILOT_STICK_OVERRIDE",
            )

        if not inputs.armed:
            self.state = ModeManagerState.WAIT_PREREQUISITES
            return self._decision(None, False, False, False, "DISARMED_NO_AUTO_MODE")

        if not inputs.prerequisites_ok:
            if self.state == ModeManagerState.ACTIVE:
                self.state = ModeManagerState.FAULT_LOCKOUT
                request = self._restore_request(now, mode, force=True)
                return self._decision(
                    request_mode=request,
                    allow_follow=False,
                    send_zero=True,
                    lockout=True,
                    reason="PREREQUISITE_LOST",
                )
            self.state = ModeManagerState.WAIT_PREREQUISITES
            return self._decision(None, False, False, False, "PREREQUISITES_NOT_READY")

        if self.state == ModeManagerState.ACTIVE:
            if mode != "GUIDED":
                # A flight-mode switch is an unambiguous pilot command.  Never
                # fight it by automatically requesting GUIDED again.
                self.state = ModeManagerState.PILOT_OVERRIDE_LOCKOUT
                return self._decision(
                    request_mode=None,
                    allow_follow=False,
                    send_zero=False,
                    lockout=True,
                    reason="PILOT_MODE_OVERRIDE",
                )
            self._was_active = True
            return self._decision(None, True, False, False, "GUIDED_CONFIRMED")

        if self.state in (ModeManagerState.DISABLED, ModeManagerState.WAIT_PREREQUISITES):
            if mode == "GUIDED":
                if not self.allow_preexisting_guided:
                    self.state = ModeManagerState.FAULT_LOCKOUT
                    return self._decision(
                        request_mode=None,
                        allow_follow=False,
                        send_zero=False,
                        lockout=True,
                        reason="PREEXISTING_GUIDED_NOT_OWNED",
                    )
                self._entry_mode = None
                self._owns_guided = False
            elif mode in self.allowed_entry_modes:
                self._entry_mode = mode
                self._owns_guided = True
            else:
                self.state = ModeManagerState.FAULT_LOCKOUT
                return self._decision(
                    request_mode=None,
                    allow_follow=False,
                    send_zero=False,
                    lockout=True,
                    reason="ENTRY_MODE_NOT_APPROVED",
                )
            self.state = ModeManagerState.WAIT_GUIDED
            self._request_started_s = now
            self._last_request_s = None
            self._guided_confirmations = 0

        if self.state == ModeManagerState.WAIT_GUIDED:
            if mode == "GUIDED":
                if (
                    inputs.mode_sample_id is None
                    or inputs.mode_sample_id != self._last_guided_sample_id
                ):
                    self._guided_confirmations += 1
                    self._last_guided_sample_id = inputs.mode_sample_id
                if self._guided_confirmations >= self.guided_confirmations:
                    self.state = ModeManagerState.ACTIVE
                    self._was_active = True
                    return self._decision(None, True, False, False, "GUIDED_CONFIRMED")
                return self._decision(None, False, False, False, "CONFIRMING_GUIDED")

            self._guided_confirmations = 0
            self._last_guided_sample_id = None
            if self._entry_mode is not None and mode != self._entry_mode:
                self.state = ModeManagerState.PILOT_OVERRIDE_LOCKOUT
                return self._decision(
                    request_mode=None,
                    allow_follow=False,
                    send_zero=False,
                    lockout=True,
                    reason="MODE_CHANGED_DURING_ENTRY",
                )
            if (
                self._request_started_s is not None
                and now - self._request_started_s > self.mode_request_timeout_s
            ):
                self.state = ModeManagerState.FAULT_LOCKOUT
                return self._decision(
                    request_mode=None,
                    allow_follow=False,
                    send_zero=False,
                    lockout=True,
                    reason="GUIDED_ENTRY_TIMEOUT",
                )
            request = None
            if self._request_due(now):
                request = "GUIDED"
                self._last_request_s = now
            return self._decision(request, False, False, False, "REQUESTING_GUIDED")

        raise RuntimeError(f"unhandled mode-manager state: {self.state}")

    def _disabled_or_restore(self, now: float, mode: str) -> ModeManagerDecision:
        send_zero = self._was_active and mode == "GUIDED"
        if self._owns_guided and self._entry_mode and mode == "GUIDED":
            self.state = ModeManagerState.RESTORE_MODE
            request = self._restore_request(now, mode)
            return self._decision(
                request_mode=request,
                allow_follow=False,
                send_zero=send_zero,
                lockout=False,
                reason="RC_DISABLED_RESTORE_MODE",
            )
        self._reset_session()
        return self._decision(None, False, send_zero, False, "RC_DISABLED")

    def _restore_request(
        self, now: float, mode: str, *, force: bool = False
    ) -> str | None:
        if not self._owns_guided or not self._entry_mode or mode != "GUIDED":
            return None
        if force or self._request_due(now):
            self._last_request_s = now
            return self._entry_mode
        return None

    def _request_due(self, now: float) -> bool:
        return (
            self._last_request_s is None
            or now - self._last_request_s >= self.mode_request_retry_s
        )

    def _reset_session(self) -> None:
        self.state = ModeManagerState.DISABLED
        self._entry_mode = None
        self._owns_guided = False
        self._request_started_s = None
        self._last_request_s = None
        self._guided_confirmations = 0
        self._last_guided_sample_id = None
        self._was_active = False

    def _decision(
        self,
        request_mode: str | None,
        allow_follow: bool,
        send_zero: bool,
        lockout: bool,
        reason: str,
    ) -> ModeManagerDecision:
        return ModeManagerDecision(
            state=self.state,
            request_mode=request_mode,
            allow_follow_velocity=allow_follow,
            send_zero_velocity=send_zero,
            lockout=lockout,
            reason=reason,
        )


class PilotStickOverrideDetector:
    """Debounced pilot override detector for centred roll/pitch/yaw sticks.

    Throttle is intentionally excluded because its neutral value depends on
    the takeoff mode and transmitter setup.  The detector is an additional
    escape path; the flight-mode switch remains the unconditional override.
    """

    def __init__(
        self,
        *,
        threshold_pwm: int = 150,
        debounce_s: float = 0.20,
        centres_pwm: Mapping[int, int] | None = None,
    ) -> None:
        if threshold_pwm <= 0 or debounce_s <= 0:
            raise ValueError("override threshold and debounce must be positive")
        self.threshold_pwm = int(threshold_pwm)
        self.debounce_s = float(debounce_s)
        self.centres_pwm = dict(centres_pwm or {1: 1500, 2: 1500, 4: 1500})
        self._deflected_since_s: float | None = None

    def update(self, channels_pwm: Mapping[int, int], timestamp_s: float) -> bool:
        deflected = any(
            channel in channels_pwm
            and abs(int(channels_pwm[channel]) - centre) >= self.threshold_pwm
            for channel, centre in self.centres_pwm.items()
        )
        if not deflected:
            self._deflected_since_s = None
            return False
        now = float(timestamp_s)
        if self._deflected_since_s is None:
            self._deflected_since_s = now
            return False
        return now - self._deflected_since_s + 1e-9 >= self.debounce_s
