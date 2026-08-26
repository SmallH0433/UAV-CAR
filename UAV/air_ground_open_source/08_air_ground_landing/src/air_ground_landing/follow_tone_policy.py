"""Transport-free audible state policy for an AprilTag follow session."""

from __future__ import annotations

from enum import Enum


class FollowToneEvent(str, Enum):
    """User-facing follow state transitions."""

    OBSERVE_READY = "OBSERVE_READY"
    FOLLOW_ACTIVE = "FOLLOW_ACTIVE"
    LANDING_ACTIVE = "LANDING_ACTIVE"
    EXIT_CONFIRMED = "EXIT_CONFIRMED"


# QBASIC 1.1 tunes accepted by MAVLink PLAY_TUNE_V2.
OBSERVE_READY_TUNE = "MFT200L8O5C"
FOLLOW_ACTIVE_TUNE = "MFT200L8O5CEG"
LANDING_ACTIVE_TUNE = "MFT200L8O5GEC"
EXIT_CONFIRMED_TUNE = "MFT200L8O5GEC"

TUNES = {
    FollowToneEvent.OBSERVE_READY: OBSERVE_READY_TUNE,
    FollowToneEvent.FOLLOW_ACTIVE: FOLLOW_ACTIVE_TUNE,
    FollowToneEvent.LANDING_ACTIVE: LANDING_ACTIVE_TUNE,
    FollowToneEvent.EXIT_CONFIRMED: EXIT_CONFIRMED_TUNE,
}


class FollowTonePolicy:
    """Emit mutually-exclusive follow, landing and exit reminders."""

    def __init__(
        self,
        follow_repeat_interval_s: float = 3.0,
        landing_repeat_interval_s: float = 2.0,
    ) -> None:
        if follow_repeat_interval_s <= 0.0 or landing_repeat_interval_s <= 0.0:
            raise ValueError("follow and landing repeat intervals must be positive")
        self.follow_repeat_interval_s = follow_repeat_interval_s
        self.landing_repeat_interval_s = landing_repeat_interval_s
        self.observation_announced = False
        self.last_follow_tone_s: float | None = None
        self.last_landing_tone_s: float | None = None
        self.active_phase: str | None = None
        self.session_started = False
        self.exit_pending = False

    def update(
        self,
        *,
        observe_ready: bool,
        follow_active: bool,
        landing_active: bool,
        exit_confirmed: bool,
        now_s: float,
    ) -> tuple[FollowToneEvent, ...]:
        events: list[FollowToneEvent] = []
        # OBSERVE_READY is driven by a fresh accepted AprilTag observation.
        # It is deliberately one-shot per acquisition cycle.
        if (
            observe_ready
            and not follow_active
            and not landing_active
            and not self.observation_announced
        ):
            self.observation_announced = True
            events.append(FollowToneEvent.OBSERVE_READY)

        # LANDING_ACTIVE has priority so follow and landing tunes never overlap.
        effective_landing = bool(landing_active and self.session_started)
        phase = "LANDING" if effective_landing else "FOLLOW" if follow_active else None
        if phase == "FOLLOW":
            due = bool(
                self.active_phase != "FOLLOW"
                or self.last_follow_tone_s is None
                or now_s - self.last_follow_tone_s >= self.follow_repeat_interval_s
            )
            self.session_started = True
            self.exit_pending = False
            self.last_landing_tone_s = None
            if due:
                self.last_follow_tone_s = now_s
                events.append(FollowToneEvent.FOLLOW_ACTIVE)
        elif phase == "LANDING":
            due = bool(
                self.active_phase != "LANDING"
                or self.last_landing_tone_s is None
                or now_s - self.last_landing_tone_s >= self.landing_repeat_interval_s
            )
            self.exit_pending = False
            self.last_follow_tone_s = None
            if due:
                self.last_landing_tone_s = now_s
                events.append(FollowToneEvent.LANDING_ACTIVE)
        elif self.session_started:
            self.exit_pending = True
            self.last_follow_tone_s = None
            self.last_landing_tone_s = None

        self.active_phase = phase

        if self.exit_pending and exit_confirmed:
            self.session_started = False
            self.exit_pending = False
            self.observation_announced = False
            events.append(FollowToneEvent.EXIT_CONFIRMED)

        if not observe_ready and not self.session_started and not self.exit_pending:
            self.observation_announced = False

        return tuple(events)
