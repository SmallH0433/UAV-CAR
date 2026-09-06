"""Transport-free audible state policy for an AprilTag follow session."""

from __future__ import annotations

from dataclasses import dataclass
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


@dataclass(frozen=True)
class FollowToneInputs:
    """Flight-state-gated inputs consumed by the tone arbiter."""

    observe_ready: bool
    follow_active: bool
    landing_active: bool
    exit_confirmed: bool


def gate_follow_tones(
    *,
    connected: bool,
    armed: bool,
    current_mode: str,
    guided_mode: str,
    land_mode: str,
    tag_detected: bool,
    control_active: bool,
    target_echo_fresh: bool,
    landing_requested_active: bool,
    suppress_tag_after_land: bool,
) -> FollowToneInputs:
    """Apply flight-mode and arming gates before audible arbitration."""

    mode = (current_mode or "UNKNOWN").upper()
    guided = guided_mode.upper()
    land = land_mode.upper()
    active_vehicle = bool(connected and armed)
    return FollowToneInputs(
        observe_ready=bool(
            active_vehicle
            and tag_detected
            and mode not in {guided, land}
            and not suppress_tag_after_land
        ),
        follow_active=bool(
            active_vehicle
            and control_active
            and target_echo_fresh
            and mode == guided
        ),
        landing_active=bool(
            active_vehicle and landing_requested_active and mode == land
        ),
        exit_confirmed=bool(not armed or mode not in {guided, land}),
    )


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
        # One arbiter owns all audible output. LAND has priority over FOLLOW,
        # and both have priority over the one-shot Tag-ready tone.
        phase = "LANDING" if landing_active else "FOLLOW" if follow_active else None
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
                self.active_phase = phase
                return (FollowToneEvent.FOLLOW_ACTIVE,)
        elif phase == "LANDING":
            due = bool(
                self.active_phase != "LANDING"
                or self.last_landing_tone_s is None
                or now_s - self.last_landing_tone_s >= self.landing_repeat_interval_s
            )
            self.session_started = True
            self.exit_pending = False
            self.last_follow_tone_s = None
            if due:
                self.last_landing_tone_s = now_s
                self.active_phase = phase
                return (FollowToneEvent.LANDING_ACTIVE,)
        elif self.session_started:
            self.exit_pending = True
            self.last_follow_tone_s = None
            self.last_landing_tone_s = None

        self.active_phase = phase

        if self.exit_pending and exit_confirmed:
            self.session_started = False
            self.exit_pending = False
            return (FollowToneEvent.EXIT_CONFIRMED,)

        if observe_ready and not self.observation_announced:
            self.observation_announced = True
            return (FollowToneEvent.OBSERVE_READY,)

        if not observe_ready and not self.session_started and not self.exit_pending:
            self.observation_announced = False

        return ()
