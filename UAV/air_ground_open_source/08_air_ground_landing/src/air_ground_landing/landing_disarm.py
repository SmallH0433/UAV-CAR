"""Ordinary DISARM after 0.5 s of continuous, independently fresh evidence."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LandingEvidence:
    now: float
    active: bool = False
    armed: bool = False
    airborne: bool = False
    terminal: bool = False
    landed: bool = False
    fresh: bool = False
    heartbeat: float = -1.
    extended: float = -1.
    pose: float = -1.
    velocity: float = -1.
    range_m: float = math.nan
    horizontal_mps: float = math.nan
    vertical_mps: float = math.nan
    tilt_deg: float = math.nan


class LandingDisarm:
    """One request per authorized descent session; adapter awaits armed=false.

    Sequence tokens must advance only on new source messages. Call on every
    control tick so invalid evidence during the dwell resets confirmation.
    Heartbeat freshness is checked by the adapter; its rate does not set dwell.
    No automatic retry: a rejected/lost request requires operator attention.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.seen_airborne = False
        self.seen_terminal = False
        self.first = None
        self.sent = False
        self.last_time = None
        self.last_extended = -1.
        self.reason = "INACTIVE"

    def update(self, x: LandingEvidence) -> bool:
        if not x.active or not x.armed:
            self.reset()
            return False
        if (not math.isfinite(x.now) or
                (self.last_time is not None and x.now < self.last_time)):
            self.reset()
            self.reason = "INVALID_CLOCK"
            return False
        if self.last_time is not None and x.now-self.last_time > .5:
            self.first = None
        self.last_time = x.now
        new_extended = x.extended > self.last_extended
        self.last_extended = x.extended
        if x.fresh and x.airborne:
            self.seen_airborne = True
        if self.seen_airborne and x.terminal:
            self.seen_terminal = True
        if self.sent:
            self.reason = "REQUEST_ALREADY_SENT"
            return False
        safe = (self.seen_airborne and self.seen_terminal and x.fresh and x.landed
                and all(math.isfinite(v) for v in (x.range_m, x.horizontal_mps,
                        x.vertical_mps, x.tilt_deg))
                and .02 <= x.range_m <= .10
                and 0 <= x.horizontal_mps <= .10
                and abs(x.vertical_mps) <= .10 and 0 <= x.tilt_deg <= 10)
        if not safe:
            self.first = None
            self.reason = "WAIT_SAFE_LANDING_EVIDENCE"
            return False
        tokens = (x.extended, x.pose, x.velocity)
        if not all(math.isfinite(t) and t >= 0 for t in tokens):
            self.first = None
            self.reason = "INVALID_SAMPLE_TOKENS"
            return False
        if self.first is None:
            if not new_extended:
                self.reason = "WAIT_NEW_ON_GROUND"
                return False
            self.first = (x.now, tokens)
            self.reason = "VERIFY_LANDING_0_5_S"
            return False
        if (x.now-self.first[0] >= .5
                and all(t > previous for t, previous in zip(tokens, self.first[1]))):
            self.sent = True
            self.reason = "CONFIRMED_LANDING_0_5_S"
            return True
        return False
