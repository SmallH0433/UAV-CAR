"""Companion-owned GUIDED descent policy. No motor, mode or disarm commands.

All velocities are earth-local (positive up). The adapter must not put the
vertical command into a tilted body frame. Hardware enabling requires SITL
and prop-off acceptance; this module alone is not flight approval.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DescentInput:
    now: float
    authorized: bool = False
    armed: bool = False
    guided_confirmed: bool = False
    requested: bool = False
    landed: bool = False
    telemetry_fresh: bool = False
    range_m: float = math.nan
    range_fresh: bool = False
    tag_fresh: bool = False
    aligned: bool = False
    horizontal_speed: float = math.nan
    tilt_deg: float = math.nan


@dataclass(frozen=True)
class DescentOutput:
    phase: str
    up_mps: float = 0.0
    track_tag: bool = False
    terminal: bool = False


class GuidedDescent:
    """Fail closed on stale evidence; never infer touchdown from distance alone."""

    def __init__(self, descent_mps=.10, terminal_mps=.05,
                 near_m=.10, dwell_s=.4, terminal_timeout_s=8.0):
        values=(descent_mps, terminal_mps, near_m, dwell_s, terminal_timeout_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('descent parameters must be finite and positive')
        if terminal_mps > descent_mps or terminal_timeout_s <= dwell_s:
            raise ValueError('invalid descent parameter ordering')
        self.descent_mps, self.terminal_mps = descent_mps, terminal_mps
        self.near_m, self.dwell_s = near_m, dwell_s
        self.terminal_timeout_s = terminal_timeout_s
        self.reset()

    def reset(self):
        self.aligned_since = None
        self.near_since = None
        self.terminal_since = None
        self.completed = False
        self.fault = False
        self.last_time = None

    def update(self, x: DescentInput) -> DescentOutput:
        if not (x.authorized and x.armed and x.guided_confirmed and x.requested):
            self.reset()
            return DescentOutput('INACTIVE')
        if not math.isfinite(x.now) or (self.last_time is not None and x.now < self.last_time):
            self.fault = True
        self.last_time = x.now
        if x.telemetry_fresh and x.landed:
            self.completed = True
        if self.completed:
            return DescentOutput('LANDED_WAIT_DISARM')
        if self.terminal_since is not None and x.now-self.terminal_since >= self.terminal_timeout_s:
            self.fault = True
        if self.fault:
            return DescentOutput('FAULT_HOLD')
        safe = (x.telemetry_fresh and x.range_fresh
                and math.isfinite(x.range_m) and .02 <= x.range_m <= 8
                and math.isfinite(x.tilt_deg) and 0 <= x.tilt_deg <= 10
                and math.isfinite(x.horizontal_speed) and 0 <= x.horizontal_speed <= .10)
        if not safe:
            self.aligned_since = self.near_since = None
            if self.terminal_since is not None:
                self.fault = True
            return DescentOutput('UNSAFE_EVIDENCE_HOLD')
        if self.terminal_since is not None:
            if x.range_m > self.near_m + .05:
                self.fault = True
                return DescentOutput('HEIGHT_INCREASE_HOLD')
            return DescentOutput('TERMINAL_DESCENT', -self.terminal_mps, False, True)
        if not x.tag_fresh:
            self.aligned_since = self.near_since = None
            return DescentOutput('TAG_LOST_HOLD')
        if not x.aligned:
            self.aligned_since = self.near_since = None
            return DescentOutput('ALIGN', track_tag=True)
        if self.aligned_since is None:
            self.aligned_since = x.now
        if x.range_m <= self.near_m:
            if self.near_since is None:
                self.near_since = x.now
            if x.now-self.near_since >= self.dwell_s:
                self.terminal_since = x.now
                return DescentOutput('TERMINAL_DESCENT', -self.terminal_mps, False, True)
            return DescentOutput('VERIFY_NEAR', track_tag=True)
        self.near_since = None
        if x.now-self.aligned_since < self.dwell_s:
            return DescentOutput('VERIFY_ALIGNMENT', track_tag=True)
        return DescentOutput('TRACK_DESCENT', -self.descent_mps, True)
