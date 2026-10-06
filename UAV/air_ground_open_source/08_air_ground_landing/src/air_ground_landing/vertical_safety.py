"""Pure vertical evidence monitor and SHADOW height reference generator.

Z and Vz are ENU (positive up); range/PnP are positive vertical ground
distance. Every timestamp is a measurement timestamp in the same clock domain
as ``now_s``. An adapter must not refresh a cached measurement's timestamp.
Thresholds originate from the 2026-10-06 exploratory replay, not flight
certification. No ROS, vehicle I/O, mode changes, or fault-session latch lives
here. The action owner MUST latch FAULT separately, across monitor resets.
"""

from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
import math
from statistics import median
from typing import Any, Deque, Dict, Optional, Tuple


class VerticalHealthState(str, Enum):
    HEALTHY = "HEALTHY"
    SUSPECT = "SUSPECT"
    FAULT = "FAULT"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class VerticalSafetyConfig:
    fast_window_s: float = 0.75
    fast_residual_m: float = 0.25
    fast_persistence_s: float = 0.2
    slow_window_s: float = 2.5
    slow_rise_m: float = 0.08
    slow_vz_max_mps: float = -0.03
    slow_persistence_s: float = 0.4
    source_max_age_s: float = 0.2
    status_max_age_s: float = 0.5
    vertical_status_max_age_s: float = 0.5
    pnp_max_age_s: float = 0.35
    max_sample_gap_s: float = 0.2
    future_tolerance_s: float = 0.02
    window_tolerance_s: float = 0.065
    max_tilt_rad: float = math.radians(10.0)
    minimum_range_m: float = 0.08
    maximum_range_m: float = 10.0
    minimum_range_quality: float = 0.0
    fast_endpoint_s: float = 0.10
    slow_endpoint_s: float = 0.25
    pnp_endpoint_s: float = 0.5
    minimum_endpoint_samples: int = 3
    minimum_pnp_endpoint_samples: int = 2

    def __post_init__(self):
        positive = ("fast_window_s", "fast_residual_m", "fast_persistence_s",
                    "slow_window_s", "slow_rise_m", "slow_persistence_s",
                    "source_max_age_s", "status_max_age_s", "vertical_status_max_age_s", "pnp_max_age_s",
                    "max_sample_gap_s", "max_tilt_rad", "maximum_range_m",
                    "fast_endpoint_s", "slow_endpoint_s", "pnp_endpoint_s")
        for name in positive:
            if not _finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError("%s must be finite and positive" % name)
        for name in ("future_tolerance_s", "window_tolerance_s", "minimum_range_m"):
            if not _finite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError("%s must be finite and nonnegative" % name)
        if not _finite(self.slow_vz_max_mps):
            raise ValueError("slow_vz_max_mps must be finite")
        if not 0 <= self.minimum_range_quality < 1:
            raise ValueError("minimum_range_quality must be in [0, 1)")
        if self.maximum_range_m <= self.minimum_range_m:
            raise ValueError("invalid range envelope")
        if self.window_tolerance_s >= self.fast_window_s:
            raise ValueError("window tolerance must be smaller than fast window")
        if self.minimum_endpoint_samples < 3 or self.minimum_pnp_endpoint_samples < 2:
            raise ValueError("endpoint evidence must resist single packets")


@dataclass(frozen=True)
class VerticalSample:
    now_s: float
    z_m: Optional[float] = None
    vz_mps: Optional[float] = None
    pose_time_s: Optional[float] = None
    velocity_time_s: Optional[float] = None
    range_m: Optional[float] = None
    range_time_s: Optional[float] = None
    range_min_m: float = 0.08
    range_max_m: float = 10.0
    range_healthy: Optional[bool] = None
    range_status_time_s: Optional[float] = None
    # None means that adapter health is the only available quality evidence.
    range_quality: Optional[float] = None
    range_source_id: str = "downward_range"
    pose_source_id: str = "local_enu"
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    attitude_time_s: Optional[float] = None
    pnp_height_m: Optional[float] = None
    pnp_time_s: Optional[float] = None
    pnp_source_id: Optional[str] = None
    pnp_accepted: bool = False
    vertical_position_valid: Optional[bool] = None
    vertical_velocity_valid: Optional[bool] = None
    vertical_status_time_s: Optional[float] = None
    ekf_reset_counter: Optional[int] = None
    active: bool = True
    context: str = ""
    mode: str = ""
    commanded_vz_mps: Optional[float] = None


@dataclass(frozen=True)
class VerticalHealthSnapshot:
    state: VerticalHealthState
    reason: str
    time_s: Optional[float]
    trusted_height_m: Optional[float] = None
    fast_state: VerticalHealthState = VerticalHealthState.UNKNOWN
    slow_state: VerticalHealthState = VerticalHealthState.UNKNOWN
    source_times: Dict[str, Optional[float]] = field(default_factory=dict)
    features: Dict[str, Any] = field(default_factory=dict)
    evidence_availability: Dict[str, bool] = field(default_factory=dict)
    confidence: str = "INSUFFICIENT"
    input_source_times: Dict[str, Optional[float]] = field(default_factory=dict)
    input_ages_s: Dict[str, Optional[float]] = field(default_factory=dict)
    buffered_sample_counts: Dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return _json_safe(asdict(self))


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _json_safe(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class VerticalSafetyMonitor:
    """Independent fast/slow evidence with measurement-time persistence.

    Repeated packets never add samples or persistence. Discontinuities rebuild
    evidence, while the caller's fault latch remains entirely independent.
    Three distinct endpoint samples protect against a single range spike;
    compared with the exploratory replay this can add detection latency.
    """

    def __init__(self, config: Optional[VerticalSafetyConfig] = None):
        self.config = config or VerticalSafetyConfig()
        self._series: Dict[str, Deque[Tuple[float, float]]] = {
            name: deque() for name in ("pose", "velocity", "range", "pnp")}
        self._highwater: Dict[str, float] = {}
        self._last_now = None
        self._context = None
        self._sources = None
        self._pnp_source = None
        self._reset_counter = None
        self._clear_windows()

    def _clear_windows(self):
        for series in self._series.values():
            series.clear()
        self._since = {"fast": None, "slow": None}
        self._last_eval = None
        self._fast = VerticalHealthState.UNKNOWN
        self._slow = VerticalHealthState.UNKNOWN
        self._features = {}
        self._candidate_not_before_s = None

    def _age_ok(self, now, stamp, maximum):
        return _finite(stamp) and -self.config.future_tolerance_s <= now - stamp <= maximum + 1e-9

    def _unknown(self, sample, reason):
        self._clear_windows()
        return self._snapshot(sample, VerticalHealthState.UNKNOWN, reason)

    def _stale(self, sample, reason):
        """Keep bounded measured history, never health or a pending fault dwell.

        A delayed callback is not proof that the original sensor timeline has
        a gap. Recovery still validates every original timestamp and window
        gap; the stale tick itself cannot use the retained observations.
        """
        cutoff = sample.now_s - self.config.slow_window_s - self.config.max_sample_gap_s
        for rows in self._series.values():
            while rows and rows[0][0] < cutoff:
                rows.popleft()
        self._since = {"fast": None, "slow": None}
        self._last_eval = None
        self._fast = self._slow = VerticalHealthState.UNKNOWN
        self._features = {}
        # Measurement timestamps can lag the callback. Neither old evidence
        # nor a resumed packet may count the unavailable interval as dwell.
        self._candidate_not_before_s = sample.now_s
        return self._snapshot(sample, VerticalHealthState.UNKNOWN, reason)

    def _expired(self, now, stamp, maximum):
        return _finite(stamp) and now - stamp > maximum + 1e-9

    def _snapshot(self, sample, state, reason, height=None, pnp_fresh=False, flags=False):
        times = {name: (series[-1][0] if series else None)
                 for name, series in self._series.items()}
        times.update(attitude=sample.attitude_time_s,
                     range_status=sample.range_status_time_s,
                     vertical_status=sample.vertical_status_time_s)
        input_times = {name: getattr(sample, name + "_time_s")
                       for name in ("pose", "velocity", "range", "pnp", "attitude",
                                    "range_status", "vertical_status")}
        input_ages = {name: sample.now_s - stamp
                      if _finite(sample.now_s) and _finite(stamp) else None
                      for name, stamp in input_times.items()}
        available = {"fast": self._fast != VerticalHealthState.UNKNOWN,
                     "slow": self._slow != VerticalHealthState.UNKNOWN,
                     "pnp_fresh": pnp_fresh, "vertical_flags": flags,
                     "range": height is not None}
        features = dict(self._features)
        features.update(pnp_fresh=pnp_fresh, slow_available=available["slow"],
                        commanded_vz_mps=sample.commanded_vz_mps, mode=sample.mode,
                        fast_candidate_since_s=self._since["fast"],
                        slow_candidate_since_s=self._since["slow"])
        return VerticalHealthSnapshot(
            state=state, reason=reason, time_s=sample.now_s,
            trusted_height_m=height, fast_state=self._fast, slow_state=self._slow,
            source_times=times, features=features, evidence_availability=available,
            confidence=("CORROBORATED" if available["slow"] else
                        "FAST_ONLY" if available["fast"] else "INSUFFICIENT"),
            input_source_times=input_times, input_ages_s=input_ages,
            buffered_sample_counts={name: len(rows) for name, rows in self._series.items()})

    def _append(self, name, stamp, value):
        if stamp > self._highwater.get(name, -math.inf):
            self._highwater[name] = stamp
            self._series[name].append((stamp, value))
        cutoff = stamp - self.config.slow_window_s - self.config.max_sample_gap_s
        while self._series[name] and self._series[name][0][0] < cutoff:
            self._series[name].popleft()

    def _window(self, name, end, width):
        # Independent sources are not phase locked. Anchor each source window
        # to its newest sample at/before the common measurement watermark;
        # do not discard an extra sample period at both edges. No value newer
        # than that watermark participates. Source freshness bounds alignment.
        rows = [row for row in self._series[name] if row[0] <= end + 1e-9]
        if not rows:
            return None
        source_end = rows[-1][0]
        if end - source_end > self.config.max_sample_gap_s + 1e-9:
            return None
        cutoff = source_end - width
        # Retain one real sample supporting the left boundary. Without it,
        # jittered 10 Hz sources lose up to a sample period at the boundary,
        # repeatedly invalidating an otherwise continuous 0.75 s window.
        # The observed span may exceed width by at most max_sample_gap_s;
        # endpoint medians still use distinct real packets, not interpolation.
        before = [row for row in rows if row[0] < cutoff - 1e-9]
        rows = [row for row in rows if cutoff - 1e-9 <= row[0]]
        if before and (not rows or rows[0][0] > cutoff + 1e-9):
            rows.insert(0, before[-1])
        if len(rows) < 2 * self.config.minimum_endpoint_samples:
            return None
        if rows[-1][0] - rows[0][0] < width - self.config.window_tolerance_s - 1e-9:
            return None
        if any(b[0] - a[0] > self.config.max_sample_gap_s + 1e-9
               for a, b in zip(rows, rows[1:])):
            return None
        return rows

    def _delta(self, rows, band):
        count = self.config.minimum_endpoint_samples
        begin = [value for stamp, value in rows if stamp <= rows[0][0] + band + 1e-9]
        end = [value for stamp, value in rows if stamp >= rows[-1][0] - band - 1e-9]
        if len(begin) < count:
            begin = [value for _, value in rows[:count]]
        if len(end) < count:
            end = [value for _, value in rows[-count:]]
        return median(end) - median(begin)

    def _signal(self, name, hit, end, duration, candidate_start=None):
        if not hit:
            self._since[name] = None
            return VerticalHealthState.HEALTHY
        if self._since[name] is None:
            self._since[name] = end if candidate_start is None else candidate_start
            if self._candidate_not_before_s is not None:
                self._since[name] = max(self._since[name], self._candidate_not_before_s)
        return (VerticalHealthState.FAULT if end - self._since[name] >= duration - 1e-9
                else VerticalHealthState.SUSPECT)

    def update(self, sample: VerticalSample) -> VerticalHealthSnapshot:
        c = self.config
        now = sample.now_s
        if not _finite(now):
            return self._unknown(sample, "INVALID_CLOCK")
        if self._last_now is not None and now < self._last_now:
            self._last_now = now
            self._highwater.clear()
            return self._unknown(sample, "CLOCK_ROLLBACK")
        self._last_now = now
        if not sample.active:
            return self._unknown(sample, "INACTIVE")
        # Mode is telemetry: an acknowledged LOITER -> GUIDED transition must
        # not erase valid evidence and immediately trip an UNKNOWN interlock.
        # The caller can explicitly change context for a physical discontinuity.
        context = sample.context
        if context != self._context:
            self._clear_windows()
            self._context = context
        sources = (sample.range_source_id, sample.pose_source_id)
        if not all(isinstance(source, str) and source for source in sources):
            return self._unknown(sample, "SOURCE_ID_UNKNOWN")
        if self._sources is not None and sources != self._sources:
            self._sources = sources
            self._highwater.clear()
            return self._unknown(sample, "SOURCE_CHANGED")
        self._sources = sources
        if sample.ekf_reset_counter is not None:
            changed = self._reset_counter is not None and sample.ekf_reset_counter != self._reset_counter
            self._reset_counter = sample.ekf_reset_counter
            if changed:
                self._highwater.clear()
                return self._unknown(sample, "EKF_RESET")

        inputs = {"pose": (sample.pose_time_s, sample.z_m),
                  "velocity": (sample.velocity_time_s, sample.vz_mps),
                  "range": (sample.range_time_s, sample.range_m)}
        stale_reason = None
        for name, (stamp, value) in inputs.items():
            if _finite(stamp) and stamp < self._highwater.get(name, -math.inf):
                return self._unknown(sample, name.upper() + "_OUT_OF_ORDER")
            if not _finite(value):
                return self._unknown(sample, name.upper() + "_INVALID_OR_STALE")
            if not self._age_ok(now, stamp, c.source_max_age_s):
                if not self._expired(now, stamp, c.source_max_age_s):
                    return self._unknown(sample, name.upper() + "_INVALID_OR_STALE")
                stale_reason = stale_reason or name.upper() + "_INVALID_OR_STALE"
        if sample.range_healthy is not True:
            return self._unknown(sample, "RANGE_QUALITY_UNKNOWN_OR_INVALID")
        if not self._age_ok(now, sample.range_status_time_s, c.status_max_age_s):
            if not self._expired(now, sample.range_status_time_s, c.status_max_age_s):
                return self._unknown(sample, "RANGE_QUALITY_UNKNOWN_OR_INVALID")
            stale_reason = stale_reason or "RANGE_QUALITY_UNKNOWN_OR_INVALID"
        if sample.range_quality is not None and (
                not _finite(sample.range_quality) or
                not c.minimum_range_quality < sample.range_quality <= 1.0):
            return self._unknown(sample, "RANGE_QUALITY_INVALID")
        if (not _finite(sample.range_min_m) or not _finite(sample.range_max_m) or
                sample.range_min_m >= sample.range_max_m or
                not max(c.minimum_range_m, sample.range_min_m) <= sample.range_m <=
                min(c.maximum_range_m, sample.range_max_m)):
            return self._unknown(sample, "RANGE_ENVELOPE_INVALID")
        if (not _finite(sample.roll_rad) or not _finite(sample.pitch_rad) or
                abs(sample.roll_rad) > c.max_tilt_rad or abs(sample.pitch_rad) > c.max_tilt_rad):
            return self._unknown(sample, "ATTITUDE_INVALID_OR_STALE")
        if not self._age_ok(now, sample.attitude_time_s, c.source_max_age_s):
            if not self._expired(now, sample.attitude_time_s, c.source_max_age_s):
                return self._unknown(sample, "ATTITUDE_INVALID_OR_STALE")
            stale_reason = stale_reason or "ATTITUDE_INVALID_OR_STALE"
        # A stale pose must not hide a bad range, invalid velocity, backward
        # source, or invalid attitude elsewhere in the same snapshot. Only an
        # otherwise valid snapshot may preserve history for transport delay.
        if stale_reason is not None:
            return self._stale(sample, stale_reason)
        # Range's projection uses attitude measured close to that measurement.
        if abs(sample.attitude_time_s - sample.range_time_s) > c.max_sample_gap_s:
            return self._unknown(sample, "RANGE_ATTITUDE_UNALIGNED")
        for name, (stamp, _) in inputs.items():
            rows = self._series[name]
            if rows and stamp - rows[-1][0] > c.max_sample_gap_s + 1e-9:
                self._clear_windows()
                break
        for name, (stamp, value) in inputs.items():
            if name == "range":
                value *= math.cos(sample.roll_rad) * math.cos(sample.pitch_rad)
            self._append(name, stamp, value)

        pnp_fresh = (sample.pnp_accepted and _finite(sample.pnp_height_m) and
                     sample.pnp_height_m > 0 and bool(sample.pnp_source_id) and
                     self._age_ok(now, sample.pnp_time_s, c.pnp_max_age_s))
        if pnp_fresh:
            if self._pnp_source != sample.pnp_source_id:
                self._series["pnp"].clear()
                self._since["slow"] = None
                self._slow = VerticalHealthState.UNKNOWN
                self._pnp_source = sample.pnp_source_id
                self._highwater.pop("pnp", None)
            if sample.pnp_time_s < self._highwater.get("pnp", -math.inf):
                pnp_fresh = False
            else:
                self._append("pnp", sample.pnp_time_s, sample.pnp_height_m)
        # accepted_this_poll=False is normal between independent camera frames.
        # Keep the last actually accepted observation only until its ORIGINAL
        # measurement age expires. A rejected/duplicate poll never refreshes it.
        pnp_rows = self._series["pnp"]
        pnp_fresh = bool(pnp_rows and self._age_ok(now, pnp_rows[-1][0], c.pnp_max_age_s))
        if not pnp_fresh:
            self._since["slow"] = None
            self._slow = VerticalHealthState.UNKNOWN

        end = min(stamp for stamp, _ in inputs.values())
        if self._last_eval is None or end > self._last_eval:
            if self._last_eval is not None and end - self._last_eval > c.max_sample_gap_s + 1e-9:
                self._since = {"fast": None, "slow": None}
            self._last_eval = end
            self._features = {"observation_time_s": end}
            fast = {name: self._window(name, end, c.fast_window_s) for name in inputs}
            self._fast = VerticalHealthState.UNKNOWN
            if all(fast.values()):
                dr = self._delta(fast["range"], c.fast_endpoint_s)
                dz = self._delta(fast["pose"], c.fast_endpoint_s)
                residual = dr - dz
                self._features.update(delta_range_fast_m=dr, delta_z_fast_m=dz,
                                      fast_residual_m=residual,
                                      fast_window_s=c.fast_window_s,
                                      fast_source_span_s={name: rows[-1][0]-rows[0][0]
                                                          for name, rows in fast.items()})
                self._fast = self._signal("fast", residual >= c.fast_residual_m, end, c.fast_persistence_s)
            else:
                self._since["fast"] = None
            slow = {name: self._window(name, end, c.slow_window_s) for name in inputs}
            self._slow = VerticalHealthState.UNKNOWN
            if all(slow.values()) and pnp_fresh:
                start = end - c.slow_window_s
                pnps = [row for row in self._series["pnp"] if start <= row[0] <= end]
                begin_pnp = [value for stamp, value in pnps if stamp <= start + c.pnp_endpoint_s]
                end_pnp = [value for stamp, value in pnps if stamp >= end - c.pnp_endpoint_s]
                self._features.update(pnp_start_packet_count=len(begin_pnp),
                                      pnp_end_packet_count=len(end_pnp))
                if min(len(begin_pnp), len(end_pnp)) >= c.minimum_pnp_endpoint_samples:
                    dr = self._delta(slow["range"], c.slow_endpoint_s)
                    dp = median(end_pnp) - median(begin_pnp)
                    # Time weighted trapezoidal mean avoids packet-rate bias.
                    velocity = slow["velocity"]
                    mean_vz = sum((b[0] - a[0]) * (a[1] + b[1]) / 2
                                  for a, b in zip(velocity, velocity[1:])) / (velocity[-1][0] - velocity[0][0])
                    self._features.update(delta_range_slow_m=dr, delta_pnp_slow_m=dp,
                                          mean_fc_vz_mps=mean_vz, slow_window_s=c.slow_window_s,
                                          slow_source_span_s={name: rows[-1][0]-rows[0][0]
                                                              for name, rows in slow.items()})
                    # Re-polling the same accepted image cannot advance slow
                    # persistence, even while its age is still permissible.
                    slow_watermark = min(end, pnps[-1][0])
                    self._features["slow_observation_time_s"] = slow_watermark
                    self._slow = self._signal("slow", dr >= c.slow_rise_m and dp >= c.slow_rise_m
                                              and mean_vz <= c.slow_vz_max_mps, slow_watermark,
                                              c.slow_persistence_s, candidate_start=end)
            if self._slow == VerticalHealthState.UNKNOWN:
                self._since["slow"] = None

        flags_fresh = self._age_ok(now, sample.vertical_status_time_s, c.vertical_status_max_age_s)
        flags_ok = flags_fresh and sample.vertical_position_valid is True and sample.vertical_velocity_valid is True
        ranges = self._series["range"]
        height = median([value for _, value in list(ranges)[-3:]]) if len(ranges) >= 3 else None
        if VerticalHealthState.FAULT in (self._fast, self._slow):
            reason = "FAST_HEIGHT_CONTRADICTION" if self._fast == VerticalHealthState.FAULT else "SLOW_CORROBORATED_RISE"
            state = VerticalHealthState.FAULT
        elif VerticalHealthState.SUSPECT in (self._fast, self._slow):
            state, reason = VerticalHealthState.SUSPECT, "CONTRADICTION_PENDING"
        elif height is not None and abs(ranges[-1][1] - height) >= c.fast_residual_m:
            state, reason = VerticalHealthState.SUSPECT, "RANGE_TRANSIENT_PENDING"
        elif flags_fresh and (sample.vertical_position_valid is False or sample.vertical_velocity_valid is False):
            state, reason = VerticalHealthState.SUSPECT, "VERTICAL_FLAGS_INVALID"
        elif not flags_ok:
            state, reason = VerticalHealthState.UNKNOWN, "VERTICAL_FLAGS_UNKNOWN_OR_STALE"
        elif self._fast == VerticalHealthState.UNKNOWN:
            state, reason = VerticalHealthState.UNKNOWN, "FAST_WINDOW_WARMUP"
        else:
            state = VerticalHealthState.HEALTHY
            reason = "CONSISTENT" if self._slow == VerticalHealthState.HEALTHY else "FAST_CONSISTENT_SLOW_UNAVAILABLE"
        return self._snapshot(sample, state, reason, height, pnp_fresh, flags_ok)


@dataclass(frozen=True)
class HeightReferenceConfig:
    shadow_only: bool = True
    capture_dwell_s: float = 0.5
    capture_band_m: float = 0.04
    source_max_age_s: float = 0.2
    deadband_m: float = 0.015
    reference_tracking_band_m: float = 0.12
    maximum_error_m: float = 0.25
    proportional_gain: float = 0.5
    maximum_climb_mps: float = 0.05
    maximum_descent_mps: float = 0.10
    maximum_acceleration_mps2: float = 0.10
    descent_phases: Tuple[str, ...] = ("GUIDED_TRACK_DESCENT",)

    def __post_init__(self):
        if not self.shadow_only:
            raise ValueError("height control is shadow-only pending independent validation")
        for name, value in asdict(self).items():
            if name not in ("shadow_only", "descent_phases") and (not _finite(value) or value <= 0):
                raise ValueError("%s must be finite and positive" % name)
        if not self.deadband_m < self.reference_tracking_band_m < self.maximum_error_m:
            raise ValueError("height error bands must be strictly ordered")


@dataclass(frozen=True)
class HeightReferenceInput:
    now_s: float
    health: str = "UNKNOWN"
    height_m: Optional[float] = None
    height_time_s: Optional[float] = None
    action: str = ""
    phase: str = ""
    tag_fresh: bool = False
    nominal_descent_mps: float = 0.05
    session_id: str = ""


@dataclass(frozen=True)
class HeightReferenceSnapshot:
    h_ref_m: Optional[float]
    shadow_vz_mps: Optional[float]
    reason: str
    active: bool
    shadow_only: bool = True
    command_vz_mps: Optional[float] = None

    def to_dict(self):
        return _json_safe(asdict(self))


class HeightReferenceController:
    """Bounded proportional reference generator, permanently shadow-only.

    Its reference can only decrease inside one authorization session. Missing
    tags freeze that reference; missing height/health suppress all corrections.
    There is no integral state and no actuator output.
    """

    def __init__(self, config: Optional[HeightReferenceConfig] = None):
        self.config = config or HeightReferenceConfig()
        self.reset()

    def reset(self):
        """Call only at an externally authorized new session, never on tag loss."""
        self.h_ref_m = None
        self._capture = deque()
        self._last_time = None
        self._height_time = None
        self._integration_time = None
        self._last_vz = 0.0
        self._session = None

    def _suppressed(self, reason, integration_time=None):
        self._last_vz = 0.0
        self._capture.clear()
        # Invalid intervals must never be accumulated into later reference
        # motion. A fresh sample after a detected gap may establish a baseline.
        self._integration_time = integration_time
        return HeightReferenceSnapshot(self.h_ref_m, None, reason, False)

    def update(self, sample: HeightReferenceInput) -> HeightReferenceSnapshot:
        c = self.config
        if self._session != sample.session_id:
            self.reset()
            self._session = sample.session_id
        now = sample.now_s
        if not _finite(now):
            return self._suppressed("INVALID_CLOCK")
        if self._last_time is not None and now < self._last_time:
            self._last_time = now
            self._height_time = None
            return self._suppressed("CLOCK_ROLLBACK")
        tick_dt = 0.0 if self._last_time is None else now - self._last_time
        self._last_time = now
        if sample.health != VerticalHealthState.HEALTHY:
            return self._suppressed("VERTICAL_HEALTH_NOT_HEALTHY")
        if sample.action not in ("FOLLOW", "LAND"):
            return self._suppressed("ACTION_INACTIVE")
        if (not _finite(sample.height_m) or sample.height_m <= 0 or
                not _finite(sample.height_time_s) or not 0 <= now - sample.height_time_s <= c.source_max_age_s + 1e-9):
            return self._suppressed("HEIGHT_INVALID_OR_STALE")
        if self._height_time is not None and sample.height_time_s < self._height_time:
            return self._suppressed("HEIGHT_OUT_OF_ORDER")
        fresh_packet = self._height_time is None or sample.height_time_s > self._height_time
        source_gap = (0.0 if self._height_time is None else sample.height_time_s - self._height_time)
        if tick_dt > c.source_max_age_s + 1e-9 or source_gap > c.source_max_age_s + 1e-9:
            self._height_time = sample.height_time_s
            return self._suppressed("OBSERVATION_GAP", integration_time=sample.height_time_s)
        self._height_time = sample.height_time_s
        # Integrate only on an independent measurement, by elapsed measurement
        # time. Using tick_dt here incorrectly scales a 10 Hz sensor's nominal
        # descent and acceleration by 10 / tick_hz. Repeated polls consume no
        # motion budget; an invalid interval resets this baseline above.
        dt = (sample.height_time_s - self._integration_time
              if fresh_packet and self._integration_time is not None else 0.0)
        if fresh_packet:
            self._integration_time = sample.height_time_s
        if self.h_ref_m is None:
            if not sample.tag_fresh:
                return self._suppressed("WAITING_FOR_TAG_TO_CAPTURE")
            if fresh_packet:
                self._capture.append((sample.height_time_s, sample.height_m))
                while self._capture and (sample.height_time_s - self._capture[0][0] > c.capture_dwell_s + c.source_max_age_s):
                    self._capture.popleft()
                values = [value for _, value in self._capture]
                if max(values) - min(values) > c.capture_band_m:
                    self._capture.clear()
                    self._capture.append((sample.height_time_s, sample.height_m))
                if len(self._capture) >= 3 and sample.height_time_s - self._capture[0][0] >= c.capture_dwell_s - 1e-9:
                    self.h_ref_m = median([value for _, value in self._capture])
            if self.h_ref_m is None:
                return HeightReferenceSnapshot(None, None, "CAPTURE_STABILIZING", False)
        error = self.h_ref_m - sample.height_m
        if abs(error) > c.maximum_error_m:
            return self._suppressed("HEIGHT_ERROR_OUTSIDE_ENVELOPE")
        descending = (sample.action == "LAND" and sample.phase in c.descent_phases and sample.tag_fresh)
        if not _finite(sample.nominal_descent_mps) or sample.nominal_descent_mps < 0:
            return self._suppressed("INVALID_DESCENT_RATE")
        feedforward = 0.0
        # A duplicate cached measurement may describe the last state but never
        # advances the reference or the acceleration ramp.
        if fresh_packet and descending and abs(error) <= c.reference_tracking_band_m:
            rate = min(sample.nominal_descent_mps, c.maximum_descent_mps)
            self.h_ref_m = max(0.0, self.h_ref_m - rate * dt)
            feedforward = -rate
        error = self.h_ref_m - sample.height_m
        correction = 0.0 if abs(error) <= c.deadband_m else c.proportional_gain * error
        target = max(-c.maximum_descent_mps, min(c.maximum_climb_mps, feedforward + correction))
        step = c.maximum_acceleration_mps2 * dt if fresh_packet else 0.0
        vz = max(self._last_vz - step, min(self._last_vz + step, target))
        self._last_vz = vz
        reason = "SHADOW_DESCENT" if descending else "SHADOW_HOLD" if sample.tag_fresh else "SHADOW_TAG_LOSS_REFERENCE_FROZEN"
        return HeightReferenceSnapshot(self.h_ref_m, vz, reason, True)
