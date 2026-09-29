"""Same-observation landing correction, with explicit FRD -> FLU yaw sign."""
from dataclasses import dataclass
import math
from typing import Mapping, Optional

from .math3d import rotate_by_quaternion


def alignment_payload(features, observation, now_s):
    """Use the bridge-validated common-pad quaternion, never raw inner-tag yaw."""
    q = observation.orientation_body_frd_wxyz
    if not features.valid or q is None:
        return None
    forward = rotate_by_quaternion((1.0, 0.0, 0.0), q)
    if not all(math.isfinite(v) for v in forward) or math.hypot(*forward[:2]) < 1e-6:
        return None
    velocity = features.correction_body_frd_mps
    return {
        "frame": "BODY_FLU",
        "velocity_flu": [velocity[0], -velocity[1]],
        # BODY_FRD right is positive; ROS yaw/FLU left is positive.
        "heading_error_rad": -math.atan2(forward[1], forward[0]),
        "center_error_px": features.centroid_error_px,
        "source_age_s": now_s - observation.capture_time_s,
    }


@dataclass(frozen=True)
class LandingAlignment:
    velocity_flu: tuple[float, float]
    heading_error_rad: float
    center_error_px: float
    source_age_s: float
    received_s: float

    @classmethod
    def from_payload(cls, payload, received_s) -> Optional["LandingAlignment"]:
        if not isinstance(payload, Mapping) or payload.get("frame") != "BODY_FLU":
            return None
        velocity = payload.get("velocity_flu")
        if not isinstance(velocity, (list, tuple)) or len(velocity) != 2:
            return None
        values = (*velocity, payload.get("heading_error_rad"),
                  payload.get("center_error_px"), payload.get("source_age_s"), received_s)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values):
            return None
        if values[3] < 0 or values[4] < 0 or abs(values[2]) > math.pi + 1e-9:
            return None
        return cls(tuple(velocity), *values[2:])

    def fresh(self, now_s, maximum_age_s):
        elapsed = now_s - self.received_s
        return math.isfinite(elapsed) and elapsed >= 0 and self.source_age_s + elapsed <= maximum_age_s
