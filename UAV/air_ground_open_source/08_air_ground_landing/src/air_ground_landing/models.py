"""Shared, dependency-free data contracts for moving-platform landing."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Optional


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class LandingTargetObservation:
    """Quality-gated AprilTag observation expressed in aircraft BODY_FRD."""

    capture_time_s: float
    received_time_s: float
    wall_time_usec: int
    tag_id: int
    tag_size_m: float
    position_body_frd_m: Vector3
    distance_m: float
    decision_margin: float
    hamming: int
    reprojection_error_px: float
    quality: float
    covariance_m2: tuple[float, ...]
    source_sequence: Optional[int] = None

    @property
    def age_s(self) -> float:
        return max(0.0, self.received_time_s - self.capture_time_s)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LandingTargetPacket:
    """MAVLink LANDING_TARGET fields without a transport dependency."""

    time_usec: int
    target_num: int
    frame: int
    angle_x: float
    angle_y: float
    distance: float
    size_x: float
    size_y: float
    x: float
    y: float
    z: float
    q: Quaternion
    type: int
    position_valid: int

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BridgeResult:
    observation: Optional[LandingTargetObservation]
    packet: Optional[LandingTargetPacket]
    reason: str
    target_lost: bool

    @property
    def accepted(self) -> bool:
        return self.observation is not None


@dataclass(frozen=True)
class UavState:
    """Aircraft state in LOCAL_NED; quaternion rotates BODY_FRD into NED."""

    timestamp_s: float
    position_ned_m: Vector3
    velocity_ned_mps: Vector3
    quaternion_body_to_ned: Quaternion = (1.0, 0.0, 0.0, 0.0)
    mode: str = "UNKNOWN"
    armed: bool = False
    landed: Optional[bool] = None
    link_healthy: bool = False
    velocity_source_independent_of_deck: bool = False


@dataclass(frozen=True)
class UgvState:
    """Landing-pad state already aligned to the UAV LOCAL_NED origin."""

    timestamp_s: float
    position_ned_m: Vector3
    velocity_ned_mps: Vector3
    yaw_rad: float = 0.0
    yaw_rate_rps: float = 0.0
    healthy: bool = False
    emergency_stop: bool = False
    common_origin_valid: bool = False

    @property
    def horizontal_speed_mps(self) -> float:
        x, y, _ = self.velocity_ned_mps
        return (x * x + y * y) ** 0.5


@dataclass(frozen=True)
class MovingPadEstimate:
    timestamp_s: float
    position_ned_m: Vector3
    velocity_ned_mps: Vector3
    covariance_m2: tuple[float, ...]
    quality: float
    sources: tuple[str, ...]
    vision_age_s: Optional[float]
    ugv_age_s: Optional[float]

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

