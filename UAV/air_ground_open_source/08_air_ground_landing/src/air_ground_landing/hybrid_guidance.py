"""Single-writer guidance arbitration for Elastic, IBVS and AC_PrecLand.

This module deliberately produces requests only.  It never opens MAVLink, changes
flight mode, arms, disarms, or publishes a motor command.  The intent is to make
the hand-over rules executable and testable before any ROS/MAVLink adapter is
allowed to act on them.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Iterable, Mapping, Optional

from .math3d import (
    Matrix3,
    add,
    clamp,
    horizontal_norm,
    rotate_ned_to_body,
    scale,
    transform_point,
    validate_rotation,
)
from .models import LandingTargetObservation, MovingPadEstimate, UavState, Vector3
from .moving_landing_supervisor import LandingState, SupervisorDecision


PixelPoint = tuple[float, float]


class IbvsMode(str, Enum):
    UNAVAILABLE = "UNAVAILABLE"
    IBVS_2DOF = "IBVS_2DOF"
    IBVS_4DOF = "IBVS_4DOF"


class ControlOwner(str, Enum):
    """The only subsystem allowed to provide guidance in the current phase."""

    NONE = "NONE"
    HOLD = "HOLD"
    ELASTIC_GUIDED = "ELASTIC_GUIDED"
    IBVS_GUIDED = "IBVS_GUIDED"
    AC_PRECLAND_LAND = "AC_PRECLAND_LAND"


@dataclass(frozen=True)
class IbvsConfig:
    image_width: int
    image_height: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    rotation_camera_to_body: Matrix3
    outer_tag_id: int = 0
    inner_tag_id: int = 1
    minimum_quality: float = 0.45
    maximum_feature_age_s: float = 0.15
    enter_4dof_error_px: float = 80.0
    exit_4dof_error_px: float = 120.0
    alignment_error_px: float = 35.0
    final_alignment_error_px: float = 18.0
    maximum_edge_asymmetry_ratio: float = 0.35
    horizontal_gain_per_s: float = 0.70
    maximum_horizontal_correction_mps: float = 0.25
    require_inner_tag_for_final: bool = True

    @classmethod
    def from_mapping(cls, root: Mapping[str, Any]) -> "IbvsConfig":
        hybrid = root.get("hybrid_guidance", {})
        camera = hybrid.get("camera", {})
        ibvs = hybrid.get("ibvs", {})
        bridge_camera = root.get("landing_target_bridge", {}).get("camera_to_body", {})
        rotation_value = camera.get(
            "rotation_camera_optical_to_body_frd",
            bridge_camera.get("rotation_camera_optical_to_body_frd"),
        )
        if rotation_value is None:
            raise ValueError("IBVS camera-to-body rotation is required")
        config = cls(
            image_width=int(camera.get("image_width", 1280)),
            image_height=int(camera.get("image_height", 800)),
            fx_px=float(camera.get("fx_px", 797.7670963230249)),
            fy_px=float(camera.get("fy_px", 805.5898276011428)),
            cx_px=float(camera.get("cx_px", 649.741100705223)),
            cy_px=float(camera.get("cy_px", 430.2853449683943)),
            rotation_camera_to_body=validate_rotation(rotation_value),
            outer_tag_id=int(ibvs.get("outer_tag_id", 0)),
            inner_tag_id=int(ibvs.get("inner_tag_id", 1)),
            minimum_quality=float(ibvs.get("minimum_quality", 0.45)),
            maximum_feature_age_s=float(ibvs.get("maximum_feature_age_s", 0.15)),
            enter_4dof_error_px=float(ibvs.get("enter_4dof_error_px", 80.0)),
            exit_4dof_error_px=float(ibvs.get("exit_4dof_error_px", 120.0)),
            alignment_error_px=float(ibvs.get("alignment_error_px", 35.0)),
            final_alignment_error_px=float(ibvs.get("final_alignment_error_px", 18.0)),
            maximum_edge_asymmetry_ratio=float(ibvs.get("maximum_edge_asymmetry_ratio", 0.35)),
            horizontal_gain_per_s=float(ibvs.get("horizontal_gain_per_s", 0.70)),
            maximum_horizontal_correction_mps=float(
                ibvs.get("maximum_horizontal_correction_mps", 0.25)
            ),
            require_inner_tag_for_final=bool(ibvs.get("require_inner_tag_for_final", True)),
        )
        config.validate()
        return config

    def validate(self) -> None:
        positive = (
            self.image_width,
            self.image_height,
            self.fx_px,
            self.fy_px,
            self.maximum_feature_age_s,
            self.enter_4dof_error_px,
            self.exit_4dof_error_px,
            self.alignment_error_px,
            self.final_alignment_error_px,
            self.maximum_horizontal_correction_mps,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("IBVS dimensions, intrinsics and limits must be positive")
        if self.enter_4dof_error_px >= self.exit_4dof_error_px:
            raise ValueError("IBVS hysteresis enter threshold must be below exit threshold")
        if not 0.0 <= self.minimum_quality <= 1.0:
            raise ValueError("IBVS minimum_quality must be in [0, 1]")
        if not 0.0 <= self.maximum_edge_asymmetry_ratio <= 1.0:
            raise ValueError("IBVS edge asymmetry limit must be in [0, 1]")
        if self.outer_tag_id == self.inner_tag_id:
            raise ValueError("outer and inner IBVS tag IDs must differ")


@dataclass(frozen=True)
class IbvsFeatureResult:
    timestamp_s: Optional[float]
    valid: bool
    reason: str
    mode: IbvsMode
    tag_id: Optional[int]
    tag_role: Optional[str]
    corners_px: tuple[PixelPoint, ...]
    centroid_px: Optional[PixelPoint]
    pixel_error_px: Optional[PixelPoint]
    centroid_error_px: Optional[float]
    edge_asymmetry_ratio: Optional[float]
    correction_body_frd_mps: Vector3
    aligned: bool
    final_ready: bool

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["mode"] = self.mode.value
        return result


class IbvsFeatureController:
    """Port the useful feature-space logic from ibvs_sim without actuation.

    The far-from-centre mode uses only centroid translation (2-DOF).  Once the
    centroid is inside a hysteresis band, the result is labelled 4-DOF so scale,
    shape and tag hand-over can be used as gates.  Only horizontal velocity
    correction is emitted; vertical/yaw control remains with ArduPilot.
    """

    def __init__(self, config: IbvsConfig) -> None:
        self.config = config
        self._mode = IbvsMode.IBVS_2DOF

    def process_status(
        self,
        status: Optional[Mapping[str, Any]],
        observation: Optional[LandingTargetObservation],
        *,
        now_s: float,
        final_approach: bool = False,
    ) -> IbvsFeatureResult:
        if status is None or observation is None:
            return self._invalid("NO_QUALITY_GATED_FEATURE_OBSERVATION")
        age_s = float(now_s) - observation.capture_time_s
        if age_s < 0.0 or age_s > self.config.maximum_feature_age_s:
            return self._invalid("FEATURES_STALE", observation.capture_time_s)
        if observation.quality < self.config.minimum_quality:
            return self._invalid("FEATURE_QUALITY_LOW", observation.capture_time_s)
        if observation.tag_id not in (self.config.outer_tag_id, self.config.inner_tag_id):
            return self._invalid("UNSUPPORTED_IBVS_TAG", observation.capture_time_s)
        try:
            reported_size = tuple(int(value) for value in status.get("analysis_size", ()))
        except (TypeError, ValueError):
            return self._invalid("IBVS_IMAGE_SIZE_MISMATCH", observation.capture_time_s)
        if reported_size != (self.config.image_width, self.config.image_height):
            return self._invalid("IBVS_IMAGE_SIZE_MISMATCH", observation.capture_time_s)

        raw_corners = status.get("corners_px", status.get("overlay_points"))
        try:
            corners = self._corners(raw_corners)
        except (TypeError, ValueError):
            return self._invalid("APRILTAG_CORNERS_MISSING_OR_INVALID", observation.capture_time_s)
        if any(
            x < 0.0 or x >= self.config.image_width or y < 0.0 or y >= self.config.image_height
            for x, y in corners
        ):
            return self._invalid("APRILTAG_CORNERS_OUTSIDE_IMAGE", observation.capture_time_s)

        centroid = (
            sum(point[0] for point in corners) / 4.0,
            sum(point[1] for point in corners) / 4.0,
        )
        pixel_error = (centroid[0] - self.config.cx_px, centroid[1] - self.config.cy_px)
        centroid_error = math.hypot(*pixel_error)
        edges = tuple(
            math.hypot(
                corners[(index + 1) % 4][0] - corners[index][0],
                corners[(index + 1) % 4][1] - corners[index][1],
            )
            for index in range(4)
        )
        mean_edge = sum(edges) / 4.0
        if mean_edge <= 1.0:
            return self._invalid("APRILTAG_TOO_SMALL_IN_IMAGE", observation.capture_time_s)
        edge_asymmetry = (max(edges) - min(edges)) / mean_edge

        if observation.tag_id == self.config.inner_tag_id:
            self._mode = IbvsMode.IBVS_4DOF
        elif self._mode == IbvsMode.IBVS_2DOF and centroid_error <= self.config.enter_4dof_error_px:
            self._mode = IbvsMode.IBVS_4DOF
        elif self._mode == IbvsMode.IBVS_4DOF and centroid_error >= self.config.exit_4dof_error_px:
            self._mode = IbvsMode.IBVS_2DOF

        # Reduced image Jacobian: v_camera ~= gain * Z * normalized pixel error.
        # The result is a request only; z/yaw are intentionally left to ArduPilot.
        correction_camera = (
            self.config.horizontal_gain_per_s * observation.distance_m * pixel_error[0] / self.config.fx_px,
            self.config.horizontal_gain_per_s * observation.distance_m * pixel_error[1] / self.config.fy_px,
            0.0,
        )
        correction_body = transform_point(
            self.config.rotation_camera_to_body,
            (0.0, 0.0, 0.0),
            correction_camera,
        )
        correction_body = self._limit_horizontal(
            (correction_body[0], correction_body[1], 0.0),
            self.config.maximum_horizontal_correction_mps,
        )

        alignment_limit = (
            self.config.final_alignment_error_px
            if final_approach
            else self.config.alignment_error_px
        )
        aligned = bool(
            self._mode == IbvsMode.IBVS_4DOF
            and centroid_error <= alignment_limit
            and edge_asymmetry <= self.config.maximum_edge_asymmetry_ratio
        )
        tag_role = "inner" if observation.tag_id == self.config.inner_tag_id else "outer"
        final_ready = bool(
            aligned
            and (
                not self.config.require_inner_tag_for_final
                or observation.tag_id == self.config.inner_tag_id
            )
        )
        reason = "FEATURES_VALID"
        if final_approach and aligned and not final_ready:
            reason = "FINAL_INNER_TAG_REQUIRED"
        elif not aligned:
            reason = "FEATURES_VALID_NOT_ALIGNED"
        return IbvsFeatureResult(
            timestamp_s=observation.capture_time_s,
            valid=True,
            reason=reason,
            mode=self._mode,
            tag_id=observation.tag_id,
            tag_role=tag_role,
            corners_px=corners,
            centroid_px=centroid,
            pixel_error_px=pixel_error,
            centroid_error_px=centroid_error,
            edge_asymmetry_ratio=edge_asymmetry,
            correction_body_frd_mps=correction_body,
            aligned=aligned,
            final_ready=final_ready,
        )

    @staticmethod
    def _corners(values: Any) -> tuple[PixelPoint, PixelPoint, PixelPoint, PixelPoint]:
        if not isinstance(values, Iterable):
            raise ValueError("corners are not iterable")
        points = []
        for value in values:
            if not isinstance(value, Iterable):
                raise ValueError("corner is not iterable")
            pair = tuple(float(component) for component in value)
            if len(pair) != 2 or not all(math.isfinite(component) for component in pair):
                raise ValueError("corner must contain two finite values")
            points.append((pair[0], pair[1]))
        if len(points) != 4:
            raise ValueError("exactly four corners are required")
        return tuple(points)  # type: ignore[return-value]

    @staticmethod
    def _limit_horizontal(vector: Vector3, maximum: float) -> Vector3:
        magnitude = horizontal_norm(vector)
        if magnitude <= maximum or magnitude <= 1.0e-9:
            return vector
        return scale(vector, maximum / magnitude)

    def _invalid(
        self,
        reason: str,
        timestamp_s: Optional[float] = None,
    ) -> IbvsFeatureResult:
        return IbvsFeatureResult(
            timestamp_s=timestamp_s,
            valid=False,
            reason=reason,
            mode=IbvsMode.UNAVAILABLE,
            tag_id=None,
            tag_role=None,
            corners_px=(),
            centroid_px=None,
            pixel_error_px=None,
            centroid_error_px=None,
            edge_asymmetry_ratio=None,
            correction_body_frd_mps=(0.0, 0.0, 0.0),
            aligned=False,
            final_ready=False,
        )


@dataclass(frozen=True)
class ElasticTrackerStatus:
    timestamp_s: float
    heartbeat_healthy: bool
    map_fresh: bool
    target_prediction_fresh: bool
    trajectory_valid: bool
    visibility_corridor_valid: bool
    trajectory_id: Optional[int] = None

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        timestamp_s: float,
    ) -> "ElasticTrackerStatus":
        trajectory_id = data.get("trajectory_id")
        return cls(
            timestamp_s=float(data.get("timestamp_s", timestamp_s)),
            heartbeat_healthy=bool(data.get("heartbeat_healthy", False)),
            map_fresh=bool(data.get("map_fresh", False)),
            target_prediction_fresh=bool(data.get("target_prediction_fresh", False)),
            trajectory_valid=bool(data.get("trajectory_valid", False)),
            visibility_corridor_valid=bool(data.get("visibility_corridor_valid", False)),
            trajectory_id=None if trajectory_id is None else int(trajectory_id),
        )


@dataclass(frozen=True)
class HybridGuidanceConfig:
    maximum_elastic_status_age_s: float = 0.25
    require_visibility_corridor: bool = True
    ibvs_alignment_hold_s: float = 0.40
    require_ibvs_for_precland_handover: bool = True
    maximum_target_feedforward_mps: float = 0.40
    maximum_total_horizontal_speed_mps: float = 0.50

    @classmethod
    def from_mapping(cls, root: Mapping[str, Any]) -> "HybridGuidanceConfig":
        hybrid = root.get("hybrid_guidance", {})
        elastic = hybrid.get("elastic_tracker", {})
        arbitration = hybrid.get("arbitration", {})
        config = cls(
            maximum_elastic_status_age_s=float(elastic.get("maximum_status_age_s", 0.25)),
            require_visibility_corridor=bool(elastic.get("require_visibility_corridor", True)),
            ibvs_alignment_hold_s=float(arbitration.get("ibvs_alignment_hold_s", 0.40)),
            require_ibvs_for_precland_handover=bool(
                arbitration.get("require_ibvs_for_precland_handover", True)
            ),
            maximum_target_feedforward_mps=float(
                arbitration.get("maximum_target_feedforward_mps", 0.40)
            ),
            maximum_total_horizontal_speed_mps=float(
                arbitration.get("maximum_total_horizontal_speed_mps", 0.50)
            ),
        )
        if min(
            config.maximum_elastic_status_age_s,
            config.maximum_target_feedforward_mps,
            config.maximum_total_horizontal_speed_mps,
        ) <= 0.0 or config.ibvs_alignment_hold_s < 0.0:
            raise ValueError("hybrid guidance timing and speed limits are invalid")
        return config


@dataclass(frozen=True)
class HybridGuidanceInputs:
    timestamp_s: float
    supervisor: SupervisorDecision
    uav: UavState
    pad: Optional[MovingPadEstimate]
    elastic: Optional[ElasticTrackerStatus]
    ibvs: Optional[IbvsFeatureResult]


@dataclass(frozen=True)
class HybridGuidanceDecision:
    control_owner: ControlOwner
    reason: str
    requested_flight_mode: Optional[str]
    elastic_trajectory_authorized: bool
    ibvs_velocity_authorized: bool
    ac_precland_authorized: bool
    landing_target_stream_required: bool
    vertical_descent_authorized: bool
    requested_body_velocity_frd_mps: Optional[Vector3]

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["control_owner"] = self.control_owner.value
        return result


class HybridGuidanceCoordinator:
    """Choose exactly one control owner for each supervisor phase."""

    def __init__(self, config: HybridGuidanceConfig, ibvs_config: IbvsConfig) -> None:
        self.config = config
        self.ibvs_config = ibvs_config
        self._ibvs_aligned_since_s: Optional[float] = None

    def decide(self, inputs: HybridGuidanceInputs) -> HybridGuidanceDecision:
        state = inputs.supervisor.state
        if state in (LandingState.IDLE, LandingState.COMPLETE, LandingState.TOUCHDOWN):
            self._ibvs_aligned_since_s = None
            return self._decision(ControlOwner.NONE, f"NO_GUIDANCE_IN_{state.value}")
        if state == LandingState.ABORT:
            self._ibvs_aligned_since_s = None
            return self._decision(ControlOwner.HOLD, "SUPERVISOR_ABORT")

        if state in (LandingState.RENDEZVOUS, LandingState.TRACK_PAD):
            self._ibvs_aligned_since_s = None
            if self._elastic_ready(inputs):
                return self._decision(ControlOwner.ELASTIC_GUIDED, "ELASTIC_RENDEZVOUS_TRACK")
            return self._decision(ControlOwner.HOLD, "ELASTIC_NOT_READY")

        if state == LandingState.MATCH_VELOCITY:
            self._ibvs_aligned_since_s = None
            if self._ibvs_ready(inputs):
                return self._ibvs_decision(inputs, "IBVS_ALIGNMENT_AND_VELOCITY_MATCH")
            if self._elastic_ready(inputs):
                return self._decision(ControlOwner.ELASTIC_GUIDED, "IBVS_UNAVAILABLE_USE_ELASTIC_TRACK")
            return self._decision(ControlOwner.HOLD, "NO_MATCH_VELOCITY_GUIDANCE")

        if state == LandingState.DESCEND:
            if not inputs.supervisor.publish_landing_target:
                self._ibvs_aligned_since_s = None
                return self._decision(ControlOwner.HOLD, "LANDING_TARGET_STREAM_UNAVAILABLE")
            ibvs_ready = self._ibvs_ready(inputs)
            ibvs_aligned = bool(ibvs_ready and inputs.ibvs and inputs.ibvs.aligned)
            if self.config.require_ibvs_for_precland_handover:
                if not ibvs_aligned:
                    self._ibvs_aligned_since_s = None
                    if ibvs_ready:
                        return self._ibvs_decision(inputs, "CONTINUE_IBVS_BEFORE_PRECLAND_HANDOVER")
                    return self._decision(ControlOwner.HOLD, "IBVS_HANDOVER_GATE_NOT_READY")
                if self._ibvs_aligned_since_s is None:
                    self._ibvs_aligned_since_s = float(inputs.timestamp_s)
                if (
                    float(inputs.timestamp_s) - self._ibvs_aligned_since_s
                    < self.config.ibvs_alignment_hold_s
                ):
                    return self._ibvs_decision(inputs, "VERIFYING_IBVS_HANDOVER_STABILITY")
            if not inputs.supervisor.descent_authorized:
                if ibvs_ready:
                    return self._ibvs_decision(inputs, "DESCENT_GATE_CLOSED_CONTINUE_ALIGNMENT")
                return self._decision(ControlOwner.HOLD, "DESCENT_GATE_CLOSED")
            return self._decision(
                ControlOwner.AC_PRECLAND_LAND,
                "HANDOVER_TO_AC_PRECLAND",
                vertical_descent=True,
            )

        if state == LandingState.FINAL_APPROACH:
            if not inputs.supervisor.publish_landing_target:
                return self._decision(ControlOwner.HOLD, "FINAL_LANDING_TARGET_UNAVAILABLE")
            if not self._ibvs_ready(inputs):
                return self._decision(ControlOwner.HOLD, "FINAL_IBVS_FEATURES_UNAVAILABLE")
            if self.ibvs_config.require_inner_tag_for_final and not inputs.ibvs.final_ready:
                return self._decision(ControlOwner.HOLD, "FINAL_INNER_TAG_OR_ALIGNMENT_NOT_READY")
            if not inputs.supervisor.descent_authorized:
                return self._decision(ControlOwner.HOLD, "FINAL_DESCENT_GATE_CLOSED")
            return self._decision(
                ControlOwner.AC_PRECLAND_LAND,
                "AC_PRECLAND_FINAL_APPROACH",
                vertical_descent=True,
            )

        return self._decision(ControlOwner.HOLD, "UNHANDLED_SUPERVISOR_STATE")

    def _elastic_ready(self, inputs: HybridGuidanceInputs) -> bool:
        elastic = inputs.elastic
        if elastic is None:
            return False
        age_s = float(inputs.timestamp_s) - elastic.timestamp_s
        return bool(
            0.0 <= age_s <= self.config.maximum_elastic_status_age_s
            and elastic.heartbeat_healthy
            and elastic.map_fresh
            and elastic.target_prediction_fresh
            and elastic.trajectory_valid
            and (
                elastic.visibility_corridor_valid
                or not self.config.require_visibility_corridor
            )
        )

    def _ibvs_ready(self, inputs: HybridGuidanceInputs) -> bool:
        ibvs = inputs.ibvs
        if ibvs is None or not ibvs.valid or ibvs.timestamp_s is None:
            return False
        age_s = float(inputs.timestamp_s) - ibvs.timestamp_s
        return 0.0 <= age_s <= self.ibvs_config.maximum_feature_age_s

    def _ibvs_decision(
        self,
        inputs: HybridGuidanceInputs,
        reason: str,
    ) -> HybridGuidanceDecision:
        if inputs.ibvs is None or inputs.pad is None:
            return self._decision(ControlOwner.HOLD, "IBVS_OR_PAD_ESTIMATE_MISSING")
        feedforward_body = rotate_ned_to_body(
            inputs.pad.velocity_ned_mps,
            inputs.uav.quaternion_body_to_ned,
        )
        feedforward_body = self._limit_horizontal(
            (feedforward_body[0], feedforward_body[1], 0.0),
            self.config.maximum_target_feedforward_mps,
        )
        requested = add(feedforward_body, inputs.ibvs.correction_body_frd_mps)
        requested = self._limit_horizontal(
            (requested[0], requested[1], 0.0),
            self.config.maximum_total_horizontal_speed_mps,
        )
        return self._decision(
            ControlOwner.IBVS_GUIDED,
            reason,
            body_velocity=requested,
        )

    @staticmethod
    def _limit_horizontal(vector: Vector3, maximum: float) -> Vector3:
        magnitude = horizontal_norm(vector)
        if magnitude <= maximum or magnitude <= 1.0e-9:
            return vector
        return scale(vector, maximum / magnitude)

    @staticmethod
    def _decision(
        owner: ControlOwner,
        reason: str,
        *,
        vertical_descent: bool = False,
        body_velocity: Optional[Vector3] = None,
    ) -> HybridGuidanceDecision:
        return HybridGuidanceDecision(
            control_owner=owner,
            reason=reason,
            requested_flight_mode={
                ControlOwner.ELASTIC_GUIDED: "GUIDED",
                ControlOwner.IBVS_GUIDED: "GUIDED",
                ControlOwner.AC_PRECLAND_LAND: "LAND",
                ControlOwner.HOLD: "HOLD",
                ControlOwner.NONE: None,
            }[owner],
            elastic_trajectory_authorized=owner == ControlOwner.ELASTIC_GUIDED,
            ibvs_velocity_authorized=owner == ControlOwner.IBVS_GUIDED,
            ac_precland_authorized=owner == ControlOwner.AC_PRECLAND_LAND,
            landing_target_stream_required=owner == ControlOwner.AC_PRECLAND_LAND,
            vertical_descent_authorized=bool(
                owner == ControlOwner.AC_PRECLAND_LAND and vertical_descent
            ),
            requested_body_velocity_frd_mps=(
                body_velocity if owner == ControlOwner.IBVS_GUIDED else None
            ),
        )
