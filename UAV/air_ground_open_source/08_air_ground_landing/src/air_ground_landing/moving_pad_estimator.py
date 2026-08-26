"""Short-horizon moving-pad estimator for AprilTag and aligned UGV odometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional

from .math3d import add, clamp, norm, rotate_by_quaternion, scale, subtract
from .models import (
    LandingTargetObservation,
    MovingPadEstimate,
    UavState,
    UgvState,
    Vector3,
)


@dataclass(frozen=True)
class EstimatorConfig:
    vision_position_gain: float = 0.65
    vision_velocity_gain: float = 0.10
    ugv_position_gain: float = 0.45
    ugv_velocity_gain: float = 0.80
    maximum_vision_residual_m: float = 0.75
    maximum_ugv_residual_m: float = 1.50
    minimum_vision_quality: float = 0.35
    maximum_uav_state_age_s: float = 0.20
    maximum_source_age_s: float = 0.60
    maximum_prediction_horizon_s: float = 0.50
    process_noise_m2_per_s: float = 0.02

    @classmethod
    def from_mapping(cls, root: Mapping[str, Any]) -> "EstimatorConfig":
        values = root.get("moving_pad_estimator", {})
        return cls(
            vision_position_gain=float(values.get("vision_position_gain", 0.65)),
            vision_velocity_gain=float(values.get("vision_velocity_gain", 0.10)),
            ugv_position_gain=float(values.get("ugv_position_gain", 0.45)),
            ugv_velocity_gain=float(values.get("ugv_velocity_gain", 0.80)),
            maximum_vision_residual_m=float(values.get("maximum_vision_residual_m", 0.75)),
            maximum_ugv_residual_m=float(values.get("maximum_ugv_residual_m", 1.50)),
            minimum_vision_quality=float(values.get("minimum_vision_quality", 0.35)),
            maximum_uav_state_age_s=float(values.get("maximum_uav_state_age_s", 0.20)),
            maximum_source_age_s=float(values.get("maximum_source_age_s", 0.60)),
            maximum_prediction_horizon_s=float(values.get("maximum_prediction_horizon_s", 0.50)),
            process_noise_m2_per_s=float(values.get("process_noise_m2_per_s", 0.02)),
        )


class MovingPadEstimator:
    """Constant-velocity fusion with source freshness and residual gates.

    ``UgvState`` is accepted only when the adapter has positively aligned its
    odometry origin with the UAV's LOCAL_NED origin.  Otherwise vision remains
    the position source and no accidental frame mixing occurs.
    """

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self._timestamp_s: Optional[float] = None
        self._position_m: Vector3 = (0.0, 0.0, 0.0)
        self._velocity_mps: Vector3 = (0.0, 0.0, 0.0)
        self._variance_m2: Vector3 = (1.0, 1.0, 1.0)
        self._last_vision_s: Optional[float] = None
        self._last_ugv_s: Optional[float] = None
        self._active_sources: set[str] = set()
        self.last_rejection_reason: Optional[str] = None
        self._validate_config()

    @property
    def initialized(self) -> bool:
        return self._timestamp_s is not None

    def reset(self) -> None:
        self.__init__(self.config)

    def update_vision(
        self,
        observation: LandingTargetObservation,
        uav_state: UavState,
    ) -> Optional[MovingPadEstimate]:
        timestamp_s = float(observation.capture_time_s)
        state_age_s = abs(timestamp_s - float(uav_state.timestamp_s))
        if state_age_s > self.config.maximum_uav_state_age_s:
            self.last_rejection_reason = "UAV_STATE_NOT_TIME_ALIGNED"
            return self.estimate(observation.received_time_s)
        if observation.quality < self.config.minimum_vision_quality:
            self.last_rejection_reason = "VISION_QUALITY_LIMIT"
            return self.estimate(observation.received_time_s)
        try:
            relative_ned = rotate_by_quaternion(
                observation.position_body_frd_m,
                uav_state.quaternion_body_to_ned,
            )
        except ValueError:
            self.last_rejection_reason = "INVALID_UAV_ATTITUDE"
            return self.estimate(observation.received_time_s)
        measured_position = add(uav_state.position_ned_m, relative_ned)
        measurement_variance = (
            float(observation.covariance_m2[0]),
            float(observation.covariance_m2[4]),
            float(observation.covariance_m2[8]),
        )
        accepted = self._fuse_position(
            timestamp_s=timestamp_s,
            measured_position=measured_position,
            initial_velocity=uav_state.velocity_ned_mps,
            position_gain=self.config.vision_position_gain * clamp(observation.quality, 0.2, 1.0),
            velocity_gain=self.config.vision_velocity_gain,
            maximum_residual_m=self.config.maximum_vision_residual_m,
            measurement_variance=measurement_variance,
        )
        if accepted:
            self._last_vision_s = timestamp_s
            self._active_sources.add("APRILTAG")
            self.last_rejection_reason = None
        else:
            self.last_rejection_reason = "VISION_RESIDUAL_LIMIT"
        return self.estimate(observation.received_time_s)

    def update_ugv(self, ugv_state: UgvState) -> Optional[MovingPadEstimate]:
        if not ugv_state.common_origin_valid:
            self.last_rejection_reason = "UGV_COMMON_ORIGIN_NOT_VALID"
            return self.estimate(ugv_state.timestamp_s)
        if not ugv_state.healthy or ugv_state.emergency_stop:
            self.last_rejection_reason = "UGV_STATE_UNHEALTHY"
            return self.estimate(ugv_state.timestamp_s)
        accepted = self._fuse_position(
            timestamp_s=ugv_state.timestamp_s,
            measured_position=ugv_state.position_ned_m,
            initial_velocity=ugv_state.velocity_ned_mps,
            position_gain=self.config.ugv_position_gain,
            velocity_gain=0.0,
            maximum_residual_m=self.config.maximum_ugv_residual_m,
            measurement_variance=(0.01, 0.01, 0.02),
        )
        if not accepted:
            self.last_rejection_reason = "UGV_RESIDUAL_LIMIT"
            return self.estimate(ugv_state.timestamp_s)
        gain = clamp(self.config.ugv_velocity_gain, 0.0, 1.0)
        self._velocity_mps = add(
            scale(self._velocity_mps, 1.0 - gain),
            scale(ugv_state.velocity_ned_mps, gain),
        )
        self._last_ugv_s = float(ugv_state.timestamp_s)
        self._active_sources.add("UGV_ODOMETRY")
        self.last_rejection_reason = None
        return self.estimate(ugv_state.timestamp_s)

    def estimate(self, timestamp_s: float) -> Optional[MovingPadEstimate]:
        if self._timestamp_s is None:
            return None
        now_s = float(timestamp_s)
        if now_s < self._timestamp_s:
            now_s = self._timestamp_s
        requested_horizon = now_s - self._timestamp_s
        prediction_horizon = min(
            requested_horizon,
            self.config.maximum_prediction_horizon_s,
        )
        predicted_position = add(
            self._position_m,
            scale(self._velocity_mps, prediction_horizon),
        )
        predicted_variance = tuple(
            value + self.config.process_noise_m2_per_s * prediction_horizon
            for value in self._variance_m2
        )
        vision_age = None if self._last_vision_s is None else max(0.0, now_s - self._last_vision_s)
        ugv_age = None if self._last_ugv_s is None else max(0.0, now_s - self._last_ugv_s)
        freshness = max(
            self._freshness(vision_age),
            self._freshness(ugv_age),
        )
        uncertainty_score = 1.0 / (1.0 + math.sqrt(sum(predicted_variance)))
        quality = clamp(0.75 * freshness + 0.25 * uncertainty_score, 0.0, 1.0)
        covariance = (
            predicted_variance[0], 0.0, 0.0,
            0.0, predicted_variance[1], 0.0,
            0.0, 0.0, predicted_variance[2],
        )
        fresh_sources = []
        if "APRILTAG" in self._active_sources and self._freshness(vision_age) > 0.0:
            fresh_sources.append("APRILTAG")
        if "UGV_ODOMETRY" in self._active_sources and self._freshness(ugv_age) > 0.0:
            fresh_sources.append("UGV_ODOMETRY")
        return MovingPadEstimate(
            timestamp_s=now_s,
            position_ned_m=predicted_position,
            velocity_ned_mps=self._velocity_mps,
            covariance_m2=covariance,
            quality=quality,
            sources=tuple(fresh_sources),
            vision_age_s=vision_age,
            ugv_age_s=ugv_age,
        )

    def _fuse_position(
        self,
        *,
        timestamp_s: float,
        measured_position: Vector3,
        initial_velocity: Vector3,
        position_gain: float,
        velocity_gain: float,
        maximum_residual_m: float,
        measurement_variance: Vector3,
    ) -> bool:
        if not all(math.isfinite(value) for value in (*measured_position, timestamp_s)):
            return False
        if self._timestamp_s is None:
            self._timestamp_s = float(timestamp_s)
            self._position_m = measured_position
            self._velocity_mps = initial_velocity
            self._variance_m2 = measurement_variance
            return True
        if timestamp_s < self._timestamp_s - 1.0e-6:
            return False
        dt = max(0.0, float(timestamp_s) - self._timestamp_s)
        predicted = add(self._position_m, scale(self._velocity_mps, dt))
        residual = subtract(measured_position, predicted)
        if norm(residual) > maximum_residual_m:
            return False
        alpha = clamp(position_gain, 0.0, 1.0)
        self._position_m = add(predicted, scale(residual, alpha))
        if dt > 1.0e-3 and velocity_gain > 0.0:
            self._velocity_mps = add(
                self._velocity_mps,
                scale(residual, clamp(velocity_gain, 0.0, 1.0) / dt),
            )
        self._variance_m2 = tuple(
            max(1.0e-6, (1.0 - alpha) * current + alpha * measured)
            for current, measured in zip(self._variance_m2, measurement_variance)
        )  # type: ignore[assignment]
        self._timestamp_s = float(timestamp_s)
        return True

    def _freshness(self, age_s: Optional[float]) -> float:
        if age_s is None:
            return 0.0
        return clamp(
            1.0 - age_s / max(self.config.maximum_source_age_s, 1.0e-6),
            0.0,
            1.0,
        )

    def _validate_config(self) -> None:
        for value in (
            self.config.vision_position_gain,
            self.config.vision_velocity_gain,
            self.config.ugv_position_gain,
            self.config.ugv_velocity_gain,
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("estimator gains must be in [0, 1]")
        if min(
            self.config.maximum_vision_residual_m,
            self.config.maximum_ugv_residual_m,
            self.config.maximum_uav_state_age_s,
            self.config.maximum_source_age_s,
            self.config.maximum_prediction_horizon_s,
        ) <= 0.0:
            raise ValueError("estimator limits must be positive")
