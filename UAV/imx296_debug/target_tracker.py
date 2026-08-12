#!/usr/bin/env python3
"""Timestamped constant-velocity tracker for AprilTag target positions.

The tracker is frame-agnostic.  Measurements supplied to one instance must all
use the same Cartesian frame (BODY_FRD for bench replay or LOCAL_NED in flight).
It performs alpha-beta filtering, residual gating, and acquisition counting.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


Vector3 = tuple[float, float, float]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[i] + b[i] for i in range(3))  # type: ignore[return-value]


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return tuple(a[i] - b[i] for i in range(3))  # type: ignore[return-value]


def _scale(a: Vector3, value: float) -> Vector3:
    return tuple(component * value for component in a)  # type: ignore[return-value]


def _norm(a: Vector3) -> float:
    return math.sqrt(sum(component * component for component in a))


@dataclass(frozen=True)
class TargetMeasurement:
    timestamp_s: float
    position_m: Vector3
    decision_margin: float = 0.0
    hamming: int = 0
    reprojection_error_px: float = 0.0


@dataclass(frozen=True)
class TargetTrack:
    timestamp_s: float
    position_m: Vector3
    velocity_mps: Vector3
    accepted: bool
    acquired: bool
    consecutive_valid: int
    residual_m: float
    rejection_reason: Optional[str] = None


class AlphaBetaTargetTracker:
    def __init__(
        self,
        *,
        alpha: float = 0.65,
        beta: float = 0.08,
        max_residual_m: float = 0.25,
        min_dt_s: float = 0.02,
        max_dt_s: float = 0.5,
        acquire_count: int = 5,
    ) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if not 0.0 <= beta <= 1.0:
            raise ValueError("beta must be in [0, 1]")
        if max_residual_m <= 0 or min_dt_s <= 0 or max_dt_s <= min_dt_s:
            raise ValueError("invalid tracker timing or residual limits")
        if acquire_count <= 0:
            raise ValueError("acquire_count must be positive")
        self.alpha = alpha
        self.beta = beta
        self.max_residual_m = max_residual_m
        self.min_dt_s = min_dt_s
        self.max_dt_s = max_dt_s
        self.acquire_count = acquire_count
        self.reset()

    def reset(self) -> None:
        self._timestamp_s: Optional[float] = None
        self._position_m: Vector3 = (0.0, 0.0, 0.0)
        self._velocity_mps: Vector3 = (0.0, 0.0, 0.0)
        self._consecutive_valid = 0

    @property
    def initialized(self) -> bool:
        return self._timestamp_s is not None

    def update(self, measurement: TargetMeasurement) -> TargetTrack:
        if not all(math.isfinite(value) for value in measurement.position_m):
            return self._rejected(measurement.timestamp_s, "NON_FINITE_POSITION")
        if not math.isfinite(measurement.timestamp_s):
            return self._rejected(0.0, "NON_FINITE_TIMESTAMP")

        if self._timestamp_s is None:
            self._timestamp_s = measurement.timestamp_s
            self._position_m = measurement.position_m
            self._velocity_mps = (0.0, 0.0, 0.0)
            self._consecutive_valid = 1
            return self._track(measurement.timestamp_s, True, 0.0)

        dt = measurement.timestamp_s - self._timestamp_s
        if dt < self.min_dt_s:
            return self._rejected(measurement.timestamp_s, "TIMESTAMP_NOT_ADVANCING")
        if dt > self.max_dt_s:
            # A long gap invalidates velocity history, but the fresh position is
            # still useful as the first sample of a new acquisition.
            self._timestamp_s = measurement.timestamp_s
            self._position_m = measurement.position_m
            self._velocity_mps = (0.0, 0.0, 0.0)
            self._consecutive_valid = 1
            return self._track(measurement.timestamp_s, True, 0.0)

        predicted = _add(self._position_m, _scale(self._velocity_mps, dt))
        residual = _sub(measurement.position_m, predicted)
        residual_m = _norm(residual)
        if residual_m > self.max_residual_m:
            self._consecutive_valid = 0
            return self._rejected(
                measurement.timestamp_s,
                "RESIDUAL_LIMIT",
                residual_m=residual_m,
            )

        self._position_m = _add(predicted, _scale(residual, self.alpha))
        self._velocity_mps = _add(
            self._velocity_mps,
            _scale(residual, self.beta / dt),
        )
        self._timestamp_s = measurement.timestamp_s
        self._consecutive_valid += 1
        return self._track(measurement.timestamp_s, True, residual_m)

    def predict(self, timestamp_s: float) -> Optional[TargetTrack]:
        if self._timestamp_s is None or timestamp_s < self._timestamp_s:
            return None
        dt = timestamp_s - self._timestamp_s
        predicted = _add(self._position_m, _scale(self._velocity_mps, dt))
        return TargetTrack(
            timestamp_s=timestamp_s,
            position_m=predicted,
            velocity_mps=self._velocity_mps,
            accepted=False,
            acquired=self._consecutive_valid >= self.acquire_count,
            consecutive_valid=self._consecutive_valid,
            residual_m=0.0,
            rejection_reason="PREDICTED",
        )

    def _track(self, timestamp_s: float, accepted: bool, residual_m: float) -> TargetTrack:
        return TargetTrack(
            timestamp_s=timestamp_s,
            position_m=self._position_m,
            velocity_mps=self._velocity_mps,
            accepted=accepted,
            acquired=self._consecutive_valid >= self.acquire_count,
            consecutive_valid=self._consecutive_valid,
            residual_m=residual_m,
        )

    def _rejected(
        self,
        timestamp_s: float,
        reason: str,
        *,
        residual_m: float = 0.0,
    ) -> TargetTrack:
        return TargetTrack(
            timestamp_s=timestamp_s,
            position_m=self._position_m,
            velocity_mps=self._velocity_mps,
            accepted=False,
            acquired=False,
            consecutive_valid=self._consecutive_valid,
            residual_m=residual_m,
            rejection_reason=reason,
        )
