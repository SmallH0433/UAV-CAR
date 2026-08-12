#!/usr/bin/env python3
"""Bounded horizontal velocity controller for moving-target following."""

from __future__ import annotations

import math
from dataclasses import dataclass


Vector2 = tuple[float, float]


def _norm(value: Vector2) -> float:
    return math.hypot(value[0], value[1])


def _limit_norm(value: Vector2, maximum: float) -> Vector2:
    magnitude = _norm(value)
    if magnitude <= maximum or magnitude <= 1e-12:
        return value
    scale = maximum / magnitude
    return value[0] * scale, value[1] * scale


@dataclass(frozen=True)
class FollowCommand:
    timestamp_s: float
    velocity_ned_mps: tuple[float, float, float]
    error_xy_m: Vector2
    target_feedforward_xy_mps: Vector2
    speed_limited: bool
    acceleration_limited: bool


class HorizontalFollowController:
    def __init__(
        self,
        *,
        kp_xy: float = 0.4,
        deadband_m: float = 0.05,
        max_speed_mps: float = 0.2,
        max_accel_mps2: float = 0.2,
        max_feedforward_mps: float = 0.5,
    ) -> None:
        if kp_xy <= 0 or deadband_m < 0:
            raise ValueError("invalid controller gain or deadband")
        if min(max_speed_mps, max_accel_mps2, max_feedforward_mps) <= 0:
            raise ValueError("controller limits must be positive")
        self.kp_xy = kp_xy
        self.deadband_m = deadband_m
        self.max_speed_mps = max_speed_mps
        self.max_accel_mps2 = max_accel_mps2
        self.max_feedforward_mps = max_feedforward_mps
        self.reset()

    def reset(self) -> None:
        self._last_timestamp_s: float | None = None
        self._last_velocity: Vector2 = (0.0, 0.0)

    def update(
        self,
        *,
        timestamp_s: float,
        vehicle_position_ned_m: Vector2,
        target_position_ned_m: Vector2,
        target_velocity_ned_mps: Vector2 = (0.0, 0.0),
        velocity_scale: float = 1.0,
    ) -> FollowCommand:
        if not 0.0 <= velocity_scale <= 1.0:
            raise ValueError("velocity_scale must be in [0, 1]")
        error = (
            target_position_ned_m[0] - vehicle_position_ned_m[0],
            target_position_ned_m[1] - vehicle_position_ned_m[1],
        )
        if _norm(error) <= self.deadband_m:
            error_for_control = (0.0, 0.0)
        else:
            error_for_control = error
        feedforward = _limit_norm(target_velocity_ned_mps, self.max_feedforward_mps)
        raw = (
            (feedforward[0] + self.kp_xy * error_for_control[0]) * velocity_scale,
            (feedforward[1] + self.kp_xy * error_for_control[1]) * velocity_scale,
        )
        speed_limited_value = _limit_norm(raw, self.max_speed_mps)
        speed_limited = speed_limited_value != raw

        acceleration_limited = False
        command_xy = speed_limited_value
        if self._last_timestamp_s is not None:
            dt = timestamp_s - self._last_timestamp_s
            if dt <= 0:
                raise ValueError("controller timestamp must advance")
            max_delta = self.max_accel_mps2 * dt
            delta = (
                speed_limited_value[0] - self._last_velocity[0],
                speed_limited_value[1] - self._last_velocity[1],
            )
            limited_delta = _limit_norm(delta, max_delta)
            acceleration_limited = limited_delta != delta
            command_xy = (
                self._last_velocity[0] + limited_delta[0],
                self._last_velocity[1] + limited_delta[1],
            )

        self._last_timestamp_s = timestamp_s
        self._last_velocity = command_xy
        return FollowCommand(
            timestamp_s=timestamp_s,
            velocity_ned_mps=(command_xy[0], command_xy[1], 0.0),
            error_xy_m=error,
            target_feedforward_xy_mps=feedforward,
            speed_limited=speed_limited,
            acceleration_limited=acceleration_limited,
        )

    def stop(self, timestamp_s: float) -> FollowCommand:
        return self.update(
            timestamp_s=timestamp_s,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(0.0, 0.0),
            target_velocity_ned_mps=(0.0, 0.0),
            velocity_scale=0.0,
        )
