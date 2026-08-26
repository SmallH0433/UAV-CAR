"""OV9281 status API to quality-gated MAVLink LANDING_TARGET bridge.

The executable is dry-run by default.  It does not open a MAVLink endpoint
unless both the command line and a commissioned configuration explicitly allow
transmission.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, TextIO

from .math3d import Matrix3, clamp, finite_vector, norm, transform_point, validate_rotation
from .models import (
    BridgeResult,
    LandingTargetObservation,
    LandingTargetPacket,
    Vector3,
)


MAV_FRAME_BODY_FRD = 12
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2


@dataclass(frozen=True)
class TargetQualityGate:
    minimum_decision_margin: float
    maximum_hamming: int
    maximum_reprojection_error_px: float

    def validate(self) -> None:
        if not math.isfinite(self.minimum_decision_margin) or self.minimum_decision_margin < 0.0:
            raise ValueError("minimum decision margin must be finite and non-negative")
        if self.maximum_hamming < 0:
            raise ValueError("maximum hamming must be non-negative")
        if (
            not math.isfinite(self.maximum_reprojection_error_px)
            or self.maximum_reprojection_error_px <= 0.0
        ):
            raise ValueError("maximum reprojection error must be finite and positive")


@dataclass(frozen=True)
class BridgeConfig:
    sensor: str
    tag_family: str
    target_sizes_m: dict[int, float]
    analysis_size: Optional[tuple[int, int]]
    output_rate_hz: float
    max_message_age_s: float
    loss_timeout_s: float
    minimum_decision_margin: float
    maximum_hamming: int
    maximum_reprojection_error_px: float
    target_quality_gates: dict[int, TargetQualityGate]
    minimum_distance_m: float
    maximum_distance_m: float
    distance_consistency_m: float
    rotation_camera_to_body: Matrix3
    translation_camera_in_body_m: Vector3

    @classmethod
    def from_mapping(cls, root: Mapping[str, Any]) -> "BridgeConfig":
        vision = root.get("vision", {})
        bridge = root.get("landing_target_bridge", {})
        quality = bridge.get("quality_gate", {})
        extrinsics = bridge.get("camera_to_body", {})
        targets = bridge.get("targets", [])
        default_minimum_margin = float(quality.get("minimum_decision_margin", 30.0))
        default_maximum_hamming = int(quality.get("maximum_hamming", 0))
        default_maximum_reprojection = float(
            quality.get("maximum_reprojection_error_px", 2.0)
        )
        target_sizes: dict[int, float] = {}
        target_quality_gates: dict[int, TargetQualityGate] = {}
        for target in targets:
            if not bool(target.get("enabled", True)):
                continue
            tag_id = int(target["id"])
            if tag_id in target_sizes:
                raise ValueError(f"duplicate landing target ID {tag_id}")
            target_sizes[tag_id] = float(target["size_m"])
            override = target.get("quality_gate", {})
            gate = TargetQualityGate(
                minimum_decision_margin=float(
                    override.get("minimum_decision_margin", default_minimum_margin)
                ),
                maximum_hamming=int(
                    override.get("maximum_hamming", default_maximum_hamming)
                ),
                maximum_reprojection_error_px=float(
                    override.get(
                        "maximum_reprojection_error_px",
                        default_maximum_reprojection,
                    )
                ),
            )
            gate.validate()
            target_quality_gates[tag_id] = gate
        if not target_sizes or any(size <= 0.0 for size in target_sizes.values()):
            raise ValueError("at least one positive landing target size is required")
        output_rate_hz = float(bridge.get("output_rate_hz", 10.0))
        if not 1.0 <= output_rate_hz <= 50.0:
            raise ValueError("output_rate_hz must be in [1, 50]")
        rotation = validate_rotation(extrinsics["rotation_camera_optical_to_body_frd"])
        translation = finite_vector(extrinsics.get("translation_m", (0.0, 0.0, 0.0)))
        analysis_size_value = vision.get("analysis_size")
        analysis_size = None
        if analysis_size_value is not None:
            if len(analysis_size_value) != 2:
                raise ValueError("vision.analysis_size must contain width and height")
            analysis_size = (int(analysis_size_value[0]), int(analysis_size_value[1]))
        return cls(
            sensor=str(vision.get("sensor", "ov9281")).lower(),
            tag_family=str(bridge.get("tag_family", "tag36h11")),
            target_sizes_m=target_sizes,
            analysis_size=analysis_size,
            output_rate_hz=output_rate_hz,
            max_message_age_s=float(quality.get("max_message_age_ms", 150.0)) / 1000.0,
            loss_timeout_s=float(quality.get("loss_timeout_ms", 500.0)) / 1000.0,
            minimum_decision_margin=default_minimum_margin,
            maximum_hamming=default_maximum_hamming,
            maximum_reprojection_error_px=default_maximum_reprojection,
            target_quality_gates=target_quality_gates,
            minimum_distance_m=float(quality.get("minimum_distance_m", 0.03)),
            maximum_distance_m=float(quality.get("maximum_distance_m", 8.0)),
            distance_consistency_m=float(quality.get("distance_consistency_m", 0.05)),
            rotation_camera_to_body=rotation,
            translation_camera_in_body_m=translation,
        )


class LandingTargetBridge:
    """Convert unique OV9281 frames while rejecting stale or weak detections."""

    def __init__(self, config: BridgeConfig) -> None:
        self.config = config
        self._last_sequence: Optional[int] = None
        self._last_observation_time_s: Optional[float] = None
        self._last_packet_time_s: Optional[float] = None

    def target_lost(self, now_s: float) -> bool:
        return (
            self._last_observation_time_s is None
            or float(now_s) - self._last_observation_time_s > self.config.loss_timeout_s
        )

    def process_status(
        self,
        status: Mapping[str, Any],
        *,
        received_time_s: float,
        wall_time_usec: int,
    ) -> BridgeResult:
        now_s = float(received_time_s)
        if not math.isfinite(now_s):
            return self._reject("NON_FINITE_RECEIVE_TIME", 0.0)

        sequence_value = status.get("analysis_sequence")
        sequence = None if sequence_value is None else int(sequence_value)
        if sequence is not None and sequence == self._last_sequence:
            return self._reject("DUPLICATE_FRAME", now_s)
        if sequence is not None:
            self._last_sequence = sequence

        metadata_reason = self._validate_metadata(status)
        if metadata_reason is not None:
            return self._reject(metadata_reason, now_s)
        if not bool(status.get("found", False)):
            return self._reject("TARGET_NOT_FOUND", now_s)

        try:
            tag_id = int(status["tag_id"])
        except KeyError:
            return self._reject("MISSING_FIELD:tag_id", now_s)
        except (TypeError, ValueError) as exc:
            return self._reject(f"INVALID_FIELD:{exc}", now_s)
        if tag_id not in self.config.target_sizes_m:
            return self._reject("TAG_ID_NOT_CONFIGURED", now_s)
        tag_size_m = self.config.target_sizes_m[tag_id]
        quality_gate = self.config.target_quality_gates[tag_id]

        try:
            reported_size = float(status["tag_size_m"])
            frame_age_s = float(status["frame_age_ms"]) / 1000.0
            camera_position = finite_vector(
                (status["x_m"], status["y_m"], status["z_m"])
            )
            reported_distance = float(status["distance_m"])
            decision_margin = float(status["decision_margin"])
            hamming = int(status.get("hamming", 0))
            reprojection_error = float(status["reprojection_error_px"])
        except KeyError as exc:
            return self._reject(f"MISSING_FIELD:{exc.args[0]}", now_s)
        except (TypeError, ValueError) as exc:
            return self._reject(f"INVALID_FIELD:{exc}", now_s)

        if abs(reported_size - tag_size_m) > 1.0e-6:
            return self._reject("TAG_SIZE_MISMATCH", now_s)
        if frame_age_s < 0.0 or frame_age_s > self.config.max_message_age_s:
            return self._reject("STALE_FRAME", now_s)
        if decision_margin < quality_gate.minimum_decision_margin:
            return self._reject("LOW_DECISION_MARGIN", now_s)
        if hamming > quality_gate.maximum_hamming:
            return self._reject("HAMMING_LIMIT", now_s)
        if (
            not math.isfinite(reprojection_error)
            or reprojection_error > quality_gate.maximum_reprojection_error_px
        ):
            return self._reject("REPROJECTION_ERROR_LIMIT", now_s)
        if camera_position[2] <= 0.0:
            return self._reject("TARGET_BEHIND_CAMERA", now_s)

        calculated_distance = norm(camera_position)
        if not math.isfinite(reported_distance) or reported_distance <= 0.0:
            return self._reject("INVALID_DISTANCE", now_s)
        allowed_distance_error = max(
            self.config.distance_consistency_m,
            0.05 * reported_distance,
        )
        if abs(calculated_distance - reported_distance) > allowed_distance_error:
            return self._reject("DISTANCE_INCONSISTENT", now_s)

        body_position = transform_point(
            self.config.rotation_camera_to_body,
            self.config.translation_camera_in_body_m,
            camera_position,
        )
        body_distance = norm(body_position)
        if body_position[2] <= 0.0:
            return self._reject("TARGET_NOT_BELOW_AIRCRAFT", now_s)
        if not self.config.minimum_distance_m <= body_distance <= self.config.maximum_distance_m:
            return self._reject("DISTANCE_OUT_OF_RANGE", now_s)

        quality = self._quality(
            decision_margin,
            reprojection_error,
            frame_age_s,
            quality_gate,
        )
        covariance = self._covariance(body_distance, reprojection_error)
        capture_time_s = now_s - frame_age_s
        capture_wall_usec = max(0, int(wall_time_usec) - int(frame_age_s * 1_000_000.0))
        observation = LandingTargetObservation(
            capture_time_s=capture_time_s,
            received_time_s=now_s,
            wall_time_usec=capture_wall_usec,
            tag_id=tag_id,
            tag_size_m=tag_size_m,
            position_body_frd_m=body_position,
            distance_m=body_distance,
            decision_margin=decision_margin,
            hamming=hamming,
            reprojection_error_px=reprojection_error,
            quality=quality,
            covariance_m2=covariance,
            source_sequence=sequence,
        )
        self._last_observation_time_s = now_s

        minimum_period_s = 1.0 / self.config.output_rate_hz
        packet = None
        reason = "ACCEPTED_RATE_LIMITED"
        if self._last_packet_time_s is None or now_s - self._last_packet_time_s >= minimum_period_s - 1.0e-6:
            packet = self._make_packet(observation)
            self._last_packet_time_s = now_s
            reason = "ACCEPTED_PACKET_READY"
        return BridgeResult(observation, packet, reason, self.target_lost(now_s))

    def _validate_metadata(self, status: Mapping[str, Any]) -> Optional[str]:
        if str(status.get("sensor", "")).lower() != self.config.sensor:
            return "SENSOR_MISMATCH"
        if str(status.get("mode", "")) != "apriltag":
            return "VISION_NOT_IN_APRILTAG_MODE"
        if str(status.get("tag_family", "")) != self.config.tag_family:
            return "TAG_FAMILY_MISMATCH"
        if status.get("flight_controller_connected") is not False:
            return "CAMERA_SERVICE_MUST_NOT_OWN_MAVLINK"
        if self.config.analysis_size is not None:
            try:
                reported_size = tuple(int(value) for value in status.get("analysis_size", ()))
            except (TypeError, ValueError):
                return "ANALYSIS_SIZE_MISMATCH"
            if reported_size != self.config.analysis_size:
                return "ANALYSIS_SIZE_MISMATCH"
        return None

    def _quality(
        self,
        margin: float,
        reprojection_error: float,
        age_s: float,
        quality_gate: TargetQualityGate,
    ) -> float:
        margin_scale = max(quality_gate.minimum_decision_margin * 2.0, 1.0)
        margin_score = clamp(margin / margin_scale, 0.0, 1.0)
        error_score = clamp(
            1.0
            - reprojection_error
            / max(quality_gate.maximum_reprojection_error_px, 1.0e-6),
            0.0,
            1.0,
        )
        age_score = clamp(
            1.0 - age_s / max(self.config.max_message_age_s, 1.0e-6),
            0.0,
            1.0,
        )
        return clamp(0.50 * margin_score + 0.30 * error_score + 0.20 * age_score, 0.0, 1.0)

    @staticmethod
    def _covariance(distance_m: float, reprojection_error_px: float) -> tuple[float, ...]:
        sigma_xy = max(0.005, 0.006 * distance_m + 0.002 * reprojection_error_px)
        sigma_z = max(0.010, 0.015 * distance_m + 0.004 * reprojection_error_px)
        return (
            sigma_xy * sigma_xy, 0.0, 0.0,
            0.0, sigma_xy * sigma_xy, 0.0,
            0.0, 0.0, sigma_z * sigma_z,
        )

    @staticmethod
    def _make_packet(observation: LandingTargetObservation) -> LandingTargetPacket:
        x, y, z = observation.position_body_frd_m
        # These angles are consistent with ArduPilot's angle-only fallback:
        # vector_body = (-tan(angle_y), tan(angle_x), 1).
        angle_x = math.atan2(y, z)
        angle_y = math.atan2(-x, z)
        angular_size = 2.0 * math.atan2(observation.tag_size_m * 0.5, observation.distance_m)
        return LandingTargetPacket(
            time_usec=observation.wall_time_usec,
            target_num=observation.tag_id,
            frame=MAV_FRAME_BODY_FRD,
            angle_x=angle_x,
            angle_y=angle_y,
            distance=observation.distance_m,
            size_x=angular_size,
            size_y=angular_size,
            x=x,
            y=y,
            z=z,
            q=(1.0, 0.0, 0.0, 0.0),
            type=LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            position_valid=1,
        )

    def _reject(self, reason: str, now_s: float) -> BridgeResult:
        return BridgeResult(None, None, reason, self.target_lost(now_s))


class MavlinkLandingTargetSender:
    """Optional transport opened only after explicit safety authorization."""

    def __init__(self, config: Mapping[str, Any]) -> None:
        try:
            from pymavlink import mavutil
        except ImportError as exc:  # pragma: no cover - target host dependency
            raise RuntimeError("pymavlink is required only for --transmit") from exc
        mavlink = config["mavlink"]
        self._link = mavutil.mavlink_connection(
            str(mavlink["endpoint"]),
            baud=int(mavlink.get("baud", 115200)),
            source_system=int(mavlink.get("source_system", 191)),
            source_component=int(mavlink.get("source_component", 191)),
            autoreconnect=False,
            force_connected=True,
        )

    def send(self, packet: LandingTargetPacket) -> None:
        self._link.mav.landing_target_send(
            packet.time_usec,
            packet.target_num,
            packet.frame,
            packet.angle_x,
            packet.angle_y,
            packet.distance,
            packet.size_x,
            packet.size_y,
            packet.x,
            packet.y,
            packet.z,
            packet.q,
            packet.type,
            packet.position_valid,
        )

    def close(self) -> None:
        self._link.close()


def _transmit_authorized(config: Mapping[str, Any]) -> bool:
    safety = config.get("safety", {})
    extrinsics = config.get("landing_target_bridge", {}).get("camera_to_body", {})
    return bool(
        safety.get("mavlink_transmit", False)
        and safety.get("flight_use_approved", False)
        and extrinsics.get("flight_use_approved", False)
    )


def _read_status(url: str, timeout_s: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        return json.load(response)


def _write_record(output: Optional[TextIO], record: Mapping[str, Any]) -> None:
    line = json.dumps(record, ensure_ascii=False, sort_keys=True)
    print(line, flush=True)
    if output is not None:
        output.write(line + "\n")
        output.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="OV9281 LANDING_TARGET bridge (dry-run by default)")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=10.0, help="0 runs until interrupted")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--transmit", action="store_true", help="requires three config approvals")
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    bridge = LandingTargetBridge(BridgeConfig.from_mapping(config))
    if args.transmit and not _transmit_authorized(config):
        raise RuntimeError("MAVLink transmission is not commissioned by this configuration")
    sender = MavlinkLandingTargetSender(config) if args.transmit else None
    output = None
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output = args.output.open("w", encoding="utf-8")

    vision = config["vision"]
    poll_period_s = 1.0 / float(vision.get("poll_hz", 10.0))
    started = time.monotonic()
    next_poll = started
    try:
        while args.duration_s <= 0.0 or time.monotonic() - started < args.duration_s:
            now_s = time.monotonic()
            if now_s < next_poll:
                time.sleep(min(next_poll - now_s, 0.05))
                continue
            try:
                status = _read_status(str(vision["status_url"]), float(vision.get("timeout_s", 1.0)))
                result = bridge.process_status(
                    status,
                    received_time_s=now_s,
                    wall_time_usec=time.time_ns() // 1_000,
                )
                transmitted = False
                if result.packet is not None and sender is not None:
                    sender.send(result.packet)
                    transmitted = True
                _write_record(output, {
                    "elapsed_s": now_s - started,
                    "accepted": result.accepted,
                    "reason": result.reason,
                    "target_lost": result.target_lost,
                    "observation": None if result.observation is None else result.observation.as_dict(),
                    "packet": None if result.packet is None else result.packet.as_dict(),
                    "mavlink_transmitted": transmitted,
                })
            except Exception as exc:  # fail closed and keep the monitor observable
                _write_record(output, {
                    "elapsed_s": now_s - started,
                    "accepted": False,
                    "reason": f"STATUS_READ_ERROR:{type(exc).__name__}:{exc}",
                    "target_lost": bridge.target_lost(now_s),
                    "mavlink_transmitted": False,
                })
            next_poll = now_s + poll_period_s
    except KeyboardInterrupt:
        pass
    finally:
        if sender is not None:
            sender.close()
        if output is not None:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
