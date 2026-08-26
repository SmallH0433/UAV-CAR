#!/usr/bin/env python3
"""Receive-only OV9281/Pixhawk integration preview.

The OV9281 vision service remains the sole camera owner.  This process reads
its HTTP status API and incoming Pixhawk telemetry only.  It deliberately has
no MAVLink send call and refuses to produce BODY_FRD control proposals until a
measured camera-to-body transform is enabled in the configuration.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from collections import Counter
from pathlib import Path

from pymavlink import mavutil


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "imx296_debug"))
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OV9281 receive-only follow preview")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def read_status(url: str, timeout_s: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        return json.load(response)


def is_armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def camera_optical_to_body_frd(position: tuple[float, float, float], extrinsics: dict) -> tuple[float, float, float]:
    rotation = extrinsics.get("rotation_camera_optical_to_body_frd")
    translation = extrinsics.get("translation_m")
    if not (
        isinstance(rotation, list) and len(rotation) == 3
        and all(isinstance(row, list) and len(row) == 3 for row in rotation)
        and isinstance(translation, list) and len(translation) == 3
    ):
        raise RuntimeError("invalid OV9281 camera-to-body transform")
    return tuple(
        sum(float(rotation[row][column]) * position[column] for column in range(3))
        + float(translation[row])
        for row in range(3)
    )


def validate_vision(status: dict, config: dict) -> None:
    camera = config["camera"]
    expected = {
        "sensor": "ov9281",
        "analysis_size": camera["analysis_size"],
        "pixel_source": "Y_MONO",
        "tag_family": config["target"]["family"],
        "tag_size_m": config["target"]["size_m"],
    }
    for key, value in expected.items():
        if status.get(key) != value:
            raise RuntimeError(f"Vision metadata mismatch: {key}={status.get(key)!r}, expected {value!r}")
    for key in ("calibration", "range_correction"):
        expected_name = Path(camera[key]).name
        if Path(status.get(key, "")).name != expected_name:
            raise RuntimeError(f"Vision metadata mismatch: {key}")
    if status.get("flight_controller_connected") is not False:
        raise RuntimeError("OV9281 service must not own the flight-controller link")


def main() -> int:
    args = parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    safety = config["safety"]
    extrinsics = config["camera_to_body"]
    if safety.get("mavlink_transmit") or safety.get("control_enabled"):
        raise RuntimeError("Receive-only adapter requires all transmission and control disabled")
    if not extrinsics.get("enabled"):
        raise RuntimeError("confirmed OV9281 camera-to-body transform is required")
    if extrinsics.get("approved_scope") != "disarmed_bench_and_sitl_only":
        raise RuntimeError("OV9281 transform is not approved for this disarmed bench preview")

    status_url = config["vision"]["status_url"]
    initial = read_status(status_url, 2.0)
    validate_vision(initial, config)

    serial = config["pixhawk"]["serial"]
    baud = int(config["pixhawk"]["baud"])
    link = mavutil.mavlink_connection(
        serial, baud=baud, autoreconnect=False, source_system=191, source_component=191
    )

    tracker = AlphaBetaTargetTracker(
        alpha=0.65, beta=0.08, max_residual_m=0.25,
        min_dt_s=0.02, max_dt_s=0.5, acquire_count=5,
    )
    records = []
    message_counts: Counter[str] = Counter()
    heartbeats = 0
    armed_heartbeats = 0
    api_samples = 0
    valid_vision_samples = 0
    accepted_camera_tracks = 0
    latest_body_frd = None
    latest_status = initial
    started = time.monotonic()
    next_api = started
    try:
        while time.monotonic() - started < args.duration_s:
            message = link.recv_match(blocking=True, timeout=0.05)
            if message is not None:
                message_counts[message.get_type()] += 1
                if (
                    message.get_type() == "HEARTBEAT"
                    and message.get_srcSystem() == 1
                    and message.get_srcComponent() == 1
                ):
                    heartbeats += 1
                    if is_armed(message):
                        armed_heartbeats += 1
                        raise RuntimeError("Safety stop: real flight controller is armed")

            now = time.monotonic()
            if now < next_api:
                continue
            latest_status = read_status(status_url, 1.0)
            validate_vision(latest_status, config)
            api_samples += 1
            if latest_status.get("found") and all(
                latest_status.get(name) is not None for name in ("x_m", "y_m", "z_m")
            ):
                valid_vision_samples += 1
                camera_position = (
                    float(latest_status["x_m"]),
                    float(latest_status["y_m"]),
                    float(latest_status["z_m"]),
                )
                body_position = camera_optical_to_body_frd(camera_position, extrinsics)
                latest_body_frd = body_position
                track = tracker.update(TargetMeasurement(
                    timestamp_s=now,
                    position_m=body_position,
                    decision_margin=float(latest_status.get("decision_margin", 0.0)),
                    hamming=int(latest_status.get("hamming", 0)),
                    reprojection_error_px=float(latest_status.get("reprojection_error_px", 0.0)),
                ))
                accepted_camera_tracks += int(track.accepted)

            records.append({
                "elapsed_s": now - started,
                "vision_found": bool(latest_status.get("found")),
                "camera_optical_m": [
                    latest_status.get("x_m"), latest_status.get("y_m"), latest_status.get("z_m")
                ] if latest_status.get("found") else None,
                "body_frd_m": list(latest_body_frd) if latest_status.get("found") and latest_body_frd else None,
                "proposed_velocity_mps": None,
                "control_block_reason": "REAL_CONTROL_DISABLED_PENDING_FLIGHT_READINESS",
                "mavlink_transmitted": False,
            })
            next_api = now + 1.0 / float(config["vision"]["poll_hz"])
    finally:
        link.close()

    summary = {
        "ok": api_samples > 0 and heartbeats > 0 and armed_heartbeats == 0,
        "scope": "ov9281_real_camera_real_fc_receive_only",
        "duration_s": time.monotonic() - started,
        "camera_owner": "ov9281-vision.service",
        "camera_opened_by_adapter": False,
        "vision_status_url": status_url,
        "api_samples": api_samples,
        "valid_vision_samples": valid_vision_samples,
        "accepted_camera_optical_tracks": accepted_camera_tracks,
        "analysis_fps": latest_status.get("analysis_fps"),
        "encoded_fps": latest_status.get("encoded_fps"),
        "pixhawk_heartbeats": heartbeats,
        "armed_heartbeats": armed_heartbeats,
        "incoming_message_counts": dict(message_counts),
        "body_frd_enabled": True,
        "body_frd_mapping": "(-camera_x,-camera_y,+camera_z)",
        "body_frd_translation_m": extrinsics["translation_m"],
        "body_frd_block_reason": "REAL_CONTROL_DISABLED_PENDING_FLIGHT_READINESS",
        "mavlink_packets_transmitted": 0,
        "parameter_write": False,
        "mode_change": False,
        "arm_command": False,
        "motor_command": False,
        "control_enabled": False,
        "flight_use_approved": False,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records),
            encoding="utf-8",
        )
        args.output.with_suffix(".summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("RECEIVE_ONLY=1 CAMERA_REOPENED=0 MAVLINK_TRANSMITTED=0 BODY_FRD_ENABLED=1")
    return 0 if summary["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
