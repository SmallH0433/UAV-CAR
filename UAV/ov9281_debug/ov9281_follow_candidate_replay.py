#!/usr/bin/env python3
"""Replay real OV9281 API samples through the follow math without transmitting.

Camera optical X/Y are deliberately kept as an unapproved proxy plane.  This
checks tracker, controller, speed limiting and MAVLink v2 encoding only; it is
not a camera-to-body transform and the packed packets are never sent.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "imx296_debug"))
from follow_controller import HorizontalFollowController  # noqa: E402
from mavlink_guided_velocity import GuidedVelocitySetpoint, pack_message  # noqa: E402
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    controller_config = config["controller_candidate"]
    tracker_config = config["tracker"]
    if not config["camera_to_body"]["enabled"]:
        raise RuntimeError("this replay requires the confirmed BODY_FRD transform")
    if config["safety"]["mavlink_transmit"] or config["safety"]["control_enabled"]:
        raise RuntimeError("this replay requires transmission and control disabled")

    rotation = config["camera_to_body"]["rotation_camera_optical_to_body_frd"]
    translation = config["camera_to_body"]["translation_m"]
    if rotation != [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]:
        raise RuntimeError("unexpected OV9281 orientation matrix")

    tracker = AlphaBetaTargetTracker(
        alpha=float(tracker_config["alpha"]),
        beta=float(tracker_config["beta"]),
        max_residual_m=float(tracker_config["max_residual_m"]),
        min_dt_s=0.02,
        max_dt_s=0.5,
        acquire_count=int(tracker_config["acquire_count"]),
    )
    controller = HorizontalFollowController(
        kp_xy=float(controller_config["kp_xy"]),
        deadband_m=float(controller_config["deadband_m"]),
        max_speed_mps=float(controller_config["max_speed_mps"]),
        max_accel_mps2=float(controller_config["max_accel_mps2"]),
        max_feedforward_mps=float(controller_config["max_feedforward_mps"]),
    )

    accepted = acquired = encoded = 0
    max_speed = 0.0
    last_track = None
    output_records = []
    for line in args.input.read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        optical = record.get("camera_optical_m")
        if not record.get("vision_found") or optical is None:
            continue
        timestamp_s = float(record["elapsed_s"])
        camera = tuple(map(float, optical))
        orientation_only_body_frd = tuple(
            sum(float(rotation[row][column]) * camera[column] for column in range(3))
            + float(translation[row])
            for row in range(3)
        )
        track = tracker.update(TargetMeasurement(timestamp_s, orientation_only_body_frd))
        accepted += int(track.accepted)
        if not track.accepted or not track.acquired:
            continue
        acquired += 1
        command = controller.update(
            timestamp_s=timestamp_s,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(track.position_m[0], track.position_m[1]),
            target_velocity_ned_mps=(track.velocity_mps[0], track.velocity_mps[1]),
        )
        speed = math.hypot(command.velocity_ned_mps[0], command.velocity_ned_mps[1])
        max_speed = max(max_speed, speed)
        setpoint = GuidedVelocitySetpoint(
            time_boot_ms=int(timestamp_s * 1000),
            vx_mps=command.velocity_ned_mps[0],
            vy_mps=command.velocity_ned_mps[1],
        )
        packet = pack_message(
            setpoint, max_speed_mps=float(controller_config["max_speed_mps"])
        )
        encoded += 1
        last_track = track
        output_records.append({
            "elapsed_s": timestamp_s,
            "orientation_only_body_frd_m": list(track.position_m),
            "proxy_velocity_mps": list(command.velocity_ned_mps),
            "speed_mps": speed,
            "mavlink_v2_packet_bytes": len(packet),
            "mavlink_transmitted": False,
            "body_frd_rotation_valid": True,
            "body_frd_translation_valid": False,
        })

    speed_limit = float(controller_config["max_speed_mps"])
    summary = {
        "scope": "real_ov9281_samples_offline_math_and_encoding_only",
        "body_frd_rotation_valid": True,
        "body_frd_translation_valid": False,
        "camera_installation_required_before_real_control": True,
        "accepted_measurements": accepted,
        "acquired_control_samples": acquired,
        "encoded_packets_not_sent": encoded,
        "max_computed_speed_mps": max_speed,
        "speed_limit_mps": speed_limit,
        "last_track_m": list(last_track.position_m) if last_track else None,
        "mavlink_packets_transmitted": 0,
        "passed": encoded > 0 and max_speed <= speed_limit + 1e-9,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in output_records),
        encoding="utf-8",
    )
    args.output.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("OFFLINE_REPLAY=1 BODY_FRD_VALID=0 MAVLINK_TRANSMITTED=0")
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
