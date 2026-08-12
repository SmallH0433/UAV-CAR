#!/usr/bin/env python3
"""Verify the user-selected zero-extrinsics BODY_FRD mapping offline only."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from pymavlink.dialects.v20 import common

from mavlink_landing_target import (
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_CAMERA_OPTICAL,
    LandingTargetPacket,
    camera_optical_to_body_frd,
    pack_message,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline zero-extrinsics BODY_FRD verifier")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def decode_type(payload: bytes) -> str | None:
    parser = common.MAVLink(None)
    decoded = None
    for value in payload:
        message = parser.parse_char(bytes([value]))
        if message is not None:
            decoded = message
    return decoded.get_type() if decoded is not None else None


def main() -> None:
    args = parse_args()
    records = [json.loads(line) for line in args.input.read_text(encoding="utf-8").splitlines() if line]
    if not records:
        raise RuntimeError("Input contains no records")

    output_records = []
    source_distances = []
    body_distances = []
    source_times = []
    for index, record in enumerate(records):
        source = record["packet"]
        if int(source["frame"]) != MAV_FRAME_CAMERA_OPTICAL:
            raise RuntimeError(f"record {index}: expected CAMERA_OPTICAL frame")
        if int(source["position_valid"]) != 0:
            raise RuntimeError(f"record {index}: expected source position_valid=0")

        x_camera = float(source["x"])
        y_camera = float(source["y"])
        z_camera = float(source["z"])
        x_body, y_body, z_body = camera_optical_to_body_frd(
            x_camera,
            y_camera,
            z_camera,
            camera_yaw="nose-left",
        )
        distance = math.sqrt(x_body * x_body + y_body * y_body + z_body * z_body)
        packet = LandingTargetPacket(
            time_usec=int(source["time_usec"]),
            target_num=int(source["target_num"]),
            frame=MAV_FRAME_BODY_FRD,
            angle_x=math.atan2(x_body, z_body),
            angle_y=math.atan2(y_body, z_body),
            distance=distance,
            size_x=float(source["size_x"]),
            size_y=float(source["size_y"]),
            x=x_body,
            y=y_body,
            z=z_body,
            q=tuple(float(value) for value in source["q"]),
            type=int(source["type"]),
            position_valid=1,
        )
        payload = pack_message(packet)
        if decode_type(payload) != "LANDING_TARGET":
            raise RuntimeError(f"record {index}: packed message did not decode")
        if not math.isclose(distance, float(source["distance"]), rel_tol=0.0, abs_tol=1e-9):
            raise RuntimeError(f"record {index}: rigid transform changed distance")
        if not (
            math.isclose(x_body, -x_camera, abs_tol=1e-12)
            and math.isclose(y_body, -y_camera, abs_tol=1e-12)
            and math.isclose(z_body, z_camera, abs_tol=1e-12)
        ):
            raise RuntimeError(f"record {index}: coordinate sign invariant failed")

        source_distances.append(float(source["distance"]))
        body_distances.append(distance)
        source_times.append(int(source["time_usec"]))
        output_records.append(
            {
                "verification_scope": "offline_sitl_only",
                "extrinsics_source": "user_zero_assumption",
                "translation_body_frd_m": [0.0, 0.0, 0.0],
                "mount_roll_pitch_deg": [0.0, 0.0],
                "mapping": "(-x_camera,-y_camera,+z_camera)",
                "message": "LANDING_TARGET",
                "packet": packet.as_dict(),
                "mavlink_v2_hex": payload.hex(),
            }
        )

    args.output.write_text(
        "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in output_records),
        encoding="utf-8",
    )
    elapsed_s = max(0.0, (max(source_times) - min(source_times)) / 1_000_000.0)
    effective_hz = 0.0 if elapsed_s == 0.0 else (len(source_times) - 1) / elapsed_s
    print(f"records={len(output_records)}")
    print(f"frame={MAV_FRAME_BODY_FRD} position_valid=1")
    print(f"mapping=(-x,-y,+z) translation_m=(0,0,0) roll_pitch_deg=(0,0)")
    print(f"distance_mean_m={sum(body_distances) / len(body_distances):.6f}")
    print(f"distance_invariance_max_error_m={max(abs(a-b) for a,b in zip(source_distances, body_distances)):.12f}")
    print(f"source_effective_rate_hz={effective_hz:.2f}")
    print(f"output={args.output}")
    print("safety=offline_only_no_serial_no_flight_controller")


if __name__ == "__main__":
    main()
