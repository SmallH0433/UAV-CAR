#!/usr/bin/env python3
"""Dry-run/loopback LANDING_TARGET bridge for observer CSV logs.

Default behavior writes decoded packet metadata and raw MAVLink bytes to JSONL.
UDP is opt-in and is restricted to localhost unless explicitly overridden.
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
from pathlib import Path
from types import SimpleNamespace

from mavlink_landing_target import (
    MAV_FRAME_BODY_FRD,
    MAV_FRAME_CAMERA_OPTICAL,
    load_body_extrinsics,
    observation_to_packet,
    pack_message,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LANDING_TARGET dry-run bridge")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--frame", choices=("camera-optical", "body-frd"), default="camera-optical")
    parser.add_argument("--camera-yaw", choices=("nose-left",), help="required for body-frd conversion")
    parser.add_argument("--extrinsics", type=Path, help="camera-to-body JSON; required for body-frd")
    parser.add_argument("--position-valid", action="store_true")
    parser.add_argument("--udp", help="optional host:port; localhost only by default")
    parser.add_argument("--allow-nonloopback", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


def row_to_observation(row: dict[str, str]):
    def optional_float(name):
        value = row.get(name, "")
        return None if value in ("", "None") else float(value)

    def optional_int(name):
        value = row.get(name, "")
        return None if value in ("", "None") else int(float(value))

    return SimpleNamespace(
        valid=row.get("valid", "False").lower() == "true",
        x_m=optional_float("x_m"),
        y_m=optional_float("y_m"),
        z_m=optional_float("z_m"),
        distance_m=optional_float("distance_m"),
        tag_id=int(row.get("tag_id", 0)),
        decision_margin=optional_float("decision_margin"),
        hamming=optional_int("hamming"),
    )


def main() -> None:
    args = parse_args()
    if not args.csv.exists():
        raise FileNotFoundError(args.csv)
    frame = MAV_FRAME_BODY_FRD if args.frame == "body-frd" else MAV_FRAME_CAMERA_OPTICAL
    if args.frame == "body-frd" and not args.position_valid:
        raise ValueError("body-frd requires --position-valid after camera extrinsics are calibrated")
    extrinsics = None
    if args.frame == "body-frd":
        if args.extrinsics is None:
            raise ValueError("body-frd requires --extrinsics")
        extrinsics = load_body_extrinsics(args.extrinsics)

    output = args.output or args.csv.with_name("landing_target_dry_run.jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    udp_socket = None
    udp_addr = None
    if args.udp:
        host, port_text = args.udp.rsplit(":", 1)
        port = int(port_text)
        if host not in {"127.0.0.1", "localhost", "::1"} and not args.allow_nonloopback:
            raise ValueError("UDP is restricted to localhost; use --allow-nonloopback explicitly to override")
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_addr = (host, port)

    processed = valid = sent = 0
    try:
        with args.csv.open("r", newline="", encoding="utf-8") as csv_file, output.open(
            "w", encoding="utf-8"
        ) as jsonl:
            for row in csv.DictReader(csv_file):
                if args.limit and processed >= args.limit:
                    break
                processed += 1
                observation = row_to_observation(row)
                if args.frame == "body-frd":
                    observation.x_m, observation.y_m, observation.z_m = extrinsics.transform(
                        observation.x_m,
                        observation.y_m,
                        observation.z_m,
                    )
                packet = observation_to_packet(
                    observation,
                    target_num=0,
                    frame=frame,
                    position_valid=1 if args.position_valid else 0,
                )
                if packet is None:
                    continue
                valid += 1
                payload = pack_message(packet)
                if udp_socket and udp_addr:
                    udp_socket.sendto(payload, udp_addr)
                    sent += 1
                record = {
                    "message": "LANDING_TARGET",
                    "frame": args.frame,
                    "position_valid": bool(args.position_valid),
                    "extrinsics": str(args.extrinsics) if args.extrinsics else None,
                    "packet": packet.as_dict(),
                    "mavlink_v2_hex": payload.hex(),
                    "payload_bytes": len(payload),
                    "udp_sent": bool(udp_socket),
                }
                jsonl.write(json.dumps(record, ensure_ascii=False) + "\n")
    finally:
        if udp_socket:
            udp_socket.close()

    mode = f"UDP {args.udp}" if args.udp else "dry-run only"
    print(f"processed_rows={processed} valid_packets={valid} udp_sent={sent}")
    print(f"mode={mode}")
    print(f"output={output}")
    print("Safety: no flight-controller serial link or flight command was opened.")


if __name__ == "__main__":
    main()
