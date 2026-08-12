#!/usr/bin/env python3
"""Replay verified BODY_FRD LANDING_TARGET records into local ArduPilot SITL."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import common


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SITL-only BODY_FRD replay")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    return parser.parse_args()


def read_param(connection, name: str) -> float | None:
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        response = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if response is None:
            continue
        raw_name = response.param_id
        response_name = (
            raw_name.decode(errors="ignore") if isinstance(raw_name, bytes) else str(raw_name)
        ).rstrip("\x00")
        if response_name == name:
            return float(response.param_value)
    return None


def main() -> None:
    args = parse_args()
    if args.rate_hz < 10.0:
        raise ValueError("SITL replay rate must be at least 10 Hz")
    records = [json.loads(line) for line in args.input.read_text(encoding="utf-8").splitlines() if line]
    if not records:
        raise RuntimeError("Replay input contains no records")
    for index, record in enumerate(records):
        packet = record["packet"]
        if int(packet["frame"]) != mavutil.mavlink.MAV_FRAME_BODY_FRD:
            raise RuntimeError(f"record {index}: frame is not BODY_FRD")
        if int(packet["position_valid"]) != 1:
            raise RuntimeError(f"record {index}: position_valid is not 1")
        if record.get("verification_scope") != "offline_sitl_only":
            raise RuntimeError(f"record {index}: missing offline-only scope marker")

    connection = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5760", source_system=250, dialect="common"
    )
    heartbeat = connection.wait_heartbeat(timeout=15)
    if heartbeat is None:
        print("SITL_HEARTBEAT=NOT_RECEIVED")
        sys.exit(1)
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("SITL_SAFETY_STOP=ARMED")
        sys.exit(2)
    print(
        "SITL_HEARTBEAT=RECEIVED "
        f"system={connection.target_system} component={connection.target_component} armed=0"
    )

    for name, value in (("PLND_ENABLED", 1), ("PLND_TYPE", 1)):
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
    for name in ("PLND_ENABLED", "PLND_TYPE"):
        value = read_param(connection, name)
        if value is None or abs(value - 1.0) > 0.01:
            raise RuntimeError(f"SITL parameter readback failed: {name}={value}")
        print(f"SITL_PARAM_OK {name}=1")

    interval = 1.0 / args.rate_hz
    started = time.monotonic()
    next_send = started
    sent = 0
    for record in records:
        now = time.monotonic()
        if now < next_send:
            time.sleep(next_send - now)
        packet = record["packet"]
        message = common.MAVLink_landing_target_message(
            time_usec=time.time_ns() // 1000,
            target_num=int(packet["target_num"]),
            frame=int(packet["frame"]),
            angle_x=float(packet["angle_x"]),
            angle_y=float(packet["angle_y"]),
            distance=float(packet["distance"]),
            size_x=float(packet["size_x"]),
            size_y=float(packet["size_y"]),
            x=float(packet["x"]),
            y=float(packet["y"]),
            z=float(packet["z"]),
            q=tuple(float(value) for value in packet["q"]),
            type=int(packet["type"]),
            position_valid=1,
        )
        connection.mav.send(message)
        sent += 1
        next_send += interval
        incoming = connection.recv_match(type="HEARTBEAT", blocking=False)
        if incoming is not None and (
            incoming.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            raise RuntimeError("SITL became armed during replay")

    elapsed = time.monotonic() - started
    print(f"BODY_FRD_RECORDS_SENT={sent}")
    print(f"REQUESTED_RATE_HZ={args.rate_hz:.2f} EFFECTIVE_RATE_HZ={sent / elapsed:.2f}")
    print("FRAME=BODY_FRD POSITION_VALID=1")
    print("EXTRINSICS=user_zero_assumption")
    print("SIMULATION_ONLY=1 ARMED=0")


if __name__ == "__main__":
    main()
