#!/usr/bin/env python3
"""Read-only GPS, optical-flow, rangefinder, battery, and EKF telemetry audit."""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter

from pymavlink import mavutil


def message_dict(message):
    data = message.to_dict()
    data.pop("mavpackettype", None)
    return data


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    interesting = {
        "HEARTBEAT",
        "GPS_RAW_INT",
        "GPS2_RAW",
        "GLOBAL_POSITION_INT",
        "LOCAL_POSITION_NED",
        "DISTANCE_SENSOR",
        "RANGEFINDER",
        "OPTICAL_FLOW",
        "OPTICAL_FLOW_RAD",
        "SYS_STATUS",
        "BATTERY_STATUS",
        "POWER_STATUS",
        "EKF_STATUS_REPORT",
    }
    counts = Counter()
    latest = {}
    heartbeats = []
    status_texts = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        name = message.get_type()
        counts[name] += 1
        if name in interesting:
            latest[name] = message_dict(message)
        if name == "HEARTBEAT" and message.get_srcComponent() == 1:
            heartbeats.append(
                {
                    "armed": bool(
                        int(message.base_mode)
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    ),
                    "custom_mode": int(message.custom_mode),
                    "system_status": int(message.system_status),
                }
            )
        elif name == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            status_texts.append(
                {"severity": int(message.severity), "text": str(text).rstrip("\x00")}
            )

    result = {
        "duration_s": args.duration,
        "counts": dict(counts),
        "latest": latest,
        "heartbeats": heartbeats,
        "statustext": status_texts,
        "read_only": True,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("READ_ONLY=1 PARAMETER_WRITE=0 ARM_COMMAND=0 MODE_CHANGE=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
