#!/usr/bin/env python3
"""Non-control MAVLink uplink/downlink probe for follow telemetry streams."""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--rate", type=int, default=10)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device, baud=args.baud, autoreconnect=False,
        source_system=191, source_component=191,
    )
    heartbeat = link.wait_heartbeat(timeout=5)
    if heartbeat is None or link.target_system != 1:
        raise RuntimeError("flight-controller heartbeat not received")
    if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError("safety stop: flight controller is armed")

    link.mav.request_data_stream_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        args.rate,
        1,
    )
    counts: Counter[str] = Counter()
    armed_heartbeats = 0
    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        message = link.recv_match(blocking=True, timeout=0.25)
        if message is None or message.get_srcSystem() != 1:
            continue
        name = message.get_type()
        counts[name] += 1
        if name == "HEARTBEAT" and message.get_srcComponent() == 1:
            armed = bool(
                int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            armed_heartbeats += int(armed)
            if armed:
                raise RuntimeError("safety stop: flight controller became armed")
    link.close()

    required_seen = {
        name: counts.get(name, 0)
        for name in ("HEARTBEAT", "ATTITUDE", "RC_CHANNELS", "EKF_STATUS_REPORT")
    }
    result = {
        "scope": "real_fc_non_control_stream_request_probe",
        "request_data_stream_transmitted": 1,
        "requested_rate_hz": args.rate,
        "incoming_counts": dict(counts),
        "required_seen": required_seen,
        "armed_heartbeats": armed_heartbeats,
        "parameter_write": False,
        "mode_change": False,
        "arm_command": False,
        "motor_command": False,
        "velocity_command": False,
        "passed_link": counts.get("HEARTBEAT", 0) > 0 and counts.get("ATTITUDE", 0) > 0,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("NON_CONTROL_STREAM_REQUEST=1 PARAM_WRITE=0 MODE_CHANGE=0 ARM=0 MOTOR=0 VELOCITY=0")
    return 0 if result["passed_link"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

