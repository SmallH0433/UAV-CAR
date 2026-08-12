#!/usr/bin/env python3
"""Read-only filtered ArduPilot sensor parameter audit over Raspberry Pi UART."""

from __future__ import annotations

import argparse
import json
import time

from pymavlink import mavutil


PREFIXES = (
    "SERIAL2_",
    "FLOW_",
    "RNGFND1_",
    "BATT_",
    "BATT1_",
    "GPS1_",
    "EK3_SRC1_",
    "EK3_SRC_OPTIONS",
    "AHRS_EKF_TYPE",
    "EK3_ENABLE",
)


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=35.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    heartbeats = []
    deadline = time.monotonic() + 15.0
    while len(heartbeats) < 3 and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is None
            or message.get_srcSystem() != 1
            or message.get_srcComponent() != 1
        ):
            continue
        armed = bool(
            int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        heartbeats.append(armed)

    if len(heartbeats) < 3 or any(heartbeats):
        print(
            json.dumps(
                {
                    "error": "DISARMED_HEARTBEAT_GATE_FAILED",
                    "heartbeats": heartbeats,
                    "read_only": True,
                },
                indent=2,
            )
        )
        return 2

    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    link.mav.param_request_list_send(1, 1)

    all_names = set()
    selected = {}
    expected = None
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        name = parameter_name(message)
        all_names.add(name)
        expected = int(message.param_count)
        if name.startswith(PREFIXES):
            selected[name] = {
                "value": float(message.param_value),
                "type": int(message.param_type),
            }
        if expected > 0 and len(all_names) >= expected:
            break

    result = {
        "device": args.device,
        "baud": args.baud,
        "disarmed_heartbeats": len(heartbeats),
        "parameters_received": len(all_names),
        "parameters_expected": expected,
        "selected": dict(sorted(selected.items())),
        "read_only": True,
        "parameter_write": False,
        "arm_command": False,
        "mode_change": False,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if selected else 3


if __name__ == "__main__":
    raise SystemExit(main())
