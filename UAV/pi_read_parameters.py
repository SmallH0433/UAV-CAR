#!/usr/bin/env python3
"""Read selected ArduPilot parameters over MAVLink without changing them."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("names", nargs="+")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=191)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    deadline = time.monotonic() + 15.0
    heartbeat = None
    while time.monotonic() < deadline:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            candidate is not None
            and candidate.get_srcSystem() == 1
            and candidate.get_srcComponent() == 1
            and int(candidate.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            heartbeat = candidate
            break
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2

    armed = bool(
        int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )
    print(f"HEARTBEAT=1/1 ARMED={int(armed)}")
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    missing = []
    for name in args.names:
        value = None
        param_type = None
        for _attempt in range(3):
            link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
            request_deadline = time.monotonic() + args.timeout
            while time.monotonic() < request_deadline:
                message = link.recv_match(
                    type="PARAM_VALUE", blocking=True, timeout=0.5
                )
                if message is not None and param_name(message) == name:
                    value = float(message.param_value)
                    param_type = int(message.param_type)
                    break
            if value is not None:
                break
        if value is None:
            print(f"PARAM {name}=NOT_RECEIVED")
            missing.append(name)
        else:
            print(f"PARAM {name}={value} TYPE={param_type}")

    print("PARAMETERS_CHANGED=0 ARM_COMMAND=0 MODE_CHANGE=0")
    return 0 if not missing else 3


if __name__ == "__main__":
    raise SystemExit(main())
