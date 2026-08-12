#!/usr/bin/env python3
"""Read a Pixhawk parameter list over USB without changing any values."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil
from serial import SerialException


def param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=190,
    )
    heartbeat = None
    heartbeat_deadline = time.monotonic() + 15.0
    while time.monotonic() < heartbeat_deadline:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            candidate is not None
            and int(candidate.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            heartbeat = candidate
            break
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2
    print(f"HEARTBEAT_SOURCE={heartbeat.get_srcSystem()}/{heartbeat.get_srcComponent()}")
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    link.mav.param_request_list_send(1, 1)

    values = {}
    expected = None
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        try:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        except SerialException as error:
            print(f"SERIAL_READ_ERROR={error}")
            break
        if message is None:
            continue
        name = param_name(message)
        values[name] = float(message.param_value)
        expected = int(message.param_count)
        if expected > 0 and len(values) >= expected:
            break

    print(f"PARAMETERS_RECEIVED={len(values)} EXPECTED={expected}")
    for name in sorted(values):
        if name.startswith(("SR", "MAV", "SERIAL", "SYSID")):
            print(f"PARAM {name}={values[name]}")
    print("PARAMETERS_CHANGED=0")
    return 0 if values else 3


if __name__ == "__main__":
    raise SystemExit(main())
