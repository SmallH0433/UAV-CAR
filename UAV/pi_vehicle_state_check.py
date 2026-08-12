#!/usr/bin/env python3
"""Read several live autopilot heartbeats and report arm state only."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    states = []
    deadline = time.monotonic() + max(15.0, args.count * 3.0)
    while len(states) < args.count and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is None
            or message.get_srcSystem() != 1
            or message.get_srcComponent() != 1
            or int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            continue
        armed = bool(
            int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        state = {
            "armed": int(armed),
            "base_mode": int(message.base_mode),
            "custom_mode": int(message.custom_mode),
            "system_status": int(message.system_status),
        }
        states.append(state)
        print(f"HEARTBEAT_{len(states)}={state}")

    if len(states) < args.count:
        print(f"RESULT=INSUFFICIENT_HEARTBEATS count={len(states)}")
        return 2
    all_disarmed = all(state["armed"] == 0 for state in states)
    print(f"RESULT={'DISARMED' if all_disarmed else 'ARMED'}")
    print("READ_ONLY=1 ARM_COMMAND=0 MODE_CHANGE=0")
    return 0 if all_disarmed else 3


if __name__ == "__main__":
    raise SystemExit(main())
