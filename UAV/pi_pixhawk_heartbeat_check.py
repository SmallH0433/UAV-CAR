#!/usr/bin/env python3
"""Read-only Pixhawk MAVLink heartbeat check for Raspberry Pi.

This script never sends parameters, arming commands, or vehicle-control
messages. It only opens the selected link and waits for a heartbeat.
"""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only Pixhawk heartbeat check")
    parser.add_argument(
        "--device",
        default="/dev/ttyACM0",
        help="serial device or MAVLink connection string",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    print(f"Opening read-only MAVLink link: {args.device} baud={args.baud}", flush=True)
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    started = time.monotonic()
    heartbeat = link.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        print(f"HEARTBEAT=NOT_RECEIVED timeout_s={args.timeout}", flush=True)
        return 2

    elapsed = time.monotonic() - started
    print(
        "HEARTBEAT=RECEIVED "
        f"system={link.target_system} component={link.target_component} "
        f"elapsed_s={elapsed:.2f}",
        flush=True,
    )
    print("READ_ONLY=1 PARAMETER_WRITE=0 ARM_COMMAND=0", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
