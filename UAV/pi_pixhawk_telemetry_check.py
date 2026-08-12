#!/usr/bin/env python3
"""Read-only telemetry quality check for a Pixhawk serial link."""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only Pixhawk telemetry check")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2

    counts: Counter[str] = Counter()
    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        message = link.recv_match(blocking=True, timeout=1)
        if message is not None:
            counts[message.get_type()] += 1

    print(
        f"HEARTBEAT=RECEIVED system={link.target_system} "
        f"component={link.target_component}"
    )
    print(f"DURATION_S={args.duration:.1f}")
    for name in ("HEARTBEAT", "ATTITUDE", "SYS_STATUS", "RC_CHANNELS", "STATUSTEXT"):
        print(f"{name}={counts.get(name, 0)}")
    print(f"TOTAL_MESSAGES={sum(counts.values())}")
    print("READ_ONLY=1 PARAMETER_WRITE=0 ARM_COMMAND=0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
