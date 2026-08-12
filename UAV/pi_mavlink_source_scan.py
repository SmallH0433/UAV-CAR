#!/usr/bin/env python3
"""List MAVLink message sources seen on the Pi UART without sending commands."""

from __future__ import annotations

import argparse
import collections
import time

from pymavlink import mavutil


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
    counts = collections.Counter()
    heartbeats = {}
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_type() == "BAD_DATA":
            continue
        source = (message.get_srcSystem(), message.get_srcComponent())
        counts[(source, message.get_type())] += 1
        if message.get_type() == "HEARTBEAT":
            heartbeats[source] = {
                "type": int(message.type),
                "autopilot": int(message.autopilot),
                "base_mode": int(message.base_mode),
                "system_status": int(message.system_status),
            }

    print("HEARTBEATS")
    for source, fields in sorted(heartbeats.items()):
        print(f"  source={source[0]}/{source[1]} fields={fields}")
    print("MESSAGE_COUNTS")
    for (source, name), count in sorted(counts.items()):
        print(f"  source={source[0]}/{source[1]} type={name} count={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
