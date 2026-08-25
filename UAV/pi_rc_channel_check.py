#!/usr/bin/env python3
"""Read-only disarmed gate for companion-controlled RC7 and RC8."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--low-maximum", type=int, default=1200)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    disarmed_heartbeats = 0
    samples: list[tuple[int, int]] = []
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "HEARTBEAT" and message.get_srcComponent() == 1:
            armed = bool(
                int(message.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if armed:
                print("SAFETY_STOP=ARMED")
                return 3
            disarmed_heartbeats += 1
        elif message.get_type() == "RC_CHANNELS":
            ch7 = int(message.chan7_raw)
            ch8 = int(message.chan8_raw)
            samples.append((ch7, ch8))
            if len(samples) >= args.samples and disarmed_heartbeats >= 3:
                break

    if disarmed_heartbeats < 3 or len(samples) < args.samples:
        print(
            f"SAFETY_STOP=INSUFFICIENT_TELEMETRY "
            f"HEARTBEATS={disarmed_heartbeats} RC_SAMPLES={len(samples)}"
        )
        return 2
    ch7_values = [sample[0] for sample in samples]
    ch8_values = [sample[1] for sample in samples]
    print(
        f"RC7_MIN={min(ch7_values)} RC7_MAX={max(ch7_values)} "
        f"RC8_MIN={min(ch8_values)} RC8_MAX={max(ch8_values)}"
    )
    both_low = bool(
        max(ch7_values) <= args.low_maximum
        and max(ch8_values) <= args.low_maximum
    )
    print(
        f"RESULT={'DISARMED_BOTH_LOW' if both_low else 'DISARMED_SWITCH_NOT_LOW'} "
        "READ_ONLY=1 PARAMETER_WRITE=0 MODE_CHANGE=0"
    )
    return 0 if both_low else 4


if __name__ == "__main__":
    raise SystemExit(main())
