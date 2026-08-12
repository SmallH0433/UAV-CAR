#!/usr/bin/env python3
"""Observe routed LANDING_TARGET messages on Pixhawk USB without commands."""

from __future__ import annotations

import argparse
import json
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM4")
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=115200,
        autoreconnect=False,
        source_system=250,
        source_component=191,
    )
    deadline = time.monotonic() + args.duration
    count = 0
    arm_samples = []
    first = None
    last = None
    print(f"LISTENING={args.device} DURATION_S={args.duration}", flush=True)
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        if (
            message.get_type() == "HEARTBEAT"
            and message.get_srcSystem() == 1
            and message.get_srcComponent() == 1
        ):
            armed = int(
                bool(
                    int(message.base_mode)
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
            )
            arm_samples.append(armed)
            if armed:
                print("SAFETY_STOP=VEHICLE_ARMED", flush=True)
                break
        if (
            message.get_type() == "LANDING_TARGET"
            and message.get_srcSystem() == 191
            and message.get_srcComponent() == 191
        ):
            record = {
                "frame": int(message.frame),
                "position_valid": int(message.position_valid),
                "angle_x": float(message.angle_x),
                "angle_y": float(message.angle_y),
                "distance": float(message.distance),
                "x": float(message.x),
                "y": float(message.y),
                "z": float(message.z),
            }
            count += 1
            if first is None:
                first = record
            last = record

    print(f"LANDING_TARGET_RECEIVED={count}")
    print(f"FIRST={json.dumps(first, sort_keys=True)}")
    print(f"LAST={json.dumps(last, sort_keys=True)}")
    print(f"ARM_SAMPLES={arm_samples}")
    print("READ_ONLY=1 ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0")
    return 0 if count > 0 and not any(arm_samples) else 3


if __name__ == "__main__":
    raise SystemExit(main())
