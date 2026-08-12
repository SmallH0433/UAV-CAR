#!/usr/bin/env python3
"""Read live arm state, status text, and battery telemetry without commands."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    deadline = time.monotonic() + args.duration
    arm_states = []
    status_texts = []
    latest_sys_status = None
    latest_battery = None
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT" and message.get_srcComponent() == 1:
            arm_states.append(
                int(
                    bool(
                        int(message.base_mode)
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                )
            )
        elif message_type == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            status_texts.append((int(message.severity), str(text).rstrip("\x00")))
        elif message_type == "SYS_STATUS":
            latest_sys_status = {
                "voltage_battery_mv": int(message.voltage_battery),
                "current_battery_ca": int(message.current_battery),
                "battery_remaining_pct": int(message.battery_remaining),
            }
        elif message_type == "BATTERY_STATUS":
            latest_battery = {
                "id": int(message.id),
                "current_battery_ca": int(message.current_battery),
                "battery_remaining_pct": int(message.battery_remaining),
            }

    print(f"ARM_SAMPLES={arm_states}")
    print(f"CURRENT_ARMED={arm_states[-1] if arm_states else 'UNKNOWN'}")
    print(f"SYS_STATUS={latest_sys_status}")
    print(f"BATTERY_STATUS={latest_battery}")
    for severity, text in status_texts:
        print(f"STATUSTEXT severity={severity} text={text}")
    print("READ_ONLY=1 ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
