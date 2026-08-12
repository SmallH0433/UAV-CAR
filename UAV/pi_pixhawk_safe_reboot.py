#!/usr/bin/env python3
"""Reboot a bench Pixhawk only after three real-FC disarmed heartbeats."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    observed = 0
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and observed < 3:
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
        observed += 1
        print(f"HEARTBEAT_{observed}=1/1 ARMED={int(armed)}")
        if armed:
            print("REBOOT=REFUSED_ARMED")
            return 3
    if observed != 3:
        print("REBOOT=REFUSED_NO_THREE_HEARTBEATS")
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    link.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        message = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if (
            message is not None
            and int(message.command)
            == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        ):
            print(f"REBOOT_ACK_RESULT={int(message.result)}")
            return 0
    print("REBOOT_SENT_ACK_NOT_OBSERVED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
