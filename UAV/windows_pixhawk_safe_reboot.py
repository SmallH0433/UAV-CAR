#!/usr/bin/env python3
"""Reboot a bench Pixhawk only after confirming MAVLink reports disarmed."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM4")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=115200,
        autoreconnect=False,
        source_system=255,
        source_component=190,
    )
    heartbeat = link.wait_heartbeat(timeout=15)
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2
    armed = bool(
        int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )
    print(
        f"HEARTBEAT_SOURCE={heartbeat.get_srcSystem()}/{heartbeat.get_srcComponent()} "
        f"BASE_MODE={int(heartbeat.base_mode)} ARMED={int(armed)}"
    )
    if armed:
        print("REBOOT=REFUSED_ARMED")
        return 3

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
        if message is None:
            continue
        if int(message.command) == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            print(f"REBOOT_ACK_RESULT={int(message.result)}")
            return 0
    print("REBOOT_SENT_ACK_NOT_OBSERVED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
