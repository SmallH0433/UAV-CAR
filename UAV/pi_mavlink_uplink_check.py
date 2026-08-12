#!/usr/bin/env python3
"""Check Pi-to-autopilot MAVLink uplink using a targeted PING only."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--source-system", type=int, default=191)
    parser.add_argument("--source-component", type=int, default=191)
    args = parser.parse_args()

    source_system = args.source_system
    source_component = args.source_component
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=source_system,
        source_component=source_component,
    )

    deadline = time.monotonic() + 15.0
    heartbeat = None
    while time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is not None
            and int(message.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            heartbeat = message
            break
    if heartbeat is None:
        print("AUTOPILOT_HEARTBEAT=NOT_RECEIVED")
        return 2

    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    time.sleep(0.25)
    sequence = 260806
    sent_usec = int(time.time() * 1_000_000)
    for attempt in range(1, 4):
        link.mav.ping_send(sent_usec, sequence, target_system, target_component)
        print(
            f"PING_SENT attempt={attempt} source={source_system}/{source_component} "
            f"target={target_system}/{target_component} sequence={sequence}"
        )
        time.sleep(0.2)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="PING", blocking=True, timeout=0.5)
        if message is None:
            continue
        print(
            f"PING_RECEIVED source={message.get_srcSystem()}/{message.get_srcComponent()} "
            f"target={message.target_system}/{message.target_component} "
            f"sequence={message.seq}"
        )
        if (
            int(message.seq) == sequence
            and int(message.target_system) == source_system
            and int(message.target_component) == source_component
        ):
            print("UPLINK=WORKING")
            return 0

    print("UPLINK=NO_REPLY")
    print("PARAMETERS_CHANGED=0 ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
