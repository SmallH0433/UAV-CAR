#!/usr/bin/env python3
"""Probe MAV_CMD_ACTUATOR_TEST support without moving an actuator."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil


def is_fc(message) -> bool:
    return (
        message is not None
        and message.get_srcSystem() == 1
        and message.get_srcComponent() == 1
    )


def armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    heartbeats = []
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and len(heartbeats) < 5:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_fc(message):
            continue
        record = {
            "armed": armed(message),
            "base_mode": int(message.base_mode),
            "custom_mode": int(message.custom_mode),
            "system_status": int(message.system_status),
        }
        heartbeats.append(record)
        print(f"HEARTBEAT_{len(heartbeats)}={record}")
        if record["armed"]:
            print("SAFETY_STOP=ARMED")
            return 3
    if len(heartbeats) != 5:
        print("SAFETY_STOP=HEARTBEAT_TIMEOUT")
        return 3

    command = 310  # MAV_CMD_ACTUATOR_TEST
    # NaN means disarmed/stop and timeout 0 restores immediately.  This probes
    # command support without requesting any actuator motion.
    link.mav.command_long_send(1, 1, command, 0, math.nan, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    ack = None
    texts = []
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "COMMAND_ACK" and int(message.command) == command:
            ack = int(message.result)
            print(f"ACTUATOR_TEST_PROBE_ACK={ack}")
            break
        if message.get_type() == "STATUSTEXT":
            value = message.text
            if isinstance(value, bytes):
                value = value.decode("utf-8", errors="replace")
            texts.append(str(value).rstrip("\x00"))

    final_disarmed = True
    final = []
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and len(final) < 5:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_fc(message):
            continue
        value = armed(message)
        final.append(value)
        if value:
            final_disarmed = False

    result = {
        "probe": "MAV_CMD_ACTUATOR_TEST stop-value timeout-zero",
        "motion_requested": False,
        "ack": ack,
        "statustext": texts,
        "initial_heartbeats": heartbeats,
        "final_armed_flags": final,
        "final_disarmed": final_disarmed and len(final) == 5,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"FINAL_DISARMED={int(result['final_disarmed'])}")
    print(f"OUTPUT={args.output}")
    return 0 if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED and result["final_disarmed"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
