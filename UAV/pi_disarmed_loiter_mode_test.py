#!/usr/bin/env python3
"""Confirm LOITER acceptance while disarmed, then restore STABILIZE."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def armed(message) -> bool:
    return bool(
        int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def wait_disarmed(link, count: int = 5) -> bool:
    observed = 0
    deadline = time.monotonic() + 15.0
    while observed < count and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is None
            or message.get_srcSystem() != 1
            or message.get_srcComponent() != 1
        ):
            continue
        observed += 1
        print(
            f"GATE_HEARTBEAT_{observed} mode={mavutil.mode_string_v10(message)} "
            f"armed={int(armed(message))}",
            flush=True,
        )
        if armed(message):
            return False
    return observed == count


def request_and_confirm(link, mode: str, timeout_s: float = 8.0) -> bool:
    mapping = link.mode_mapping()
    if mode not in mapping:
        print(f"MODE_NOT_AVAILABLE={mode}")
        return False
    link.mav.set_mode_send(
        1,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            print(f"STATUSTEXT={str(text).rstrip(chr(0))}", flush=True)
        elif message.get_type() == "HEARTBEAT" and message.get_srcComponent() == 1:
            actual = mavutil.mode_string_v10(message).upper()
            print(f"MODE_HEARTBEAT={actual} ARMED={int(armed(message))}", flush=True)
            if armed(message):
                print("SAFETY_STOP=AUTOPILOT_BECAME_ARMED")
                return False
            if actual == mode:
                return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    if not wait_disarmed(link):
        print("SAFETY_STOP=DISARMED_GATE_FAILED")
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    loiter_ok = request_and_confirm(link, "LOITER")
    print(f"LOITER_CONFIRMED={int(loiter_ok)}", flush=True)
    stabilize_ok = request_and_confirm(link, "STABILIZE")
    print(f"STABILIZE_RESTORED={int(stabilize_ok)}", flush=True)
    print(
        "SAFETY=DISARMED_MODE_ACCEPTANCE_ONLY ARM_COMMAND=0 "
        "TAKEOFF_COMMAND=0 MOTOR_COMMAND=0"
    )
    return 0 if loiter_ok and stabilize_ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
