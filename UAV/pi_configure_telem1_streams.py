#!/usr/bin/env python3
"""Read or configure a minimal ArduPilot TELEM1 telemetry stream set.

Only SR1_* stream-rate parameters are in scope. No arming, mode, motor,
navigation, or flight-control parameters are touched.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


RECOMMENDED_RATES_HZ = {
    "SR1_EXT_STAT": 2.0,
    "SR1_RC_CHAN": 5.0,
    "SR1_POSITION": 3.0,
    "SR1_EXTRA1": 10.0,
    "SR1_EXTRA2": 3.0,
    "SR1_EXTRA3": 3.0,
}


def wait_autopilot_heartbeat(link, timeout: float = 15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is None:
            continue
        if int(message.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return message
    return None


def normalized_param_id(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def read_param(
    link, target_system: int, target_component: int, name: str, timeout: float = 4.0
) -> float | None:
    link.mav.param_request_read_send(
        target_system,
        target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and normalized_param_id(message) == name:
            return float(message.param_value)
    return None


def set_param(
    link,
    target_system: int,
    target_component: int,
    name: str,
    value: float,
    timeout: float = 5.0,
) -> float | None:
    link.mav.param_set_send(
        target_system,
        target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and normalized_param_id(message) == name:
            return float(message.param_value)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure TELEM1 SR1 stream rates")
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--source-system", type=int, default=191)
    parser.add_argument("--source-component", type=int, default=191)
    parser.add_argument("--target-component", type=int, default=1)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--backup",
        type=Path,
        default=Path.home() / "uav" / "logs" / "telem1_stream_backup.json",
    )
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    heartbeat = wait_autopilot_heartbeat(link)
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2

    target_system = heartbeat.get_srcSystem() or link.target_system or 1
    target_component = args.target_component
    print(
        "HEARTBEAT_SOURCE="
        f"{heartbeat.get_srcSystem()}/{heartbeat.get_srcComponent()} "
        f"PARAM_TARGET={target_system}/{target_component}"
    )
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    time.sleep(0.25)

    current: dict[str, float | None] = {}
    for name in RECOMMENDED_RATES_HZ:
        current[name] = read_param(link, target_system, target_component, name)
        print(f"CURRENT {name}={current[name]}")

    args.backup.parent.mkdir(parents=True, exist_ok=True)
    backup_record = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "device": args.device,
        "baud": args.baud,
        "source_system": args.source_system,
        "source_component": args.source_component,
        "heartbeat_source_system": heartbeat.get_srcSystem(),
        "heartbeat_source_component": heartbeat.get_srcComponent(),
        "target_system": target_system,
        "target_component": target_component,
        "parameters": current,
    }
    args.backup.write_text(
        json.dumps(backup_record, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(f"BACKUP={args.backup}")

    if not args.apply:
        print("MODE=READ_ONLY")
        return 0

    changed = 0
    failed = 0
    for name, minimum in RECOMMENDED_RATES_HZ.items():
        old_value = current[name]
        if old_value is None:
            print(f"SKIP {name}=NOT_FOUND")
            failed += 1
            continue
        target = max(old_value, minimum)
        if target == old_value:
            print(f"KEEP {name}={old_value}")
            continue
        confirmed = set_param(
            link, target_system, target_component, name, target
        )
        if confirmed is None or abs(confirmed - target) > 0.01:
            print(f"FAILED {name} requested={target} confirmed={confirmed}")
            failed += 1
            continue
        print(f"SET {name}: {old_value} -> {confirmed}")
        changed += 1

    print(f"CHANGED={changed} FAILED={failed}")
    print("SCOPE=SR1_STREAM_RATES_ONLY")
    print("ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0")
    return 0 if failed == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
