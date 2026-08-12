#!/usr/bin/env python3
"""Back up and configure ArduPilot 4.7 TELEM1 (MAV2_*) stream rates."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


TARGET_RATES_HZ = {
    "MAV2_EXT_STAT": 2.0,
    "MAV2_RC_CHAN": 5.0,
    "MAV2_POSITION": 3.0,
    "MAV2_EXTRA1": 10.0,
    "MAV2_EXTRA2": 3.0,
    "MAV2_EXTRA3": 3.0,
}


def name_of(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def receive_all_params(link, timeout: float = 25.0):
    link.mav.param_request_list_send(1, 1)
    records = {}
    expected = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        records[name_of(message)] = {
            "value": float(message.param_value),
            "type": int(message.param_type),
        }
        expected = int(message.param_count)
        if expected > 0 and len(records) >= expected:
            break
    return records, expected


def set_and_confirm(link, name: str, value: float, param_type: int):
    for _attempt in range(3):
        link.mav.param_set_send(1, 1, name.encode("ascii"), value, param_type)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and name_of(message) == name:
                return float(message.param_value), int(message.param_type)
    return None, None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM4")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--backup",
        type=Path,
        default=Path("telem1_mav2_stream_backup_20260806.json"),
    )
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
    if heartbeat.get_srcSystem() != 1 or heartbeat.get_srcComponent() != 1:
        print(
            f"UNEXPECTED_HEARTBEAT={heartbeat.get_srcSystem()}/"
            f"{heartbeat.get_srcComponent()}"
        )
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    records, expected = receive_all_params(link)
    if not records or any(name not in records for name in TARGET_RATES_HZ):
        print(f"PARAMETERS_RECEIVED={len(records)} EXPECTED={expected}")
        print("REQUIRED_MAV2_PARAMETERS=NOT_COMPLETE")
        return 3

    before = {name: records[name] for name in TARGET_RATES_HZ}
    backup = {
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "firmware_heartbeat_source": "1/1",
        "device": args.device,
        "mapping": "ArduPilot 4.7 old SR1_* -> new MAV2_*",
        "parameters": before,
    }
    args.backup.parent.mkdir(parents=True, exist_ok=True)
    args.backup.write_text(json.dumps(backup, indent=2), encoding="utf-8")
    print(f"BACKUP={args.backup.resolve()}")
    for name in TARGET_RATES_HZ:
        print(
            f"BEFORE {name}={before[name]['value']} "
            f"TYPE={before[name]['type']}"
        )

    if not args.apply:
        print("MODE=READ_ONLY")
        return 0

    failures = 0
    for name, minimum in TARGET_RATES_HZ.items():
        old = before[name]["value"]
        target = max(old, minimum)
        if abs(target - old) < 0.001:
            print(f"KEEP {name}={old}")
            continue
        confirmed, confirmed_type = set_and_confirm(
            link, name, target, before[name]["type"]
        )
        if confirmed is None or abs(confirmed - target) > 0.001:
            print(
                f"FAILED {name} requested={target} confirmed={confirmed} "
                f"type={confirmed_type}"
            )
            failures += 1
        else:
            print(f"SET {name}: {old} -> {confirmed} TYPE={confirmed_type}")

    print(f"FAILED={failures}")
    print("SCOPE=TELEM1_MAV2_STREAM_RATES_ONLY")
    print("ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0 REBOOT_COMMAND=0")
    return 0 if failures == 0 else 4


if __name__ == "__main__":
    raise SystemExit(main())
