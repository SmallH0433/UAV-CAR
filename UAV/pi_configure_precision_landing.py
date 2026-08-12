#!/usr/bin/env python3
"""Safely back up or change only real-Pixhawk precision-landing parameters."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


PARAMETERS = (
    "PLND_ENABLED",
    "PLND_TYPE",
    "PLND_EST_TYPE",
    "PLND_OPTIONS",
    "PLND_ORIENT",
    "PLND_LAG",
    "PLND_YAW_ALIGN",
    "PLND_CAM_POS_X",
    "PLND_CAM_POS_Y",
    "PLND_CAM_POS_Z",
    "PLND_LAND_OFS_X",
    "PLND_LAND_OFS_Y",
    "PLND_STRICT",
    "PLND_ALT_MIN",
    "PLND_ALT_MAX",
    "PLND_XY_DIST_MAX",
)


def param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def wait_disarmed(link, count: int = 3) -> bool:
    observed = 0
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and observed < count:
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
        print(
            f"HEARTBEAT_{observed}=1/1 ARMED={int(armed)} "
            f"SYSTEM_STATUS={int(message.system_status)}"
        )
        if armed:
            return False
    return observed == count


def read_param(link, name: str) -> dict[str, float | int] | None:
    for _attempt in range(4):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and param_name(message) == name:
                return {
                    "value": float(message.param_value),
                    "type": int(message.param_type),
                }
    return None


def set_and_confirm(link, name: str, value: float, param_type: int) -> float | None:
    for _attempt in range(4):
        link.mav.param_set_send(1, 1, name.encode("ascii"), value, param_type)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and param_name(message) == name:
                return float(message.param_value)
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("backup", "set-enabled", "set-type", "rollback"))
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--backup", type=Path)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    if not wait_disarmed(link):
        print("SAFETY_STOP=NOT_CONFIRMED_DISARMED")
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    current: dict[str, dict[str, float | int]] = {}
    for name in PARAMETERS:
        record = read_param(link, name)
        if record is not None:
            current[name] = record
            print(f"BEFORE {name}={record['value']} TYPE={record['type']}")
    for required in ("PLND_ENABLED", "PLND_TYPE"):
        if required not in current:
            print(f"SAFETY_STOP={required}_NOT_RECEIVED")
            return 3

    if args.backup is not None:
        args.backup.parent.mkdir(parents=True, exist_ok=True)
        args.backup.write_text(
            json.dumps(
                {
                    "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "vehicle": "real Pixhawk system 1 component 1",
                    "armed": False,
                    "parameters": current,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"BACKUP={args.backup}")

    if args.phase == "backup":
        print("MODE=READ_ONLY PARAMETERS_CHANGED=0")
        return 0

    requested: list[tuple[str, float]]
    if args.phase == "set-enabled":
        if current["PLND_TYPE"]["value"] != 0.0:
            print("SAFETY_STOP=PLND_TYPE_NOT_ZERO_BEFORE_ENABLE")
            return 4
        requested = [("PLND_ENABLED", 1.0)]
    elif args.phase == "set-type":
        if current["PLND_ENABLED"]["value"] != 1.0:
            print("SAFETY_STOP=PLND_ENABLED_NOT_ONE_BEFORE_TYPE")
            return 5
        requested = [("PLND_TYPE", 1.0)]
    else:
        requested = [("PLND_TYPE", 0.0), ("PLND_ENABLED", 0.0)]

    for name, value in requested:
        record = current[name]
        confirmed = set_and_confirm(link, name, value, int(record["type"]))
        print(f"SET {name} requested={value} confirmed={confirmed}")
        if confirmed is None or abs(confirmed - value) > 0.001:
            print(f"FAILED={name}")
            return 6
    print(
        "SAFETY=DISARMED_PARAMETER_SCOPE_ONLY "
        "ARM_COMMAND=0 MODE_CHANGE=0 MOTOR_COMMAND=0"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
