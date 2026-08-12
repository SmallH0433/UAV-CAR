#!/usr/bin/env python3
"""Safely configure a disarmed Pixhawk for optical-flow EKF navigation.

The script is intentionally narrow: it only writes the official ArduPilot
optical-flow source-selection parameters and the MTF-01 serial isolation bit.
It never changes arming checks, flight modes, origin, parameters outside the
declared set, or sends arm/takeoff/land/motor commands.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


DESIRED = {
    # ArduCopter 4.7 rejects the legacy SERIALx_OPTIONS no-forward bit at
    # pre-arm.  MAVLink forwarding is controlled with MAVn_OPTIONS instead.
    "SERIAL2_OPTIONS": 0.0,
    "EK3_SRC_OPTIONS": 0.0,
    "EK3_SRC1_POSXY": 0.0,
    "EK3_SRC1_VELXY": 5.0,
    "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELZ": 0.0,
    "EK3_SRC1_YAW": 1.0,
}


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def wait_disarmed(link, count: int = 5, timeout_s: float = 15.0) -> bool:
    observed = 0
    deadline = time.monotonic() + timeout_s
    while observed < count and time.monotonic() < deadline:
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
        print(f"HEARTBEAT_{observed}=1/1 ARMED={int(armed)}", flush=True)
        if armed:
            return False
    return observed == count


def read_parameter(link, name: str) -> dict | None:
    for _attempt in range(4):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return {
                    "value": float(message.param_value),
                    "type": int(message.param_type),
                }
    return None


def write_and_confirm(link, name: str, value: float, param_type: int) -> float | None:
    for _attempt in range(4):
        link.mav.param_set_send(1, 1, name.encode("ascii"), value, param_type)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return float(message.param_value)
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("backup", "apply"))
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--backup", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    if not wait_disarmed(link):
        print("SAFETY_STOP=FIVE_DISARMED_HEARTBEATS_NOT_CONFIRMED")
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    current = {}
    for name in DESIRED:
        record = read_parameter(link, name)
        if record is None:
            print(f"SAFETY_STOP=PARAMETER_NOT_RECEIVED:{name}")
            return 3
        current[name] = record
        print(f"BEFORE {name}={record['value']} TYPE={record['type']}", flush=True)

    args.backup.parent.mkdir(parents=True, exist_ok=True)
    args.backup.write_text(
        json.dumps(
            {
                "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "vehicle": "real Pixhawk system 1 component 1",
                "armed": False,
                "parameters": current,
                "desired": DESIRED,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"BACKUP={args.backup}", flush=True)

    if args.phase == "backup":
        print("READ_ONLY=1 PARAMETERS_CHANGED=0")
        return 0

    changed = []
    for name, desired in DESIRED.items():
        before = current[name]
        if abs(float(before["value"]) - desired) <= 0.001:
            print(f"UNCHANGED {name}={desired}", flush=True)
            continue
        confirmed = write_and_confirm(link, name, desired, int(before["type"]))
        print(
            f"SET {name} requested={desired} confirmed={confirmed}",
            flush=True,
        )
        if confirmed is None or abs(confirmed - desired) > 0.001:
            print(f"SAFETY_STOP=WRITE_NOT_CONFIRMED:{name}")
            return 4
        changed.append(name)

    print("CHANGED=" + ",".join(changed))
    print(
        "SAFETY=DISARMED_PARAMETER_SCOPE_ONLY ARMING_CHECK_UNCHANGED=1 "
        "ORIGIN_UNCHANGED=1 MODE_CHANGE=0 ARM_COMMAND=0 MOTOR_COMMAND=0"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
