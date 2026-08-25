#!/usr/bin/env python3
"""Configure a disarmed Pixhawk TELEM2 for MicoAir MTF-01P.

This deliberately configures only the sensor transport and sensor backends.
It does not alter EKF sources, flight modes, arming checks, RC, actuators,
missions, or safety settings.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


DESIRED = {
    "SERIAL2_PROTOCOL": 1.0,  # MAVLink1
    "SERIAL2_BAUD": 115.0,   # 115200 baud
    # ArduPilot 4.7 moved no-forward from SERIALx_OPTIONS bit 10 to
    # the matching MAVLink channel's MAVx_OPTIONS bit 1.  SERIAL2 is
    # the third MAVLink channel (USB/SERIAL0=MAV1, SERIAL1=MAV2,
    # SERIAL2=MAV3), so leave UART options clear and isolate MAV3.
    "SERIAL2_OPTIONS": 0.0,
    "MAV3_OPTIONS": 2.0,      # bit 1: don't forward MAVLink to/from
    "FLOW_TYPE": 5.0,        # MAVLink optical flow
    "RNGFND1_TYPE": 10.0,    # MAVLink rangefinder
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
            or int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            or message.get_srcComponent() != 1
        ):
            continue
        armed = bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        observed += 1
        print(f"HEARTBEAT_{observed} sys={message.get_srcSystem()} comp=1 ARMED={int(armed)}", flush=True)
        if armed:
            return False
    return observed == count


def read_parameter(link, target_system: int, name: str) -> dict | None:
    for _attempt in range(4):
        link.mav.param_request_read_send(target_system, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return {"value": float(message.param_value), "type": int(message.param_type)}
    return None


def write_and_confirm(link, target_system: int, name: str, value: float, param_type: int) -> float | None:
    for _attempt in range(4):
        link.mav.param_set_send(target_system, 1, name.encode("ascii"), value, param_type)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return float(message.param_value)
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("backup", "apply"))
    parser.add_argument("--device", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--backup", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=250,
        source_component=190,
    )
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        print("SAFETY_STOP=NO_HEARTBEAT")
        return 2
    target_system = heartbeat.get_srcSystem()
    if not wait_disarmed(link):
        print("SAFETY_STOP=FIVE_DISARMED_HEARTBEATS_NOT_CONFIRMED")
        return 3

    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    current = {}
    for name in DESIRED:
        record = read_parameter(link, target_system, name)
        if record is None:
            print(f"SAFETY_STOP=PARAMETER_NOT_RECEIVED:{name}")
            return 4
        current[name] = record
        print(f"BEFORE {name}={record['value']} TYPE={record['type']}", flush=True)

    args.backup.parent.mkdir(parents=True, exist_ok=True)
    args.backup.write_text(
        json.dumps(
            {
                "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "vehicle_system": target_system,
                "armed": False,
                "parameters": current,
                "desired": DESIRED,
                "scope": "MTF-01P sensor transport only; EKF unchanged",
            },
            indent=2,
        ) + "\n",
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
        confirmed = write_and_confirm(link, target_system, name, desired, int(before["type"]))
        print(f"SET {name} requested={desired} confirmed={confirmed}", flush=True)
        if confirmed is None or abs(confirmed - desired) > 0.001:
            print(f"SAFETY_STOP=WRITE_NOT_CONFIRMED:{name}")
            return 5
        changed.append(name)

    print("CHANGED=" + ",".join(changed))
    print("SAFETY=DISARMED SENSOR_SCOPE_ONLY EKF_UNCHANGED=1 MODE_CHANGE=0 ARM_COMMAND=0 MOTOR_COMMAND=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
