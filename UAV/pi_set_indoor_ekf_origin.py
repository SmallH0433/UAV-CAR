#!/usr/bin/env python3
"""Safely set and persist an indoor EKF origin while the vehicle is disarmed."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil


PARAMETERS = (
    "AHRS_OPTIONS",
    "AHRS_ORIGIN_LAT",
    "AHRS_ORIGIN_LON",
    "AHRS_ORIGIN_ALT",
)
RECORD_ORIGIN = 1 << 3
USE_RECORDED_ORIGIN_FOR_NON_GPS = 1 << 4


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_disarmed(link, samples: int = 5, timeout_s: float = 15.0) -> None:
    observed = 0
    deadline = time.monotonic() + timeout_s
    while observed < samples and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            continue
        if armed(message):
            raise RuntimeError("Safety stop: flight controller is armed")
        observed += 1
    if observed != samples:
        raise RuntimeError("Safety stop: five disarmed heartbeats were not received")


def read_parameter(link, name: str, timeout_s: float = 4.0) -> tuple[float, int]:
    for _ in range(3):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return float(message.param_value), int(message.param_type)
    raise RuntimeError(f"parameter not received: {name}")


def set_parameter(link, name: str, value: float, param_type: int, tolerance: float) -> float:
    for _ in range(4):
        link.mav.param_set_send(1, 1, name.encode("ascii"), float(value), int(param_type))
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is None or parameter_name(message) != name:
                continue
            actual = float(message.param_value)
            if math.isclose(actual, value, rel_tol=0.0, abs_tol=tolerance):
                return actual
            break
    raise RuntimeError(f"parameter verification failed: {name}")


def confirm_runtime_origin(link, latitude: float, longitude: float, altitude_m: float) -> dict:
    latitude_e7 = int(round(latitude * 1e7))
    longitude_e7 = int(round(longitude * 1e7))
    altitude_mm = int(round(altitude_m * 1000.0))
    for _ in range(4):
        link.mav.set_gps_global_origin_send(1, latitude_e7, longitude_e7, altitude_mm)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = link.recv_match(
                type=["GPS_GLOBAL_ORIGIN", "GLOBAL_POSITION_INT"],
                blocking=True,
                timeout=0.5,
            )
            if message is None or message.get_srcSystem() != 1:
                continue
            if message.get_type() == "GPS_GLOBAL_ORIGIN":
                result = {
                    "confirmation_message": "GPS_GLOBAL_ORIGIN",
                    "latitude_e7": int(message.latitude),
                    "longitude_e7": int(message.longitude),
                    "altitude_mm": int(message.altitude),
                }
            else:
                result = {
                    "confirmation_message": "GLOBAL_POSITION_INT",
                    "latitude_e7": int(message.lat),
                    "longitude_e7": int(message.lon),
                    "altitude_mm": int(message.alt),
                }
            if (
                abs(result["latitude_e7"] - latitude_e7) <= 20
                and abs(result["longitude_e7"] - longitude_e7) <= 20
            ):
                return result
    raise RuntimeError("runtime GPS_GLOBAL_ORIGIN was not confirmed")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--latitude", type=float, required=True)
    parser.add_argument("--longitude", type=float, required=True)
    parser.add_argument("--altitude-m", type=float, default=0.0)
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    if not (-90.0 <= args.latitude <= 90.0 and -180.0 <= args.longitude <= 180.0):
        raise ValueError("invalid latitude or longitude")
    if abs(args.latitude) < 1e-9 and abs(args.longitude) < 1e-9:
        raise ValueError("0,0 is not a valid project origin")

    link = mavutil.mavlink_connection(
        args.device, baud=args.baud, autoreconnect=False,
        source_system=255, source_component=191,
    )
    try:
        wait_disarmed(link)
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        before = {}
        types = {}
        for name in PARAMETERS:
            value, param_type = read_parameter(link, name)
            before[name] = value
            types[name] = param_type
        args.backup.parent.mkdir(parents=True, exist_ok=True)
        args.backup.write_text(json.dumps({"before": before, "types": types}, indent=2) + "\n")

        desired_options = int(round(before["AHRS_OPTIONS"])) | RECORD_ORIGIN | USE_RECORDED_ORIGIN_FOR_NON_GPS
        written = {
            "AHRS_OPTIONS": set_parameter(link, "AHRS_OPTIONS", desired_options, types["AHRS_OPTIONS"], 0.1),
            "AHRS_ORIGIN_LAT": set_parameter(link, "AHRS_ORIGIN_LAT", args.latitude, types["AHRS_ORIGIN_LAT"], 2e-6),
            "AHRS_ORIGIN_LON": set_parameter(link, "AHRS_ORIGIN_LON", args.longitude, types["AHRS_ORIGIN_LON"], 2e-6),
            "AHRS_ORIGIN_ALT": set_parameter(link, "AHRS_ORIGIN_ALT", args.altitude_m, types["AHRS_ORIGIN_ALT"], 0.05),
        }
        runtime = confirm_runtime_origin(link, args.latitude, args.longitude, args.altitude_m)
        after = {name: read_parameter(link, name)[0] for name in PARAMETERS}
        result = {
            "scope": "disarmed_indoor_ekf_origin_configuration",
            "requested": {"latitude": args.latitude, "longitude": args.longitude, "altitude_m": args.altitude_m},
            "before": before,
            "written_acknowledgements": written,
            "after": after,
            "runtime_gps_global_origin": runtime,
            "record_origin_enabled": bool(int(round(after["AHRS_OPTIONS"])) & RECORD_ORIGIN),
            "restore_non_gps_origin_enabled": bool(int(round(after["AHRS_OPTIONS"])) & USE_RECORDED_ORIGIN_FOR_NON_GPS),
            "armed": False,
            "parameter_writes": list(PARAMETERS),
            "mode_change": False,
            "arm_command": False,
            "takeoff_command": False,
            "land_command": False,
            "motor_command": False,
            "guided_velocity_command": False,
        }
        args.result.parent.mkdir(parents=True, exist_ok=True)
        args.result.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
