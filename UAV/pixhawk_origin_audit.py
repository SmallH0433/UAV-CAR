#!/usr/bin/env python3
"""Read-only audit of Pixhawk EKF origin persistence and runtime origin."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


PARAMETERS = (
    "AHRS_OPTIONS",
    "AHRS_ORIGIN_LAT",
    "AHRS_ORIGIN_LON",
    "AHRS_ORIGIN_ALT",
    "AHRS_EKF_TYPE",
    "EK3_PRIMARY",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "GPS1_TYPE",
    "FLOW_TYPE",
)


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def is_armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def read_parameter(link, name: str, timeout_s: float = 4.0) -> dict:
    for _ in range(3):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return {
                    "value": float(message.param_value),
                    "type": int(message.param_type),
                }
    raise RuntimeError(f"parameter not received: {name}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--runtime-timeout", type=float, default=8.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    try:
        heartbeats = []
        deadline = time.monotonic() + 15.0
        while len(heartbeats) < 5 and time.monotonic() < deadline:
            message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if (
                message is None
                or message.get_srcSystem() != 1
                or message.get_srcComponent() != 1
                or int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                continue
            heartbeats.append(
                {
                    "armed": is_armed(message),
                    "base_mode": int(message.base_mode),
                    "custom_mode": int(message.custom_mode),
                    "system_status": int(message.system_status),
                }
            )
        if len(heartbeats) != 5:
            raise RuntimeError("five flight-controller heartbeats were not received")

        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        parameters = {name: read_parameter(link, name) for name in PARAMETERS}

        requested_ids = (
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        )
        for message_id in requested_ids:
            link.mav.command_long_send(
                1,
                1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                message_id,
                0,
                0,
                0,
                0,
                0,
                0,
            )

        runtime = {}
        deadline = time.monotonic() + args.runtime_timeout
        while time.monotonic() < deadline and len(runtime) < 3:
            message = link.recv_match(
                type=["GPS_GLOBAL_ORIGIN", "GLOBAL_POSITION_INT", "EKF_STATUS_REPORT"],
                blocking=True,
                timeout=0.5,
            )
            if message is None or message.get_srcSystem() != 1:
                continue
            if message.get_type() == "GPS_GLOBAL_ORIGIN":
                runtime[message.get_type()] = {
                    "latitude_e7": int(message.latitude),
                    "longitude_e7": int(message.longitude),
                    "altitude_mm": int(message.altitude),
                }
            elif message.get_type() == "GLOBAL_POSITION_INT":
                runtime[message.get_type()] = {
                    "latitude_e7": int(message.lat),
                    "longitude_e7": int(message.lon),
                    "altitude_msl_mm": int(message.alt),
                    "relative_altitude_mm": int(message.relative_alt),
                }
            else:
                runtime[message.get_type()] = {
                    "flags": int(message.flags),
                    "velocity_variance": float(message.velocity_variance),
                    "position_horizontal_variance": float(message.pos_horiz_variance),
                    "position_vertical_variance": float(message.pos_vert_variance),
                }

        result = {
            "scope": "read_only_ekf_origin_audit",
            "device": args.device,
            "heartbeats": heartbeats,
            "all_heartbeats_disarmed": all(not item["armed"] for item in heartbeats),
            "parameters": parameters,
            "runtime": runtime,
            "parameter_writes": 0,
            "mode_commands": 0,
            "arm_commands": 0,
            "motor_commands": 0,
        }
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
