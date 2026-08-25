#!/usr/bin/env python3
"""Read-only MAVLink audit for a Pixhawk connected over USB.

The script only requests telemetry, parameters and pre-arm check results.  It
never changes a parameter, mode, arm state, actuator or mission.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path

from pymavlink import mavutil


IMPORTANT_PARAMS = (
    "FRAME_CLASS", "FRAME_TYPE", "AHRS_ORIENTATION",
    "ARMING_CHECK", "BRD_SAFETYENABLE",
    "BATT_MONITOR", "BATT_CAPACITY", "BATT_LOW_VOLT", "BATT_CRT_VOLT",
    "FS_THR_ENABLE", "FS_GCS_ENABLE",
    "GPS1_TYPE", "GPS_TYPE",
    "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",
    "FLOW_TYPE", "RNGFND1_TYPE", "RNGFND1_ORIENT",
    "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
    "SERIAL1_PROTOCOL", "SERIAL1_BAUD",
    "SERIAL2_PROTOCOL", "SERIAL2_BAUD", "SERIAL2_OPTIONS",
    "MOT_PWM_TYPE", "MOT_SPIN_ARM", "MOT_SPIN_MIN",
    "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW",
    "FLTMODE_CH", "FLTMODE1", "FLTMODE2", "FLTMODE3",
    "FLTMODE4", "FLTMODE5", "FLTMODE6",
)

SENSOR_BITS = {
    0: "3D_GYRO", 1: "3D_ACCEL", 2: "3D_MAG", 3: "ABS_PRESSURE",
    4: "DIFF_PRESSURE", 5: "GPS", 6: "OPTICAL_FLOW",
    7: "VISION_POSITION", 8: "LASER_POSITION", 9: "EXTERNAL_GROUND_TRUTH",
    10: "ANGULAR_RATE_CONTROL", 11: "ATTITUDE_STABILIZATION",
    12: "YAW_POSITION", 13: "Z_ALTITUDE_CONTROL",
    14: "XY_POSITION_CONTROL", 15: "MOTOR_OUTPUTS",
    16: "RC_RECEIVER", 17: "3D_GYRO2", 18: "3D_ACCEL2", 19: "3D_MAG2",
    20: "GEOFENCE", 21: "AHRS", 22: "TERRAIN", 23: "REVERSE_MOTOR",
    24: "LOGGING", 25: "BATTERY", 26: "PROXIMITY", 27: "SATCOM",
    28: "PREARM_CHECK", 29: "OBSTACLE_AVOIDANCE", 30: "PROPULSION",
    31: "EXTENSION_USED",
}


def clean(value):
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def message_dict(message):
    data = message.to_dict()
    data.pop("mavpackettype", None)
    return {key: clean(value) for key, value in data.items()}


def decode_bits(mask):
    return [name for bit, name in SENSOR_BITS.items() if int(mask) & (1 << bit)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=18.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=250,
        source_component=190,
        autoreconnect=False,
    )
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit("No MAVLink heartbeat received within 10 seconds")

    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()
    armed = bool(int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        raise SystemExit("Vehicle is ARMED; audit stopped without sending requests")

    link.target_system = target_system
    link.target_component = target_component
    link.mav.request_data_stream_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )
    link.mav.param_request_list_send(target_system, target_component)
    link.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0,
    )
    prearm_command = getattr(mavutil.mavlink, "MAV_CMD_RUN_PREARM_CHECKS", None)
    if prearm_command is not None:
        link.mav.command_long_send(
            target_system, target_component, prearm_command, 0,
            0, 0, 0, 0, 0, 0, 0,
        )

    params = {}
    latest = {}
    counts = Counter()
    status_text = []
    command_acks = []
    param_count_expected = None
    deadline = time.monotonic() + args.seconds

    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.25)
        if message is None or message.get_type() == "BAD_DATA":
            continue
        name = message.get_type()
        counts[name] += 1
        if name == "PARAM_VALUE":
            param_id = message.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode(errors="replace")
            param_id = str(param_id).rstrip("\x00")
            params[param_id] = float(message.param_value)
            param_count_expected = int(message.param_count)
        elif name == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            item = {"severity": int(message.severity), "text": str(text).rstrip("\x00")}
            if item not in status_text:
                status_text.append(item)
        elif name == "COMMAND_ACK":
            command_acks.append(message_dict(message))
        elif name in {
            "HEARTBEAT", "SYS_STATUS", "POWER_STATUS", "HWSTATUS",
            "AUTOPILOT_VERSION", "GPS_RAW_INT", "GPS2_RAW", "ATTITUDE",
            "VFR_HUD", "RC_CHANNELS", "BATTERY_STATUS", "EKF_STATUS_REPORT",
            "RANGEFINDER", "DISTANCE_SENSOR", "OPTICAL_FLOW", "OPTICAL_FLOW_RAD",
            "EXTENDED_SYS_STATE", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3",
        }:
            latest[name] = message_dict(message)

    sys_status = latest.get("SYS_STATUS", {})
    present = int(sys_status.get("onboard_control_sensors_present", 0) or 0)
    enabled = int(sys_status.get("onboard_control_sensors_enabled", 0) or 0)
    healthy = int(sys_status.get("onboard_control_sensors_health", 0) or 0)

    result = {
        "port": args.port,
        "baud": args.baud,
        "target_system": target_system,
        "target_component": target_component,
        "armed": armed,
        "mode": mavutil.mode_string_v10(heartbeat),
        "heartbeat": message_dict(heartbeat),
        "parameter_progress": {
            "received": len(params),
            "expected": param_count_expected,
        },
        "important_parameters": {name: params.get(name) for name in IMPORTANT_PARAMS},
        "sensor_bits": {
            "present": decode_bits(present),
            "enabled": decode_bits(enabled),
            "healthy": decode_bits(healthy),
            "enabled_but_unhealthy": decode_bits(enabled & ~healthy),
        },
        "status_text": status_text,
        "command_acks": command_acks,
        "latest_messages": latest,
        "message_counts": dict(counts),
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
