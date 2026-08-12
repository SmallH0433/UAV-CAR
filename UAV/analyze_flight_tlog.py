#!/usr/bin/env python3
"""Summarise an ArduPilot telemetry log, with armed-flight statistics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil


def finite(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def stats(values: list[float]) -> dict | None:
    if not values:
        return None
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "max": max(values),
        "stddev": statistics.pstdev(values),
    }


def wall_time(timestamp: float | None) -> str | None:
    if timestamp is None:
        return None
    return datetime.fromtimestamp(timestamp).astimezone().isoformat()


def source(message) -> str:
    return f"{message.get_srcSystem()}/{message.get_srcComponent()}"


def analyse(path: Path) -> dict:
    connection = mavutil.mavlink_connection(str(path), notimestamps=False)
    counts = Counter()
    source_counts: dict[str, Counter] = {}
    first_timestamp = None
    last_timestamp = None
    armed = False
    arm_started = None
    armed_segments = []
    mode_segments = []
    current_mode = None
    current_mode_started = None
    status_texts = []
    command_counts = Counter()
    command_acks = Counter()
    during = {
        "relative_altitude_m": [], "vfr_altitude_m": [], "rangefinder_m": [],
        "flow_ground_distance_m": [], "flow_quality": [],
        "flow_comp_x_mps": [], "flow_comp_y_mps": [],
        "local_x_m": [], "local_y_m": [], "local_z_down_m": [],
        "local_vx_mps": [], "local_vy_mps": [], "local_vz_down_mps": [],
        "roll_deg": [], "pitch_deg": [], "climb_mps": [], "groundspeed_mps": [],
        "battery_voltage_v": [], "battery_current_a": [], "battery_remaining_pct": [],
        "vibration_x": [], "vibration_y": [], "vibration_z": [],
        "rc3_pwm": [], "rc5_pwm": [], "rc7_pwm": [],
        "gps_satellites": [], "gps_fix_type": [],
    }
    ekf_flags = Counter()
    landing_target_sources = Counter()
    guided_target_sources = Counter()
    servo_max = {str(index): 0 for index in range(1, 9)}
    last_armed_heartbeat = None

    while True:
        message = connection.recv_match(blocking=False)
        if message is None:
            break
        name = message.get_type()
        if name == "BAD_DATA":
            continue
        timestamp = finite(getattr(message, "_timestamp", None))
        if timestamp is not None:
            first_timestamp = timestamp if first_timestamp is None else min(first_timestamp, timestamp)
            last_timestamp = timestamp if last_timestamp is None else max(last_timestamp, timestamp)
        counts[name] += 1
        source_counts.setdefault(name, Counter())[source(message)] += 1

        if name == "HEARTBEAT" and source(message) == "1/1":
            now_armed = bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = mavutil.mode_string_v10(message).upper()
            if current_mode is None:
                current_mode, current_mode_started = mode, timestamp
            elif mode != current_mode:
                mode_segments.append({"mode": current_mode, "start": wall_time(current_mode_started),
                                      "end": wall_time(timestamp),
                                      "duration_s": None if timestamp is None or current_mode_started is None else timestamp-current_mode_started})
                current_mode, current_mode_started = mode, timestamp
            if now_armed and not armed:
                arm_started = timestamp
            elif armed and not now_armed:
                armed_segments.append({"start": wall_time(arm_started), "end": wall_time(timestamp),
                                       "duration_s": None if timestamp is None or arm_started is None else timestamp-arm_started})
                arm_started = None
            armed = now_armed
            last_armed_heartbeat = timestamp
            continue

        if name == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            status_texts.append({"time": wall_time(timestamp), "severity": int(message.severity),
                                 "text": str(text).rstrip("\x00"), "source": source(message)})
        elif name in {"COMMAND_LONG", "COMMAND_INT"}:
            command_counts[str(int(message.command))] += 1
        elif name == "COMMAND_ACK":
            command_acks[f"{int(message.command)}:{int(message.result)}"] += 1
        elif name == "LANDING_TARGET":
            landing_target_sources[source(message)] += 1
        elif name == "SET_POSITION_TARGET_LOCAL_NED":
            guided_target_sources[source(message)] += 1

        if not armed:
            continue

        if name == "GLOBAL_POSITION_INT":
            during["relative_altitude_m"].append(float(message.relative_alt) / 1000.0)
        elif name == "VFR_HUD":
            for key, value in (("vfr_altitude_m", message.alt), ("climb_mps", message.climb),
                               ("groundspeed_mps", message.groundspeed)):
                number = finite(value)
                if number is not None:
                    during[key].append(number)
        elif name == "DISTANCE_SENSOR" and int(message.orientation) == 25:
            during["rangefinder_m"].append(float(message.current_distance) / 100.0)
        elif name == "OPTICAL_FLOW":
            during["flow_ground_distance_m"].append(float(message.ground_distance))
            during["flow_quality"].append(float(message.quality))
            during["flow_comp_x_mps"].append(float(message.flow_comp_m_x))
            during["flow_comp_y_mps"].append(float(message.flow_comp_m_y))
        elif name == "LOCAL_POSITION_NED":
            for key, value in (("local_x_m", message.x), ("local_y_m", message.y),
                               ("local_z_down_m", message.z), ("local_vx_mps", message.vx),
                               ("local_vy_mps", message.vy), ("local_vz_down_mps", message.vz)):
                number = finite(value)
                if number is not None:
                    during[key].append(number)
        elif name == "ATTITUDE":
            during["roll_deg"].append(math.degrees(float(message.roll)))
            during["pitch_deg"].append(math.degrees(float(message.pitch)))
        elif name == "SYS_STATUS":
            during["battery_voltage_v"].append(float(message.voltage_battery) / 1000.0)
            if int(message.current_battery) >= 0:
                during["battery_current_a"].append(float(message.current_battery) / 100.0)
            if int(message.battery_remaining) >= 0:
                during["battery_remaining_pct"].append(float(message.battery_remaining))
        elif name == "VIBRATION":
            during["vibration_x"].append(float(message.vibration_x))
            during["vibration_y"].append(float(message.vibration_y))
            during["vibration_z"].append(float(message.vibration_z))
        elif name == "RC_CHANNELS":
            during["rc3_pwm"].append(float(message.chan3_raw))
            during["rc5_pwm"].append(float(message.chan5_raw))
            during["rc7_pwm"].append(float(message.chan7_raw))
        elif name == "GPS_RAW_INT":
            during["gps_satellites"].append(float(message.satellites_visible))
            during["gps_fix_type"].append(float(message.fix_type))
        elif name == "EKF_STATUS_REPORT":
            ekf_flags[str(int(message.flags))] += 1
        elif name == "SERVO_OUTPUT_RAW":
            for index in range(1, 9):
                value = int(getattr(message, f"servo{index}_raw", 0))
                servo_max[str(index)] = max(servo_max[str(index)], value)

    if armed:
        armed_segments.append({"start": wall_time(arm_started), "end": wall_time(last_timestamp),
                               "duration_s": None if last_timestamp is None or arm_started is None else last_timestamp-arm_started,
                               "ended_while_still_armed": True})
    if current_mode is not None:
        mode_segments.append({"mode": current_mode, "start": wall_time(current_mode_started),
                              "end": wall_time(last_timestamp),
                              "duration_s": None if last_timestamp is None or current_mode_started is None else last_timestamp-current_mode_started})

    return {
        "path": str(path.resolve()), "size_bytes": path.stat().st_size,
        "first_time": wall_time(first_timestamp), "last_time": wall_time(last_timestamp),
        "duration_s": None if first_timestamp is None or last_timestamp is None else last_timestamp-first_timestamp,
        "message_counts": dict(counts),
        "armed_segments": armed_segments, "mode_segments": mode_segments,
        "armed_statistics": {key: stats(values) for key, values in during.items()},
        "ekf_flags_during_armed": dict(ekf_flags),
        "landing_target_sources": dict(landing_target_sources),
        "guided_velocity_target_sources": dict(guided_target_sources),
        "command_counts": dict(command_counts), "command_ack_counts": dict(command_acks),
        "servo_output_max_during_armed": servo_max,
        "status_texts": status_texts,
        "selected_source_counts": {name: dict(source_counts.get(name, {})) for name in (
            "HEARTBEAT", "GPS_RAW_INT", "OPTICAL_FLOW", "DISTANCE_SENSOR", "LANDING_TARGET",
            "SET_POSITION_TARGET_LOCAL_NED", "STATUSTEXT")},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = [analyse(path) for path in args.logs]
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
