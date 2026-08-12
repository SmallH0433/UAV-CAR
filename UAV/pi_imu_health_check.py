#!/usr/bin/env python3
"""Read-only Pixhawk IMU and pre-arm health observer over MAVLink."""

from __future__ import annotations

import argparse
import statistics
import time
from collections import Counter, defaultdict

from pymavlink import mavutil


def text_value(message) -> str:
    value = message.text
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=35.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    heartbeat = None
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            candidate is not None
            and candidate.get_srcSystem() == 1
            and candidate.get_srcComponent() == 1
            and int(candidate.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            heartbeat = candidate
            break
    if heartbeat is None:
        print("HEARTBEAT=NOT_RECEIVED")
        return 2

    armed = bool(
        int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )
    print(f"HEARTBEAT=1/1 ARMED={int(armed)} SYSTEM_STATUS={int(heartbeat.system_status)}")
    if armed:
        print("SAFETY_STOP=VEHICLE_ARMED")
        return 3

    counts: Counter[str] = Counter()
    values: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: defaultdict(list)
    )
    status_texts: list[tuple[int, str]] = []
    gyro_health: list[bool] = []
    accel_health: list[bool] = []
    gyro_bit = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
    accel_bit = int(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL)

    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        message = link.recv_match(blocking=True, timeout=1.0)
        if message is None:
            continue
        if message.get_srcSystem() != 1:
            continue
        name = message.get_type()
        counts[name] += 1
        if name == "STATUSTEXT":
            entry = (int(message.severity), text_value(message))
            if entry not in status_texts:
                status_texts.append(entry)
        elif name == "SYS_STATUS":
            health = int(message.onboard_control_sensors_health)
            gyro_health.append(bool(health & gyro_bit))
            accel_health.append(bool(health & accel_bit))
        elif name in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
            for field in ("xgyro", "ygyro", "zgyro", "temperature"):
                if hasattr(message, field):
                    values[name][field].append(float(getattr(message, field)))

    print(f"DURATION_S={args.duration:.1f}")
    print(f"GYRO_HEALTHY_SAMPLES={sum(gyro_health)}/{len(gyro_health)}")
    print(f"ACCEL_HEALTHY_SAMPLES={sum(accel_health)}/{len(accel_health)}")
    for name in ("RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"):
        fields = values.get(name, {})
        if not fields:
            continue
        print(f"{name}_COUNT={counts[name]}")
        temperatures = fields.get("temperature", [])
        if temperatures:
            print(
                f"{name}_TEMP_C_MEAN={statistics.fmean(temperatures) / 100.0:.3f}"
            )
        for axis in ("xgyro", "ygyro", "zgyro"):
            samples = fields.get(axis, [])
            if samples:
                deviation = statistics.pstdev(samples) if len(samples) > 1 else 0.0
                print(
                    f"{name}_{axis.upper()}_MEAN={statistics.fmean(samples):.3f} "
                    f"STD={deviation:.3f}"
                )
    if status_texts:
        for severity, message in status_texts:
            print(f"STATUSTEXT severity={severity} text={message}")
    else:
        print("STATUSTEXT=NONE_RECEIVED")
    print("READ_ONLY=1 PARAMETER_WRITE=0 ARM_COMMAND=0 MODE_CHANGE=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
