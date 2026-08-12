#!/usr/bin/env python3
"""Read-only static quality audit for MTF-01P optical flow and range data."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter

from pymavlink import mavutil


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
        "max_abs": max(abs(value) for value in values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    armed_samples = []
    quality = []
    ground_distance_m = []
    distance_sensor_m = []
    flow_comp_x_mps = []
    flow_comp_y_mps = []
    flow_rate_x = []
    flow_rate_y = []
    ekf_flags = []
    source_counts: dict[str, Counter[str]] = {
        "OPTICAL_FLOW": Counter(),
        "DISTANCE_SENSOR": Counter(),
        "HEARTBEAT": Counter(),
    }

    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        name = message.get_type()
        if name in source_counts:
            source_counts[name][
                f"{message.get_srcSystem()}/{message.get_srcComponent()}"
            ] += 1
        if (
            name == "HEARTBEAT"
            and message.get_srcSystem() == 1
            and message.get_srcComponent() == 1
        ):
            armed_samples.append(
                bool(
                    int(message.base_mode)
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
            )
        elif name == "OPTICAL_FLOW":
            base_values = (
                float(message.flow_comp_m_x),
                float(message.flow_comp_m_y),
                float(message.ground_distance),
            )
            if all(math.isfinite(value) for value in base_values):
                quality.append(float(message.quality))
                flow_comp_x_mps.append(base_values[0])
                flow_comp_y_mps.append(base_values[1])
                ground_distance_m.append(base_values[2])
            rate_x = getattr(message, "flow_rate_x", None)
            rate_y = getattr(message, "flow_rate_y", None)
            if rate_x is not None and math.isfinite(float(rate_x)):
                flow_rate_x.append(float(rate_x))
            if rate_y is not None and math.isfinite(float(rate_y)):
                flow_rate_y.append(float(rate_y))
        elif name == "DISTANCE_SENSOR":
            distance_sensor_m.append(float(message.current_distance) / 100.0)
        elif name == "EKF_STATUS_REPORT":
            ekf_flags.append(int(message.flags))

    result = {
        "device": args.device,
        "baud": args.baud,
        "duration_s": args.duration,
        "disarmed_heartbeat_samples": sum(not value for value in armed_samples),
        "armed_heartbeat_samples": sum(value for value in armed_samples),
        "optical_flow_quality": stats(quality),
        "optical_flow_ground_distance_m": stats(ground_distance_m),
        "distance_sensor_m": stats(distance_sensor_m),
        "flow_comp_m_x_mps": stats(flow_comp_x_mps),
        "flow_comp_m_y_mps": stats(flow_comp_y_mps),
        "flow_rate_x": stats(flow_rate_x),
        "flow_rate_y": stats(flow_rate_y),
        "ekf_flags": sorted(set(ekf_flags)),
        "message_source_counts": {
            name: dict(counts) for name, counts in source_counts.items()
        },
        "read_only": True,
        "parameter_request": False,
        "parameter_write": False,
        "arm_command": False,
        "mode_change": False,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not quality or not distance_sensor_m or any(armed_samples):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
