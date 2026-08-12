#!/usr/bin/env python3
"""Collect one disarmed MTF-01P range/flow bench sample over MAVLink."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

from pymavlink import mavutil


def numeric_stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "median": None,
            "max": None,
            "stddev": None,
        }
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "max": max(values),
        "stddev": statistics.pstdev(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--true-distance-m", type=float, required=True)
    parser.add_argument("--nominal-label", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=254,
        source_component=191,
    )

    gate_samples = 0
    gate_deadline = time.monotonic() + 15.0
    while gate_samples < 5 and time.monotonic() < gate_deadline:
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
        if armed:
            link.close()
            raise RuntimeError("Safety stop: flight controller is armed")
        gate_samples += 1
    if gate_samples < 5:
        link.close()
        raise RuntimeError("Did not receive five disarmed FC heartbeats")

    message_counts: Counter[str] = Counter()
    source_counts: Counter[str] = Counter()
    distance_by_source: dict[str, list[float]] = defaultdict(list)
    distance_orientation_by_source: dict[str, Counter[int]] = defaultdict(Counter)
    flow_quality_by_source: dict[str, list[float]] = defaultdict(list)
    flow_distance_by_source: dict[str, list[float]] = defaultdict(list)
    armed_heartbeats = 0
    disarmed_heartbeats = gate_samples

    started = time.monotonic()
    while time.monotonic() - started < args.duration_s:
        message = link.recv_match(blocking=True, timeout=0.25)
        if message is None:
            continue
        message_type = message.get_type()
        source = f"{message.get_srcSystem()}/{message.get_srcComponent()}"
        message_counts[message_type] += 1
        source_counts[source] += 1

        if message_type == "HEARTBEAT" and source == "1/1":
            armed = bool(
                int(message.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if armed:
                armed_heartbeats += 1
                link.close()
                raise RuntimeError("Safety stop: FC became armed during collection")
            disarmed_heartbeats += 1
        elif message_type == "DISTANCE_SENSOR":
            current_cm = float(message.current_distance)
            if math.isfinite(current_cm) and current_cm > 0:
                distance_by_source[source].append(current_cm / 100.0)
            distance_orientation_by_source[source][int(message.orientation)] += 1
        elif message_type == "OPTICAL_FLOW":
            flow_quality_by_source[source].append(float(message.quality))
            ground_distance = float(message.ground_distance)
            if math.isfinite(ground_distance) and ground_distance >= 0:
                flow_distance_by_source[source].append(ground_distance)

    link.close()
    elapsed = time.monotonic() - started

    fc_distances = distance_by_source.get("1/1", [])
    fc_stats = numeric_stats(fc_distances)
    mean_distance = fc_stats["mean"]
    error_m = (
        float(mean_distance) - args.true_distance_m
        if mean_distance is not None
        else None
    )
    summary = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "props_off_disarmed_receive_only_multihheight_bench",
        "nominal_label": args.nominal_label,
        "true_distance_m": args.true_distance_m,
        "duration_s": elapsed,
        "safety": {
            "disarmed_heartbeat_samples": disarmed_heartbeats,
            "armed_heartbeat_samples": armed_heartbeats,
            "mavlink_packets_transmitted": 0,
            "parameter_writes": 0,
            "mode_changes": 0,
            "arm_commands": 0,
        },
        "message_counts": dict(message_counts),
        "source_counts": dict(source_counts),
        "distance_sensor": {
            source: {
                "distance_m": numeric_stats(values),
                "orientation_counts": dict(
                    distance_orientation_by_source.get(source, Counter())
                ),
            }
            for source, values in distance_by_source.items()
        },
        "optical_flow": {
            source: {
                "quality": numeric_stats(values),
                "ground_distance_m": numeric_stats(
                    flow_distance_by_source.get(source, [])
                ),
            }
            for source, values in flow_quality_by_source.items()
        },
        "fc_1_1_result": {
            "distance_m": fc_stats,
            "error_m": error_m,
            "absolute_error_m": abs(error_m) if error_m is not None else None,
            "raw_mtf_sysid_200_forwarded": "200/88" in source_counts,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if fc_distances and armed_heartbeats == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
