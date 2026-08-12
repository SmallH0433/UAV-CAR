#!/usr/bin/env python3
"""Summarise Raspberry Pi follow-readiness JSONL, especially armed intervals."""

from __future__ import annotations

import argparse
import json
import statistics
from datetime import datetime
from pathlib import Path


def stamp(ts):
    return datetime.fromtimestamp(float(ts)).astimezone().isoformat()


def stats(values):
    if not values:
        return None
    return {"count": len(values), "min": min(values), "mean": statistics.fmean(values),
            "median": statistics.median(values), "max": max(values)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    bad_lines = 0
    for line in args.input.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
            if row.get("timestamp_unix") is not None:
                rows.append(row)
        except json.JSONDecodeError:
            bad_lines += 1
    rows.sort(key=lambda row: float(row["timestamp_unix"]))
    armed_rows = [row for row in rows if row.get("armed") is True]
    intervals = []
    start = previous = None
    for row in armed_rows:
        ts = float(row["timestamp_unix"])
        if start is None or (previous is not None and ts - previous > 2.5):
            if start is not None:
                intervals.append((start, previous))
            start = ts
        previous = ts
    if start is not None:
        intervals.append((start, previous))

    interval_results = []
    for start, end in intervals:
        subset = [row for row in armed_rows if start <= float(row["timestamp_unix"]) <= end]
        interval_results.append({
            "start": stamp(start), "end": stamp(end), "sample_span_s": end-start,
            "samples": len(subset), "modes": sorted({str(row.get("mode")) for row in subset}),
            "rc7_pwm": stats([float(row["rc7_pwm"]) for row in subset if row.get("rc7_pwm") is not None]),
            "range_m": stats([float(row["range_m"]) for row in subset if row.get("range_m") is not None]),
            "flow_quality": stats([float(row["flow_quality"]) for row in subset if row.get("flow_quality") is not None]),
            "battery_voltage_v": stats([float(row["battery_voltage_v"]) for row in subset if row.get("battery_voltage_v") is not None]),
            "target_visible_samples": sum(bool(row.get("target_visible_this_frame")) for row in subset),
            "target_acquired_samples": sum(bool(row.get("target_acquired")) for row in subset),
            "ready_samples": sum(bool(row.get("ready_for_follow_request")) for row in subset),
            "blockers": sorted({item for row in subset for item in row.get("blockers", [])}),
            "mavlink_transmitted_samples": sum(bool(row.get("mavlink_transmitted")) for row in subset),
            "velocity_setpoint_sent_samples": sum(bool(row.get("velocity_setpoint_sent")) for row in subset),
        })

    result = {
        "input": str(args.input.resolve()), "rows": len(rows), "bad_lines": bad_lines,
        "first_time": stamp(rows[0]["timestamp_unix"]) if rows else None,
        "last_time": stamp(rows[-1]["timestamp_unix"]) if rows else None,
        "armed_samples": len(armed_rows), "armed_intervals": interval_results,
        "all_modes": sorted({str(row.get("mode")) for row in rows}),
        "max_range_m_all": max((float(row["range_m"]) for row in rows if row.get("range_m") is not None), default=None),
        "target_acquired_samples_all": sum(bool(row.get("target_acquired")) for row in rows),
        "ch7_high_samples_all": sum((row.get("rc7_pwm") or 0) >= 1800 for row in rows),
        "mavlink_transmitted_any": any(bool(row.get("mavlink_transmitted")) for row in rows),
        "velocity_setpoint_sent_any": any(bool(row.get("velocity_setpoint_sent")) for row in rows),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
