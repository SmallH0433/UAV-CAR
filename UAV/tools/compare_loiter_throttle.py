#!/usr/bin/env python3
"""Compare LOITER position behavior with throttle deadband and other drivers.

Read-only against the exported DataFlash logs.  The output is a compact JSON/CSV
evidence set used by the afternoon flight-log diagnostic report.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_left, bisect_right
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable

from pymavlink import DFReader


KEEP_TYPES = {
    "ARM", "ATT", "CTUN", "MODE", "MOTB", "MSG", "OF", "PARM", "PSCD",
    "PSCE", "PSCN", "RCIN", "RFND", "XKF1",
}


def percentile(values: Iterable[float], p: float) -> float | None:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return None
    i = (len(vals) - 1) * p
    lo, hi = math.floor(i), math.ceil(i)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - i) + vals[hi] * (i - lo)


def round_or_none(value: float | None, digits: int = 3) -> float | None:
    return None if value is None or not math.isfinite(value) else round(value, digits)


def pearson(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3 or len(xs) != len(ys):
        return None
    mx, my = mean(xs), mean(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = math.sqrt(sum((x - mx) ** 2 for x in xs) * sum((y - my) ** 2 for y in ys))
    return num / den if den else None


class LogRows:
    def __init__(self, path: Path):
        self.path = path
        self.rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
        self.times: dict[str, list[float]] = defaultdict(list)
        self.params: dict[str, float] = {}
        reader = DFReader.DFReader_binary(str(path))
        while True:
            msg = reader.recv_msg()
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "BAD_DATA" or kind not in KEEP_TYPES:
                continue
            row = msg.to_dict()
            t = float(row.get("TimeUS", 0)) / 1e6
            row["t"] = t
            self.rows[kind].append(row)
            self.times[kind].append(t)
            if kind == "PARM":
                try:
                    self.params[str(row["Name"])] = float(row["Value"])
                except (KeyError, TypeError, ValueError):
                    pass

    def between(self, kind: str, intervals: list[tuple[float, float]]) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        times = self.times.get(kind, [])
        rows = self.rows.get(kind, [])
        for start, end in intervals:
            lo, hi = bisect_left(times, start), bisect_right(times, end)
            result.extend(rows[lo:hi])
        return result


def arm_intervals(log: LogRows) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    start: float | None = None
    for row in log.rows.get("ARM", []):
        armed = bool(row.get("ArmState"))
        if armed and start is None:
            start = row["t"]
        elif not armed and start is not None:
            intervals.append((start, row["t"]))
            start = None
    return intervals


def mode_intervals(log: LogRows, start: float, end: float, target_mode: int = 5) -> list[tuple[float, float]]:
    modes = log.rows.get("MODE", [])
    current: int | None = None
    for row in modes:
        if row["t"] <= start:
            current = int(row.get("Mode", row.get("ModeNum", -1)))
        else:
            break
    cursor = start
    output: list[tuple[float, float]] = []
    for row in modes:
        t = row["t"]
        if t <= start:
            continue
        if t >= end:
            break
        new_mode = int(row.get("Mode", row.get("ModeNum", -1)))
        if new_mode == current:
            continue
        if current == target_mode and t > cursor:
            output.append((cursor, t))
        current = new_mode
        cursor = t
    if current == target_mode and end > cursor:
        output.append((cursor, end))
    return [(a, b) for a, b in output if b - a >= 0.05]


def throttle_calibration(params: dict[str, float]) -> dict[str, float]:
    rc_min = params.get("RC3_MIN", 1000.0)
    rc_max = params.get("RC3_MAX", 2000.0)
    rc_dz = params.get("RC3_DZ", 30.0)
    reversed_input = bool(params.get("RC3_REVERSED", 0.0))
    thr_dz = params.get("THR_DZ", 100.0)
    trim_low = rc_min + rc_dz

    def control(raw: float) -> float:
        pwm = min(max(raw, rc_min), rc_max)
        if reversed_input:
            pwm = rc_max - (pwm - rc_min)
        if pwm <= trim_low:
            return 0.0
        return 1000.0 * (pwm - trim_low) / (rc_max - trim_low)

    mid_raw = math.floor((rc_min + rc_max) / 2.0)
    mid_control = control(mid_raw)
    low_control = max(0.0, mid_control - thr_dz)
    high_control = min(1000.0, mid_control + thr_dz)

    def raw_from_control(value: float) -> float:
        pwm = trim_low + value * (rc_max - trim_low) / 1000.0
        if reversed_input:
            pwm = rc_max - (pwm - rc_min)
        return pwm

    raw_a, raw_b = raw_from_control(low_control), raw_from_control(high_control)
    return {
        "rc3_min_pwm": rc_min,
        "rc3_max_pwm": rc_max,
        "rc3_bottom_dz_pwm": rc_dz,
        "rc3_reversed": float(reversed_input),
        "thr_dz_control_units": thr_dz,
        "mid_control": mid_control,
        "deadband_low_control": low_control,
        "deadband_high_control": high_control,
        "deadband_low_pwm_approx": min(raw_a, raw_b),
        "deadband_high_pwm_approx": max(raw_a, raw_b),
    }


def rc_control(raw: float, calibration: dict[str, float]) -> float:
    rc_min = calibration["rc3_min_pwm"]
    rc_max = calibration["rc3_max_pwm"]
    trim_low = rc_min + calibration["rc3_bottom_dz_pwm"]
    pwm = min(max(raw, rc_min), rc_max)
    if calibration["rc3_reversed"]:
        pwm = rc_max - (pwm - rc_min)
    if pwm <= trim_low:
        return 0.0
    return 1000.0 * (pwm - trim_low) / (rc_max - trim_low)


def axis_centered(row: dict[str, Any], channel: int, params: dict[str, float]) -> bool:
    value = float(row.get(f"C{channel}", math.nan))
    trim = params.get(f"RC{channel}_TRIM", 1500.0)
    dz = params.get(f"RC{channel}_DZ", 20.0)
    return math.isfinite(value) and abs(value - trim) <= dz


def radius(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    n0, e0 = points[0]
    return max(math.hypot(n - n0, e - e0) for n, e in points)


def endpoint_shift(n_rows: list[dict[str, Any]], e_rows: list[dict[str, Any]], n_field: str, e_field: str) -> float | None:
    if not n_rows or not e_rows:
        return None
    return math.hypot(float(n_rows[-1][n_field]) - float(n_rows[0][n_field]),
                      float(e_rows[-1][e_field]) - float(e_rows[0][e_field]))


def segment_metrics(log: LogRows, flight_id: str, segment_id: str, interval: tuple[float, float]) -> dict[str, Any]:
    intervals = [interval]
    start, end = interval
    cal = throttle_calibration(log.params)
    rcin = log.between("RCIN", intervals)
    c3_pwm = [float(r["C3"]) for r in rcin if r.get("C3") is not None]
    c3_control = [rc_control(v, cal) for v in c3_pwm]
    low, high = cal["deadband_low_control"], cal["deadband_high_control"]
    in_dz = [low <= v <= high for v in c3_control]
    below = [v < low for v in c3_control]
    above = [v > high for v in c3_control]
    xy_centered = [axis_centered(r, 1, log.params) and axis_centered(r, 2, log.params) for r in rcin]

    ctun = log.between("CTUN", intervals)
    dcrt = [float(r["DCRt"]) for r in ctun if r.get("DCRt") is not None]
    crt = [float(r["CRt"]) for r in ctun if r.get("CRt") is not None]
    alt = [float(r["Alt"]) for r in ctun if r.get("Alt") is not None]
    dalt = [float(r["DAlt"]) for r in ctun if r.get("DAlt") is not None]
    alt_err = [abs(a - d) for a, d in zip(alt, dalt)]
    motb = log.between("MOTB", intervals)
    thr_out = [float(r["ThrOut"]) for r in motb if r.get("ThrOut") is not None]
    rfnd = [r for r in log.between("RFND", intervals) if int(r.get("Instance", 0)) == 0]
    dist = [float(r["Dist"]) for r in rfnd if r.get("Dist") is not None]
    gndclr = log.params.get("RNGFND1_GNDCLR", 0.02)
    of_rows = log.between("OF", intervals)
    ofq = [float(r["Qual"]) for r in of_rows if r.get("Qual") is not None]
    xkf = [r for r in log.between("XKF1", intervals) if int(r.get("C", 0)) == 0]
    points = [(float(r["PN"]), float(r["PE"])) for r in xkf if r.get("PN") is not None and r.get("PE") is not None]
    pscn, psce = log.between("PSCN", intervals), log.between("PSCE", intervals)
    messages = [str(r.get("Message", "")) for r in log.between("MSG", intervals)]

    def percent(flags: list[bool]) -> float | None:
        return 100.0 * sum(flags) / len(flags) if flags else None

    likely_ground = bool(
        c3_control
        and percent(below) is not None and percent(below) >= 80.0
        and (not thr_out or (percentile(thr_out, 0.5) or 0.0) <= 0.08)
        and dist and (percentile(dist, 0.5) or 0.0) <= gndclr + 0.015
    )
    return {
        "flight_id": flight_id,
        "segment_id": segment_id,
        "source_log": log.path.name,
        "start_s": start,
        "end_s": end,
        "duration_s": end - start,
        "rc3_pwm_min": min(c3_pwm) if c3_pwm else None,
        "rc3_pwm_median": percentile(c3_pwm, 0.5),
        "rc3_pwm_max": max(c3_pwm) if c3_pwm else None,
        "throttle_in_deadband_pct": percent(in_dz),
        "throttle_below_deadband_pct": percent(below),
        "throttle_above_deadband_pct": percent(above),
        "roll_pitch_centered_pct": percent(xy_centered),
        "pilot_xy_outside_pct": None if not xy_centered else 100.0 - percent(xy_centered),
        "desired_climb_zero_pct": percent([abs(v) <= 0.05 for v in dcrt]),
        "desired_climb_abs_p95_mps": percentile([abs(v) for v in dcrt], 0.95),
        "actual_climb_abs_p95_mps": percentile([abs(v) for v in crt], 0.95),
        "altitude_span_m": max(alt) - min(alt) if alt else None,
        "desired_altitude_span_m": max(dalt) - min(dalt) if dalt else None,
        "vertical_tracking_error_p95_m": percentile(alt_err, 0.95),
        "motor_thrust_median": percentile(thr_out, 0.5),
        "motor_thrust_p95": percentile(thr_out, 0.95),
        "rangefinder_median_m": percentile(dist, 0.5),
        "rangefinder_max_m": max(dist) if dist else None,
        "rangefinder_at_groundclear_pct": percent([v <= gndclr + 0.005 for v in dist]),
        "optical_flow_quality_p05": percentile(ofq, 0.05),
        "optical_flow_quality_median": percentile(ofq, 0.5),
        "ekf_aiding_restart_count": sum("started relative aiding" in m for m in messages),
        "mag_yaw_realign_count": sum("yaw re-aligned" in m or "yaw alignment complete" in m for m in messages),
        "xy_estimated_radius_m": radius(points),
        "xy_target_endpoint_shift_m": endpoint_shift(pscn, psce, "DPN", "DPE"),
        "xy_actual_endpoint_shift_m": endpoint_shift(pscn, psce, "PN", "PE"),
        "likely_ground_or_landing_segment": likely_ground,
    }


def weighted_metric(segments: list[dict[str, Any]], key: str) -> float | None:
    pairs = [(float(s[key]), float(s["duration_s"])) for s in segments if s.get(key) is not None]
    total = sum(w for _, w in pairs)
    return sum(v * w for v, w in pairs) / total if total else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()
    root = Path(args.output_root).resolve()
    summary = json.loads((root / "analysis" / "analysis_summary.json").read_text(encoding="utf-8"))

    log54 = LogRows(root / "dataflash" / "log054" / "pixhawk_log_054.BIN")
    log55 = LogRows(root / "dataflash" / "log055" / "pixhawk_log_055.BIN")
    logs = {"log54": log54, "log55": log55}

    flights_by_log: dict[str, list[tuple[str, float, float]]] = defaultdict(list)
    f01 = next(f for f in summary["flight_sessions"] if f["flight_id"] == "F01")
    boot54 = datetime.fromisoformat(summary["summary"]["wall_time_alignment"]["log54"]["boot_local"])
    f01_start = (datetime.fromisoformat(f01["start_local"]) - boot54).total_seconds()
    f01_end = (datetime.fromisoformat(f01["end_local"]) - boot54).total_seconds()
    flights_by_log["log54"].append(("F01", f01_start, f01_end))
    for index, (start, end) in enumerate(arm_intervals(log55), start=2):
        flights_by_log["log55"].append((f"F{index:02d}", start, end))

    segment_rows: list[dict[str, Any]] = []
    for log_name, flights in flights_by_log.items():
        log = logs[log_name]
        for flight_id, start, end in flights:
            for seg_no, interval in enumerate(mode_intervals(log, start, end), start=1):
                segment_rows.append(segment_metrics(log, flight_id, f"{flight_id}-L{seg_no}", interval))

    flight_lookup = {row["flight_id"]: row for row in summary["flight_sessions"]}
    flight_rows: list[dict[str, Any]] = []
    for flight_id in sorted({s["flight_id"] for s in segment_rows}):
        segments = [s for s in segment_rows if s["flight_id"] == flight_id]
        airborne = [s for s in segments if not s["likely_ground_or_landing_segment"]]
        source = flight_lookup[flight_id]
        flight_rows.append({
            "flight_id": flight_id,
            "loiter_duration_s": sum(float(s["duration_s"]) for s in segments),
            "reported_xy_radius_m": source.get("loiter_xy_max_radius_m"),
            "airborne_xy_radius_m": max((s["xy_estimated_radius_m"] for s in airborne if s["xy_estimated_radius_m"] is not None), default=None),
            "throttle_in_deadband_pct": weighted_metric(airborne or segments, "throttle_in_deadband_pct"),
            "throttle_below_deadband_pct": weighted_metric(airborne or segments, "throttle_below_deadband_pct"),
            "pilot_xy_outside_pct": weighted_metric(airborne or segments, "pilot_xy_outside_pct"),
            "desired_climb_zero_pct": weighted_metric(airborne or segments, "desired_climb_zero_pct"),
            "vertical_tracking_error_p95_m": max((s["vertical_tracking_error_p95_m"] for s in airborne if s["vertical_tracking_error_p95_m"] is not None), default=None),
            "xy_target_endpoint_shift_m": max((s["xy_target_endpoint_shift_m"] for s in airborne if s["xy_target_endpoint_shift_m"] is not None), default=None),
            "optical_flow_quality_p05": min((s["optical_flow_quality_p05"] for s in airborne if s["optical_flow_quality_p05"] is not None), default=None),
            "ekf_aiding_restart_count": sum(int(s["ekf_aiding_restart_count"]) for s in airborne),
            "mag_yaw_realign_count": sum(int(s["mag_yaw_realign_count"]) for s in airborne),
            "ground_or_landing_segments": sum(bool(s["likely_ground_or_landing_segment"]) for s in segments),
        })

    correlations = {}
    for driver in ("throttle_in_deadband_pct", "throttle_below_deadband_pct", "pilot_xy_outside_pct", "xy_target_endpoint_shift_m", "optical_flow_quality_p05"):
        pairs = [(r[driver], r["airborne_xy_radius_m"]) for r in flight_rows if r.get(driver) is not None and r.get("airborne_xy_radius_m") is not None]
        correlations[driver] = {
            "n": len(pairs),
            "pearson_r": round_or_none(pearson([float(a) for a, _ in pairs], [float(b) for _, b in pairs]), 3),
        }

    for rows in (segment_rows, flight_rows):
        for row in rows:
            for key, value in list(row.items()):
                if isinstance(value, float):
                    row[key] = round_or_none(value, 3)

    payload = {
        "question": "LOITER定点偏差是否由油门进入THR_DZ死区造成",
        "parameters": {
            name: {
                **{k: round_or_none(v, 3) for k, v in throttle_calibration(log.params).items()},
                "THR_DZ": log.params.get("THR_DZ"),
                "RC3_MIN": log.params.get("RC3_MIN"),
                "RC3_MAX": log.params.get("RC3_MAX"),
                "RC3_DZ": log.params.get("RC3_DZ"),
                "MOT_THST_HOVER": log.params.get("MOT_THST_HOVER"),
                "PILOT_SPEED_UP": log.params.get("PILOT_SPEED_UP"),
                "PILOT_SPEED_DN": log.params.get("PILOT_SPEED_DN"),
            }
            for name, log in logs.items()
        },
        "segment_comparison": segment_rows,
        "flight_comparison": flight_rows,
        "descriptive_correlations": correlations,
        "interpretation_guardrails": [
            "THR_DZ只把中位油门映射为零目标爬升率；LOITER水平位置控制仍由独立的loiter_nav更新。",
            "相关系数仅为小样本描述，不构成因果证明。",
            "无GPS或外部定位真值；XY位移为飞控内部估计。",
            "被判为落地/降落段的区间不用于airborne_xy_radius_m比较。",
        ],
    }
    analysis_dir = root / "analysis"
    json_path = analysis_dir / "loiter_throttle_comparison.json"
    csv_path = analysis_dir / "loiter_throttle_flights.csv"
    segment_csv_path = analysis_dir / "loiter_throttle_segments.csv"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    for path, rows in ((csv_path, flight_rows), (segment_csv_path, segment_rows)):
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
