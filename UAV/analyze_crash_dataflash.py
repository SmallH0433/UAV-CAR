#!/usr/bin/env python3
"""Extract a focused timeline and statistics from an ArduPilot crash DataFlash log."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path

from pymavlink import DFReader


SELECTED = {
    "MODE", "ARM", "ERR", "MSG", "RCIN", "RCOU", "CTUN", "RFND", "OF",
    "ATT", "XKF1", "XKF4", "XKF5", "LDET", "VIBE", "BAT", "POS",
}


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def stats(values):
    values = [value for value in (finite(item) for item in values) if value is not None]
    if not values:
        return None
    return {
        "count": len(values), "min": min(values), "mean": statistics.fmean(values),
        "median": statistics.median(values), "max": max(values),
    }


def nearest(rows, when, tolerance=0.25):
    if not rows:
        return None
    times = [item[0] for item in rows]
    index = bisect.bisect_left(times, when)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index:
        candidates.append(rows[index - 1])
    best = min(candidates, key=lambda item: abs(item[0] - when))
    return best[1] if abs(best[0] - when) <= tolerance else None


def values_between(rows, start, end, field):
    return [item[1].get(field) for item in rows if start <= item[0] <= end]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    reader = DFReader.DFReader_binary(str(args.log))
    rows = {name: [] for name in SELECTED}
    while True:
        message = reader.recv_msg()
        if message is None:
            break
        name = message.get_type()
        if name not in rows:
            continue
        timestamp = finite(getattr(message, "TimeUS", None))
        if timestamp is None:
            continue
        rows[name].append((timestamp / 1_000_000.0, message.to_dict()))

    modes = [
        {"time_s": time_s, "mode": int(item["Mode"]), "reason": int(item["Rsn"])}
        for time_s, item in rows["MODE"]
    ]
    loiter_events = [item for item in modes if item["mode"] == 5]
    if not loiter_events:
        raise SystemExit("No LOITER mode event found")
    loiter_time = loiter_events[-1]["time_s"]

    crash_messages = [
        {"time_s": time_s, "message": str(item.get("Message", ""))}
        for time_s, item in rows["MSG"] if "Crash:" in str(item.get("Message", ""))
    ]
    if not crash_messages:
        raise SystemExit("No crash message found")
    crash_time = crash_messages[-1]["time_s"]
    start = loiter_time - 3.0
    end = crash_time + 2.0

    timeline = []
    step = 0.25
    tick = start
    while tick <= end + 1e-9:
        rcin = nearest(rows["RCIN"], tick)
        rcou = nearest(rows["RCOU"], tick)
        ctun = nearest(rows["CTUN"], tick)
        rfnd = nearest(rows["RFND"], tick)
        xkf1 = nearest([item for item in rows["XKF1"] if int(item[1].get("C", 0)) == 0], tick)
        xkf4 = nearest([item for item in rows["XKF4"] if int(item[1].get("C", 0)) == 0], tick)
        xkf5 = nearest([item for item in rows["XKF5"] if int(item[1].get("C", 0)) == 0], tick)
        attitude = nearest(rows["ATT"], tick)
        optical_flow = nearest(rows["OF"], tick)
        land = nearest(rows["LDET"], tick)
        battery = nearest(rows["BAT"], tick, tolerance=0.6)
        vibe = nearest([item for item in rows["VIBE"] if int(item[1].get("IMU", 0)) == 0], tick)
        timeline.append({
            "time_s": round(tick, 3), "from_loiter_s": round(tick - loiter_time, 3),
            "rc1": None if not rcin else rcin.get("C1"),
            "rc2": None if not rcin else rcin.get("C2"),
            "rc3": None if not rcin else rcin.get("C3"),
            "rc4": None if not rcin else rcin.get("C4"),
            "rc5": None if not rcin else rcin.get("C5"),
            "rc7": None if not rcin else rcin.get("C7"),
            "motor_pwm": None if not rcou else [rcou.get(f"C{i}") for i in range(1, 5)],
            "range_m": None if not rfnd else rfnd.get("Dist"),
            "alt_m": None if not ctun else ctun.get("Alt"),
            "surface_alt_m": None if not ctun else ctun.get("SAlt"),
            "climb_rate_mps": None if not ctun else ctun.get("CRt"),
            "throttle_out": None if not ctun else ctun.get("ThO"),
            "north_m": None if not xkf1 else xkf1.get("PN"),
            "east_m": None if not xkf1 else xkf1.get("PE"),
            "down_m": None if not xkf1 else xkf1.get("PD"),
            "vn_mps": None if not xkf1 else xkf1.get("VN"),
            "ve_mps": None if not xkf1 else xkf1.get("VE"),
            "vd_mps": None if not xkf1 else xkf1.get("VD"),
            "roll_deg": None if not attitude else attitude.get("Roll"),
            "pitch_deg": None if not attitude else attitude.get("Pitch"),
            "desired_roll_deg": None if not attitude else attitude.get("DesRoll"),
            "desired_pitch_deg": None if not attitude else attitude.get("DesPitch"),
            "flow_quality": None if not optical_flow else optical_flow.get("Qual"),
            "ekf_velocity_variance": None if not xkf4 else xkf4.get("SV"),
            "ekf_position_variance": None if not xkf4 else xkf4.get("SP"),
            "ekf_compass_variance": None if not xkf4 else xkf4.get("SM"),
            "hagl_m": None if not xkf5 else xkf5.get("HAGL"),
            "land_flags": None if not land else land.get("Flags"),
            "battery_v": None if not battery else battery.get("Volt"),
            "battery_a": None if not battery else battery.get("Curr"),
            "vibration": None if not vibe else [vibe.get("VibeX"), vibe.get("VibeY"), vibe.get("VibeZ")],
        })
        tick += step

    before = (start, loiter_time)
    after = (loiter_time, crash_time)
    xkf_primary = [item for item in rows["XKF1"] if int(item[1].get("C", 0)) == 0]
    xkf_at_switch = nearest(xkf_primary, loiter_time)
    pn0 = finite(xkf_at_switch.get("PN")) if xkf_at_switch else None
    pe0 = finite(xkf_at_switch.get("PE")) if xkf_at_switch else None
    displacement = []
    if pn0 is not None and pe0 is not None:
        for time_s, item in xkf_primary:
            if loiter_time <= time_s <= crash_time:
                pn, pe = finite(item.get("PN")), finite(item.get("PE"))
                if pn is not None and pe is not None:
                    displacement.append(math.hypot(pn - pn0, pe - pe0))

    range_after = [
        (time_s, finite(item.get("Dist"))) for time_s, item in rows["RFND"]
        if loiter_time <= time_s <= crash_time and finite(item.get("Dist")) is not None
    ]
    touchdown = next((time_s for time_s, value in range_after if value <= 0.22), None)
    bounce_max = None
    if touchdown is not None:
        bounce_values = [value for time_s, value in range_after if touchdown < time_s <= crash_time]
        bounce_max = max(bounce_values) if bounce_values else None

    def phase_summary(bounds):
        phase_start, phase_end = bounds
        return {
            "range_m": stats(values_between(rows["RFND"], phase_start, phase_end, "Dist")),
            "rc_throttle_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C3")),
            "rc_roll_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C1")),
            "rc_pitch_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C2")),
            "rc_yaw_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C4")),
            "rc_mode_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C5")),
            "rc_follow_pwm": stats(values_between(rows["RCIN"], phase_start, phase_end, "C7")),
            "flow_quality": stats(values_between(rows["OF"], phase_start, phase_end, "Qual")),
            "roll_deg": stats(values_between(rows["ATT"], phase_start, phase_end, "Roll")),
            "pitch_deg": stats(values_between(rows["ATT"], phase_start, phase_end, "Pitch")),
            "climb_rate_mps": stats(values_between(rows["CTUN"], phase_start, phase_end, "CRt")),
            "throttle_out": stats(values_between(rows["CTUN"], phase_start, phase_end, "ThO")),
            "battery_v": stats(values_between(rows["BAT"], phase_start, phase_end, "Volt")),
            "battery_a": stats(values_between(rows["BAT"], phase_start, phase_end, "Curr")),
            "motor_1_pwm": stats(values_between(rows["RCOU"], phase_start, phase_end, "C1")),
            "motor_2_pwm": stats(values_between(rows["RCOU"], phase_start, phase_end, "C2")),
            "motor_3_pwm": stats(values_between(rows["RCOU"], phase_start, phase_end, "C3")),
            "motor_4_pwm": stats(values_between(rows["RCOU"], phase_start, phase_end, "C4")),
            "ekf_velocity_variance": stats(values_between(
                [item for item in rows["XKF4"] if int(item[1].get("C", 0)) == 0],
                phase_start, phase_end, "SV")),
            "ekf_position_variance": stats(values_between(
                [item for item in rows["XKF4"] if int(item[1].get("C", 0)) == 0],
                phase_start, phase_end, "SP")),
            "ekf_compass_variance": stats(values_between(
                [item for item in rows["XKF4"] if int(item[1].get("C", 0)) == 0],
                phase_start, phase_end, "SM")),
        }

    payload = {
        "log": str(args.log.resolve()),
        "loiter_switch_time_s": loiter_time,
        "crash_time_s": crash_time,
        "loiter_to_crash_s": crash_time - loiter_time,
        "crash_messages": crash_messages,
        "mode_events": modes,
        "arm_events": [{"time_s": t, **item} for t, item in rows["ARM"]],
        "error_events": [{"time_s": t, **item} for t, item in rows["ERR"]],
        "messages_near_event": [
            {"time_s": t, "message": str(item.get("Message", ""))}
            for t, item in rows["MSG"] if start <= t <= end + 12.0
        ],
        "before_loiter_3s": phase_summary(before),
        "loiter_until_crash": phase_summary(after),
        "horizontal_displacement_after_loiter_m": stats(displacement),
        "touchdown_range_threshold_time_s": touchdown,
        "touchdown_after_loiter_s": None if touchdown is None else touchdown - loiter_time,
        "post_touchdown_max_range_m": bounce_max,
        "timeline_4hz": timeline,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in (
        "loiter_switch_time_s", "crash_time_s", "loiter_to_crash_s",
        "crash_messages", "mode_events", "arm_events", "error_events",
        "messages_near_event", "before_loiter_3s", "loiter_until_crash",
        "horizontal_displacement_after_loiter_m", "touchdown_range_threshold_time_s",
        "touchdown_after_loiter_s", "post_touchdown_max_range_m",
    )}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
