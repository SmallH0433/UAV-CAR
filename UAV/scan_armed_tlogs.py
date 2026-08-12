#!/usr/bin/env python3
"""Quickly identify real-flight-controller armed intervals in many tlogs."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil


def stamp(value):
    return None if value is None else datetime.fromtimestamp(value).astimezone().isoformat()


def scan(path: Path) -> dict:
    link = mavutil.mavlink_connection(str(path), notimestamps=False)
    first = last = None
    armed = False
    armed_start = None
    intervals = []
    modes_while_armed = set()
    heartbeat_count = 0
    fc_heartbeat_count = 0
    status_during_or_near_armed = []
    last_status = []
    while True:
        message = link.recv_match(blocking=False)
        if message is None:
            break
        ts = getattr(message, "_timestamp", None)
        if ts is not None:
            ts = float(ts)
            first = ts if first is None else min(first, ts)
            last = ts if last is None else max(last, ts)
        name = message.get_type()
        if name == "STATUSTEXT" and message.get_srcSystem() == 1:
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            item = {"time": stamp(ts), "text": str(text).rstrip("\x00")}
            last_status.append(item)
            last_status = last_status[-8:]
            if armed:
                status_during_or_near_armed.append(item)
        if name != "HEARTBEAT":
            continue
        heartbeat_count += 1
        if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            continue
        fc_heartbeat_count += 1
        now_armed = bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mavutil.mode_string_v10(message).upper()
        if now_armed:
            modes_while_armed.add(mode)
        if now_armed and not armed:
            armed_start = ts
            status_during_or_near_armed.extend(last_status)
        elif armed and not now_armed:
            intervals.append({"start": stamp(armed_start), "end": stamp(ts),
                              "duration_s": None if armed_start is None or ts is None else ts-armed_start})
            armed_start = None
        armed = now_armed
    if armed:
        intervals.append({"start": stamp(armed_start), "end": stamp(last),
                          "duration_s": None if armed_start is None or last is None else last-armed_start,
                          "log_ended_armed": True})
    return {
        "path": str(path.resolve()), "size_bytes": path.stat().st_size,
        "first_time": stamp(first), "last_time": stamp(last),
        "fc_heartbeat_count": fc_heartbeat_count,
        "armed_intervals": intervals,
        "armed_total_s": sum(float(item.get("duration_s") or 0.0) for item in intervals),
        "modes_while_armed": sorted(modes_while_armed),
        "status_near_or_during_armed": status_during_or_near_armed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    files = sorted(args.root.rglob("*.tlog"), key=lambda p: p.stat().st_mtime)
    results = []
    errors = []
    for path in files:
        try:
            if path.stat().st_size <= 0:
                continue
            results.append(scan(path))
        except (OSError, ValueError) as exc:
            errors.append({"path": str(path.resolve()), "error": f"{type(exc).__name__}: {exc}"})
    payload = {
        "root": str(args.root.resolve()),
        "files_scanned": len(results),
        "armed_files": [result for result in results if result["armed_intervals"]],
        "all_files": results,
        "errors": errors,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"files_scanned": len(results), "armed_files": payload["armed_files"],
                      "errors": errors},
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
