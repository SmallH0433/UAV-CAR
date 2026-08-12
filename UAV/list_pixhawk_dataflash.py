#!/usr/bin/env python3
"""List ArduPilot onboard DataFlash logs over MAVLink without changing vehicle state."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone

from pymavlink import mavutil


def iso_utc(value: int) -> str | None:
    if not value:
        return None
    try:
        return datetime.fromtimestamp(value, tz=timezone.utc).astimezone().isoformat()
    except (OverflowError, OSError, ValueError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False)
    heartbeat = link.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        raise SystemExit(f"No flight-controller heartbeat on {args.port}")
    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()

    entries: dict[int, dict] = {}
    expected = None
    deadline = time.monotonic() + args.timeout
    last_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_request >= 2.0:
            link.mav.log_request_list_send(target_system, target_component, 0, 0xFFFF)
            last_request = now
        message = link.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if message is None:
            continue
        expected = int(message.num_logs)
        entry_id = int(message.id)
        entries[entry_id] = {
            "id": entry_id,
            "num_logs": expected,
            "last_log_num": int(message.last_log_num),
            "time_utc": int(message.time_utc),
            "time_local": iso_utc(int(message.time_utc)),
            "size_bytes": int(message.size),
        }
        if expected == 0 or len(entries) >= expected:
            break

    payload = {
        "port": args.port,
        "baud": args.baud,
        "target_system": target_system,
        "target_component": target_component,
        "expected_logs": expected,
        "received_logs": len(entries),
        "logs": [entries[key] for key in sorted(entries)],
    }
    encoded = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(encoded + "\n")
    print(encoded)
    link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
