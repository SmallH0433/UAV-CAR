#!/usr/bin/env python3
"""Find the newest valid GPS position in Mission Planner telemetry logs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    matches = []
    for path in args.root.rglob("*.tlog"):
        try:
            link = mavutil.mavlink_connection(str(path))
            newest = None
            while True:
                message = link.recv_match(type="GPS_RAW_INT", blocking=False)
                if message is None:
                    break
                if (
                    int(message.fix_type) >= 3
                    and abs(int(message.lat)) > 1_000_000
                    and abs(int(message.lon)) > 1_000_000
                ):
                    newest = {
                        "path": str(path),
                        "fix_type": int(message.fix_type),
                        "satellites": int(message.satellites_visible),
                        "latitude_deg": int(message.lat) / 1e7,
                        "longitude_deg": int(message.lon) / 1e7,
                        "altitude_msl_m": int(message.alt) / 1000.0,
                    }
            if newest is not None:
                newest["mtime_ns"] = path.stat().st_mtime_ns
                matches.append(newest)
        except Exception:
            continue
    matches.sort(key=lambda item: item["mtime_ns"], reverse=True)
    for match in matches:
        match.pop("mtime_ns", None)
    print(json.dumps({"valid_logs": len(matches), "newest": matches[:10]}, indent=2))
    return 0 if matches else 2


if __name__ == "__main__":
    raise SystemExit(main())
