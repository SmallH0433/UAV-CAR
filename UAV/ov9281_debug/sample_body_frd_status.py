#!/usr/bin/env python3
"""Sample OV9281 HTTP status locally and report orientation-only BODY_FRD."""

from __future__ import annotations

import json
import statistics
import time
import urllib.request


rows = []
for _ in range(30):
    with urllib.request.urlopen("http://127.0.0.1:8765/api/status", timeout=1) as response:
        status = json.load(response)
    if status.get("found"):
        rows.append({
            "forward": -float(status["x_m"]),
            "right": -float(status["y_m"]),
            "down": float(status["z_m"]),
            "distance": float(status["distance_m"]),
            "margin": float(status["decision_margin"]),
            "reprojection": float(status["reprojection_error_px"]),
        })
    time.sleep(0.1)

def mean(name: str):
    return statistics.fmean(row[name] for row in rows) if rows else None

result = {
    "valid_samples": len(rows),
    "body_forward_mean_m": mean("forward"),
    "body_right_mean_m": mean("right"),
    "body_down_mean_m": mean("down"),
    "distance_mean_m": mean("distance"),
    "decision_margin_mean": mean("margin"),
    "reprojection_error_mean_px": mean("reprojection"),
    "mapping": "BODY_FRD=(-camera_x,-camera_y,+camera_z)",
    "translation_applied": False,
    "mavlink_transmitted": 0,
}
print(json.dumps(result, ensure_ascii=False, indent=2))
raise SystemExit(0 if len(rows) >= 20 else 2)
