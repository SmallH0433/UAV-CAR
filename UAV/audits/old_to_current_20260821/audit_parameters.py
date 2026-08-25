#!/usr/bin/env python3
"""Compare the old Pixhawk, migration-final snapshot, and current Log 41 parameters."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

from pymavlink import DFReader


ROOT = Path(r"D:\Codex\UAV")
OUT = ROOT / "audits" / "old_to_current_20260821"
OLD = ROOT / "pixhawk_backups" / "pixhawk_parameters_20260818_190345" / "pixhawk_full.param"
MIGRATION_FINAL = ROOT / "qav280_migration" / "final" / "pixhawk_parameters_20260818_193506" / "pixhawk_full.param"
SELECTED = ROOT / "qav280_migration" / "stage2_selected_migration.json"
CURRENT_LOG = ROOT / "flight_logs" / "20260821_171322_log041" / "pixhawk_log_041.BIN"


def load_param(path: Path) -> dict[str, float]:
    result: dict[str, float] = {}
    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        raw = raw.strip()
        if not raw or raw.startswith("#"):
            continue
        name, value = raw.split(",", 1)
        result[name.strip()] = float(value.strip())
    return result


def load_dataflash_params(path: Path) -> dict[str, float]:
    reader = DFReader.DFReader_binary(str(path))
    result: dict[str, float] = {}
    while True:
        message = reader.recv_msg()
        if message is None:
            break
        if message.get_type() == "PARM":
            result[str(message.Name).rstrip("\x00")] = float(message.Value)
    return result


def equal(left: float | None, right: float | None) -> bool:
    if left is None or right is None:
        return left is right
    return math.isclose(left, right, rel_tol=1e-6, abs_tol=1e-5)


def category(name: str) -> str:
    if re.match(r"^(INS_|COMPASS_|BARO|AHRS_TRIM_|SERVO|MOT_|ATC_|PSC_|BATT_)", name):
        return "hardware_specific"
    if re.match(r"^RC\d+_(MIN|MAX|TRIM|DZ|REVERSED)$", name):
        return "hardware_specific"
    if name.startswith(("EK3_", "AHRS_", "FLOW_", "RNGFND")):
        return "navigation_sensor"
    if name.startswith(("SERIAL", "MAV")):
        return "serial_telemetry"
    if name.startswith(("RC", "FLTMODE", "RCMAP")):
        return "rc_mode_mapping"
    if name.startswith(("PLND_", "LAND_", "PILOT_THR", "DISARM")):
        return "landing"
    if name.startswith(("FS_", "ARMING", "BRD_SAFETY")):
        return "safety"
    if name.startswith("LOG_"):
        return "logging"
    return "other"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    old = load_param(OLD)
    migration_final = load_param(MIGRATION_FINAL)
    current = load_dataflash_params(CURRENT_LOG)
    selected_payload = json.loads(SELECTED.read_text(encoding="utf-8"))
    selected = {name: float(value) for name, value in selected_payload["parameters"].items()}

    names = sorted(set(old) | set(migration_final) | set(current))
    rows = []
    for name in names:
        old_value = old.get(name)
        migration_value = migration_final.get(name)
        current_value = current.get(name)
        rows.append(
            {
                "name": name,
                "category": category(name),
                "selected_migration": name in selected,
                "selected_value": selected.get(name),
                "old_value": old_value,
                "migration_final_value": migration_value,
                "current_log41_value": current_value,
                "old_vs_current": "same" if equal(old_value, current_value) else "different",
                "migration_final_vs_current": "same" if equal(migration_value, current_value) else "different",
                "selected_vs_current": (
                    "not_selected"
                    if name not in selected
                    else ("same" if equal(selected[name], current_value) else "different")
                ),
            }
        )

    with (OUT / "parameter_differences.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    selected_mismatches = [row for row in rows if row["selected_vs_current"] == "different"]
    final_changes = [
        row
        for row in rows
        if row["migration_final_value"] is not None
        and row["current_log41_value"] is not None
        and row["migration_final_vs_current"] == "different"
    ]
    category_counts: dict[str, int] = {}
    for row in final_changes:
        category_counts[row["category"]] = category_counts.get(row["category"], 0) + 1

    payload = {
        "sources": {
            "old_pixhawk": str(OLD),
            "migration_final": str(MIGRATION_FINAL),
            "selected_migration": str(SELECTED),
            "current_snapshot": str(CURRENT_LOG),
            "current_snapshot_note": "DataFlash Log 41 PARM messages; live COM10 read was blocked by Mission Planner.",
        },
        "counts": {
            "old": len(old),
            "migration_final": len(migration_final),
            "current_log41": len(current),
            "selected": len(selected),
            "selected_mismatches": len(selected_mismatches),
            "post_migration_common_parameter_changes": len(final_changes),
        },
        "post_migration_changes_by_category": category_counts,
        "selected_mismatches": selected_mismatches,
        "post_migration_changes": final_changes,
    }
    (OUT / "parameter_audit.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(payload["counts"], ensure_ascii=False))
    print("SELECTED_MISMATCHES")
    for row in selected_mismatches:
        print(row["name"], row["selected_value"], row["current_log41_value"], row["category"])


if __name__ == "__main__":
    main()
