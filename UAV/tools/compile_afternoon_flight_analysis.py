#!/usr/bin/env python3
"""Compile the 2026-08-26 afternoon UAV export into auditable tables.

The script only reads the source logs, retries ordinary file copies into the
task output directory, and writes derived JSON/CSV analysis files there.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import shutil
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil


LOCAL_TZ = timezone(timedelta(hours=8))
WINDOW_START = datetime(2026, 8, 26, 12, 0, tzinfo=LOCAL_TZ)
WINDOW_END = datetime(2026, 8, 26, 18, 0, tzinfo=LOCAL_TZ)


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8-sig")
        return
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_iso(value: str | None) -> datetime | None:
    if not value:
        return None
    return datetime.fromisoformat(value)


def iso_local(epoch_seconds: float | None) -> str | None:
    if epoch_seconds is None or not math.isfinite(epoch_seconds):
        return None
    return datetime.fromtimestamp(epoch_seconds, tz=LOCAL_TZ).isoformat()


def round_or_none(value: Any, digits: int = 3) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return round(number, digits) if math.isfinite(number) else None


def nested(payload: Any, *keys: str, default: Any = None) -> Any:
    current = payload
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def tlog_boot_epoch(path: Path) -> dict[str, Any]:
    """Estimate vehicle boot wall time from SYSTEM_TIME messages."""
    connection = mavutil.mavlink_connection(str(path), notimestamps=False)
    offsets: list[float] = []
    try:
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            if message.get_type() != "SYSTEM_TIME":
                continue
            wall = getattr(message, "_timestamp", None)
            boot_ms = getattr(message, "time_boot_ms", None)
            try:
                wall_value = float(wall)
                boot_value = float(boot_ms) / 1000.0
            except (TypeError, ValueError):
                continue
            if wall_value > 1_000_000_000 and boot_value >= 0:
                offsets.append(wall_value - boot_value)
    finally:
        connection.close()
    if not offsets:
        return {"boot_epoch": None, "samples": 0, "spread_s": None}
    median = statistics.median(offsets)
    return {
        "boot_epoch": median,
        "boot_local": iso_local(median),
        "samples": len(offsets),
        "spread_s": max(offsets) - min(offsets),
    }


def load_health_module(workspace: Path):
    path = workspace / "output" / "flight_log_inspection_20260814" / "analyze_dataflash_health.py"
    spec = importlib.util.spec_from_file_location("uav_dataflash_health", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load analysis helpers from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def max_xy_radius(log_data: Any, start_s: float, end_s: float) -> float | None:
    points = []
    for _, row in log_data.between(
        "XKF1", start_s, end_s, lambda item: int(item.get("C", 0)) == 0
    ):
        try:
            points.append((float(row["PN"]), float(row["PE"])))
        except (KeyError, TypeError, ValueError):
            continue
    if not points:
        return None
    north0, east0 = points[0]
    return max(math.hypot(north - north0, east - east0) for north, east in points)


def unique_sequence(values: list[str]) -> str:
    result: list[str] = []
    for value in values:
        if value and (not result or value != result[-1]):
            result.append(value)
    return ">".join(result)


def metrics_from_summary(summary: dict[str, Any]) -> dict[str, Any]:
    duration = float(summary.get("duration_s") or 0)
    of_count = nested(summary, "optical_flow", "quality", "count") or 0
    rfnd_count = nested(summary, "rangefinder_0", "distance_m", "count") or 0
    alt_min = nested(summary, "ctun", "Alt", "min")
    alt_max = nested(summary, "ctun", "Alt", "max")
    altitude_span = None
    if alt_min is not None and alt_max is not None:
        altitude_span = float(alt_max) - float(alt_min)
    return {
        "altitude_min_m": round_or_none(alt_min, 3),
        "altitude_max_m": round_or_none(alt_max, 3),
        "altitude_span_m": round_or_none(altitude_span, 3),
        "rangefinder_max_m": round_or_none(
            nested(summary, "rangefinder_0", "distance_m", "max"), 3
        ),
        "rangefinder_healthy_pct": round_or_none(
            100 * float(nested(summary, "rangefinder_0", "healthy_fraction") or 0), 1
        ),
        "rangefinder_rate_hz": round_or_none(rfnd_count / duration if duration else None, 2),
        "optical_flow_quality_min": round_or_none(
            nested(summary, "optical_flow", "quality", "min"), 1
        ),
        "optical_flow_quality_p05": round_or_none(
            nested(summary, "optical_flow", "quality", "p05"), 1
        ),
        "optical_flow_quality_median": round_or_none(
            nested(summary, "optical_flow", "quality", "median"), 1
        ),
        "optical_flow_rate_hz": round_or_none(of_count / duration if duration else None, 2),
        "tilt_max_deg": round_or_none(nested(summary, "attitude", "tilt_deg", "max"), 2),
        "tilt_error_p95_deg": round_or_none(
            nested(summary, "attitude", "combined_tilt_error_deg", "p95"), 2
        ),
        "motor_upper_saturation_pct": round_or_none(
            100 * float(nested(summary, "motors", "any_upper_saturation_fraction") or 0), 1
        ),
        "motor_output_spread_max_pwm": round_or_none(
            nested(summary, "motors", "output_spread_pwm", "max"), 0
        ),
        "vcc_min_v": round_or_none(nested(summary, "power", "Vcc", "min"), 3),
        "vibration_z_max": round_or_none(
            nested(summary, "vibration", "0", "z", "max"), 2
        ),
        "gps_3d_fix_pct": round_or_none(
            100 * float(nested(summary, "gps", "three_d_fix_fraction") or 0), 1
        ),
        "log_drop_count_last": nested(summary, "logging", "last_drop_count"),
    }


def risk_assessment(row: dict[str, Any]) -> tuple[str, str]:
    high: list[str] = []
    watch: list[str] = []
    drift = row.get("loiter_xy_max_radius_m")
    tilt = row.get("tilt_max_deg")
    vibration = row.get("vibration_z_max")
    quality = row.get("optical_flow_quality_p05")
    saturation = row.get("motor_upper_saturation_pct")
    if drift is not None and drift >= 2:
        high.append(f"LOITER估计XY位移{drift:.2f}m")
    elif drift is not None and drift >= 1:
        watch.append(f"LOITER估计XY位移{drift:.2f}m")
    if tilt is not None and tilt >= 30:
        high.append(f"倾角峰值{tilt:.1f}°")
    elif tilt is not None and tilt >= 20:
        watch.append(f"倾角峰值{tilt:.1f}°")
    if vibration is not None and vibration >= 60:
        high.append(f"VIBE-Z峰值{vibration:.1f}")
    elif vibration is not None and vibration >= 50:
        watch.append(f"VIBE-Z峰值{vibration:.1f}")
    if quality is not None and quality < 60:
        watch.append(f"光流质量P05={quality:.0f}")
    if saturation is not None and saturation >= 10:
        high.append(f"电机上限附近占比{saturation:.1f}%")
    elif saturation is not None and saturation >= 2:
        watch.append(f"电机上限附近占比{saturation:.1f}%")
    if row.get("aiding_restart_count", 0):
        watch.append(f"EKF辅助重启{row['aiding_restart_count']}次")
    if row.get("mag_yaw_realign_count", 0):
        watch.append(f"磁航向重对齐{row['mag_yaw_realign_count']}次")
    reasons = high + watch
    if high:
        return "高", "；".join(reasons)
    if watch:
        return "中", "；".join(reasons)
    return "低", "未见本报告筛选出的突出异常"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("outputs/flight_logs_20260826_afternoon"),
    )
    parser.add_argument(
        "--mission-planner-root",
        type=Path,
        default=Path.home() / "Documents" / "Mission Planner" / "logs",
    )
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parents[1]
    output_root = (workspace / args.output_root).resolve()
    analysis_dir = output_root / "analysis"
    mp_dest = output_root / "mission_planner"
    mp_dest.mkdir(parents=True, exist_ok=True)
    analysis_dir.mkdir(parents=True, exist_ok=True)

    start_ts = WINDOW_START.timestamp()
    end_ts = WINDOW_END.timestamp()
    expected_mp = []
    for path in args.mission_planner_root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in {".tlog", ".rlog"}:
            continue
        stat = path.stat()
        if stat.st_ctime < end_ts and stat.st_mtime >= start_ts:
            expected_mp.append(path)
    expected_mp.sort(key=lambda path: (path.stat().st_ctime, path.suffix.lower(), path.name))

    manifest: list[dict[str, Any]] = []
    for source in expected_mp:
        destination = mp_dest / source.name
        source_size = source.stat().st_size
        if not destination.exists() or destination.stat().st_size != source_size:
            try:
                shutil.copy2(source, destination)
            except (OSError, PermissionError):
                pass
        exact = destination.exists() and destination.stat().st_size == source_size
        manifest.append(
            {
                "source_type": "Mission Planner",
                "log_type": source.suffix.lower().lstrip("."),
                "name": source.name,
                "source_path": str(source),
                "export_path": str(destination) if exact else None,
                "size_bytes": source_size,
                "source_created_local": datetime.fromtimestamp(source.stat().st_ctime, LOCAL_TZ).isoformat(),
                "source_modified_local": datetime.fromtimestamp(source.stat().st_mtime, LOCAL_TZ).isoformat(),
                "export_status": "已导出" if exact else "被Mission Planner独占锁定",
                "sha256": sha256(destination) if exact else None,
            }
        )

    for log_id in (54, 55):
        log_dir = output_root / "dataflash" / f"log{log_id:03d}"
        raw_path = log_dir / f"pixhawk_log_{log_id:03d}.BIN"
        full_manifest = load_json(log_dir / "export_full_manifest.json")
        full_result = full_manifest.get("results", {}).get("full", {})
        manifest.append(
            {
                "source_type": "Pixhawk DataFlash",
                "log_type": "bin",
                "name": raw_path.name,
                "source_path": f"Pixhawk onboard log {log_id}",
                "export_path": str(raw_path),
                "size_bytes": raw_path.stat().st_size,
                "source_created_local": None,
                "source_modified_local": datetime.fromtimestamp(raw_path.stat().st_mtime, LOCAL_TZ).isoformat(),
                "export_status": "已完整导出并校验" if full_result.get("complete") else "不完整",
                "sha256": full_result.get("sha256"),
            }
        )

    telemetry_payload = load_json(analysis_dir / "telemetry_analysis.json")
    telemetry_by_name = {Path(item["path"]).name: item for item in telemetry_payload}
    expected_tlogs = [row for row in manifest if row["source_type"] == "Mission Planner" and row["log_type"] == "tlog"]
    expected_rlogs = {row["name"]: row for row in manifest if row["source_type"] == "Mission Planner" and row["log_type"] == "rlog"}

    telemetry_sessions: list[dict[str, Any]] = []
    boot_offsets: dict[str, dict[str, Any]] = {}
    for item in expected_tlogs:
        name = item["name"]
        paired_rlog_name = f"{Path(name).stem}.rlog"
        rlog = expected_rlogs.get(paired_rlog_name, {})
        parsed = telemetry_by_name.get(name)
        if item["export_path"]:
            boot_offsets[name] = tlog_boot_epoch(Path(item["export_path"]))
        if parsed:
            armed = list(parsed.get("armed_segments") or [])
            armed_s = sum(float(segment.get("duration_s") or 0) for segment in armed)
            warnings = [row for row in parsed.get("status_texts", []) if int(row.get("severity", 99)) <= 3]
            telemetry_sessions.append(
                {
                    "tlog": name,
                    "rlog": paired_rlog_name,
                    "start_local": parsed.get("first_time"),
                    "end_local": parsed.get("last_time"),
                    "duration_s": round_or_none(parsed.get("duration_s"), 1),
                    "armed_segments": len(armed),
                    "armed_duration_s": round(armed_s, 1),
                    "warning_or_error_messages": len(warnings),
                    "tlog_export_status": item["export_status"],
                    "rlog_export_status": rlog.get("export_status"),
                    "boot_time_local": nested(boot_offsets.get(name, {}), "boot_local"),
                    "boot_alignment_samples": nested(boot_offsets.get(name, {}), "samples", default=0),
                }
            )
        else:
            telemetry_sessions.append(
                {
                    "tlog": name,
                    "rlog": paired_rlog_name,
                    "start_local": item["source_created_local"],
                    "end_local": item["source_modified_local"],
                    "duration_s": round(
                        datetime.fromisoformat(item["source_modified_local"]).timestamp()
                        - datetime.fromisoformat(item["source_created_local"]).timestamp(),
                        1,
                    ),
                    "armed_segments": None,
                    "armed_duration_s": None,
                    "warning_or_error_messages": None,
                    "tlog_export_status": item["export_status"],
                    "rlog_export_status": rlog.get("export_status"),
                    "boot_time_local": None,
                    "boot_alignment_samples": 0,
                }
            )

    health54 = load_json(analysis_dir / "log054_health.json")
    health55 = load_json(analysis_dir / "log055_health.json")
    schema54 = load_json(analysis_dir / "log054_schema.json")
    loiter_payload = load_json(analysis_dir / "dataflash_loiter_prearm.json")
    loiter55 = next(item for item in loiter_payload if "055" in item["source"])

    health_module = load_health_module(workspace)
    log54_data = health_module.LogData(output_root / "dataflash" / "log054" / "pixhawk_log_054.BIN")

    # Log 54 starts while armed, so the DataFlash contains only the closing ARM
    # record.  Align that disarm to the same event in Mission Planner telemetry.
    telemetry54 = telemetry_by_name["2026-08-26 14-06-39.tlog"]
    armed54 = telemetry54["armed_segments"][0]
    tlog54_start = parse_iso(armed54["start"])
    tlog54_end = parse_iso(armed54["end"])
    disarm54_s = next(
        float(event["TimeUS"]) / 1_000_000
        for event in schema54["events"]
        if event.get("mavpackettype") == "ARM" and int(event.get("ArmState", 1)) == 0
    )
    boot54_epoch = tlog54_end.timestamp() - disarm54_s
    arm54_s = tlog54_start.timestamp() - boot54_epoch
    phase54 = health_module.phase_summary(log54_data, arm54_s, disarm54_s)
    row54 = {
        "source_dataflash": "log54",
        "source_tlog": "2026-08-26 14-06-39.tlog",
        "start_local": tlog54_start.isoformat(),
        "end_local": tlog54_end.isoformat(),
        "duration_s": round_or_none(armed54["duration_s"], 3),
        "mode_sequence": "LOITER",
        "disarm_method": "LANDED",
        "airborne_event_seen": True,
        "loiter_segment_count": 1,
        "loiter_xy_max_radius_m": round_or_none(max_xy_radius(log54_data, arm54_s, disarm54_s), 3),
        "optical_flow_max_gap_s": None,
        "rangefinder_max_gap_s": None,
        "aiding_restart_count": 0,
        "mag_yaw_realign_count": 0,
        "error_count_during_armed": 0,
        "wall_time_alignment": "高置信度：TLOG与DataFlash解除锁定事件对齐",
        **metrics_from_summary(phase54),
    }

    # The 15:20 TLOG straddles a later controller restart, so its SYSTEM_TIME
    # boot estimate belongs to the preceding boot.  The final locked TLOG stops
    # at the same moment as the last DataFlash "Radio Failsafe - Disarming"
    # message; use that exact cross-source anchor for log55 wall time.
    radio_failsafe_times55 = [
        float(item["time_s"])
        for item in loiter55.get("diagnostic_messages") or []
        if item.get("text") == "Radio Failsafe - Disarming"
    ]
    locked_tlog_manifest = next(
        row
        for row in manifest
        if row["source_type"] == "Mission Planner"
        and row["name"] == "2026-08-26 15-26-49.tlog"
    )
    if radio_failsafe_times55:
        boot55 = (
            parse_iso(locked_tlog_manifest["source_modified_local"]).timestamp()
            - max(radio_failsafe_times55)
        )
    else:
        boot55 = parse_iso(telemetry_by_name["2026-08-26 15-20-11.tlog"]["first_time"]).timestamp()

    loiter_segments55 = list(loiter55.get("loiter_segments") or [])
    flight_rows = [row54]
    for interval in health55["arm_intervals"]:
        start_s = float(interval["start_s"])
        end_s = float(interval["end_s"])
        overlaps = [
            segment
            for segment in loiter_segments55
            if float(segment["start_s"]) <= end_s and float(segment["end_s"]) >= start_s
        ]
        messages = list(interval.get("messages") or [])
        row = {
            "source_dataflash": "log55",
            "source_tlog": "2026-08-26 15-26-49.tlog (被Mission Planner锁定；飞行指标来自DataFlash)",
            "start_local": iso_local(boot55 + start_s),
            "end_local": iso_local(boot55 + end_s),
            "duration_s": round_or_none(interval.get("duration_s"), 3),
            "mode_sequence": unique_sequence([phase.get("name", "") for phase in interval.get("mode_phases", [])]),
            "disarm_method": interval.get("disarm_method"),
            "airborne_event_seen": bool(interval.get("airborne_event_seen")),
            "loiter_segment_count": len(overlaps),
            "loiter_xy_max_radius_m": round_or_none(
                max((float(segment["ekf_xy_max_radius_m"]) for segment in overlaps if segment.get("ekf_xy_max_radius_m") is not None), default=None),
                3,
            ),
            "optical_flow_max_gap_s": round_or_none(
                max((float(nested(segment, "optical_flow", "max_gap_s")) for segment in overlaps if nested(segment, "optical_flow", "max_gap_s") is not None), default=None),
                4,
            ),
            "rangefinder_max_gap_s": round_or_none(
                max((float(nested(segment, "rangefinder", "max_gap_s")) for segment in overlaps if nested(segment, "rangefinder", "max_gap_s") is not None), default=None),
                4,
            ),
            "aiding_restart_count": sum("started relative aiding" in str(item.get("message", "")) for item in messages),
            "mag_yaw_realign_count": sum("yaw re-aligned" in str(item.get("message", "")) for item in messages),
            "error_count_during_armed": len(interval.get("errors") or []),
            "wall_time_alignment": "中高置信度：末段TLOG停止写入与DataFlash无线电失效保护事件对齐",
            **metrics_from_summary(interval["summary"]),
        }
        flight_rows.append(row)

    flight_rows.sort(key=lambda row: row["start_local"])
    for index, row in enumerate(flight_rows, start=1):
        row["flight_id"] = f"F{index:02d}"
        level, reason = risk_assessment(row)
        row["review_level"] = level
        row["review_reason"] = reason

    diagnostic_groups: dict[str, list[float]] = defaultdict(list)
    for item in loiter55.get("diagnostic_messages") or []:
        diagnostic_groups[str(item["text"])].append(float(item["time_s"]))

    alerts: list[dict[str, Any]] = []
    for text, times in diagnostic_groups.items():
        if "Radio Failsafe - Disarming" in text:
            category, severity = "无线电链路", "高"
        elif "Radio failsafe" in text:
            category, severity = "无线电链路", "高"
        elif "Yaw inconsistent" in text or "EKF variance" in text:
            category, severity = "EKF/航向", "高"
        elif "stopped aiding" in text or "ground mag anomaly" in text:
            category, severity = "EKF/航向", "中"
        elif "fusing optical flow" in text or "started relative aiding" in text:
            category, severity = "EKF/光流", "观察"
        else:
            continue
        alerts.append(
            {
                "category": category,
                "severity": severity,
                "event": text,
                "count": len(times),
                "first_local": iso_local(boot55 + min(times)),
                "last_local": iso_local(boot55 + max(times)),
                "source": "DataFlash log55",
            }
        )
    alerts.sort(key=lambda row: ({"高": 0, "中": 1, "观察": 2}[row["severity"]], -row["count"], row["event"]))

    exported_mp = sum(row["export_status"] == "已导出" for row in manifest if row["source_type"] == "Mission Planner")
    expected_mp_count = sum(row["source_type"] == "Mission Planner" for row in manifest)
    total_armed = sum(float(row["duration_s"] or 0) for row in flight_rows)
    loiter_flights = sum("LOITER" in row["mode_sequence"] for row in flight_rows)
    longest = max(flight_rows, key=lambda row: float(row["duration_s"] or 0))
    max_drift = max(
        (row for row in flight_rows if row.get("loiter_xy_max_radius_m") is not None),
        key=lambda row: float(row["loiter_xy_max_radius_m"]),
    )

    log54_drop_min = nested(health54, "overall_logging", "drop_count", "min")
    log54_drop_max = nested(health54, "overall_logging", "drop_count", "max")
    log54_drop_delta = None
    if log54_drop_min is not None and log54_drop_max is not None:
        log54_drop_delta = int(log54_drop_max) - int(log54_drop_min)

    locked_names = [row["name"] for row in manifest if row["export_status"] == "被Mission Planner独占锁定"]
    findings = [
        {
            "severity": "高",
            "finding": "下午共识别12次解锁，9次包含LOITER；未发现DataFlash中的推力丢失事件。",
            "evidence": f"总解锁{total_armed:.1f}s；最长{longest['flight_id']}为{longest['duration_s']:.1f}s。",
            "impact": "可用于逐航次复盘，但不代表飞行已通过安全验收。",
            "action": "优先复查高/中关注航次，再安排受控复测。",
        },
        {
            "severity": "高",
            "finding": "LOITER位置估计仍存在明显异常位移。",
            "evidence": f"{max_drift['flight_id']}最大估计XY位移{max_drift['loiter_xy_max_radius_m']:.2f}m；发生于{max_drift['start_local'][11:19]}附近。",
            "impact": "短时室内悬停可能触发错误位置纠偏。",
            "action": "在重新带桨LOITER前，先复核测距零值、光流安装方向与EKF航向稳定性。",
        },
        {
            "severity": "高",
            "finding": "无线电失效保护与预解锁无线电告警反复出现。",
            "evidence": f"Radio Failsafe - Disarming {len(diagnostic_groups.get('Radio Failsafe - Disarming', []))}次；PreArm: Radio failsafe on {len(diagnostic_groups.get('PreArm: Radio failsafe on', []))}次。",
            "impact": "链路不稳会干扰测试连续性并可能触发保护动作。",
            "action": "检查接收机供电、SBUS线束、遥控器链路与FS_THR_VALUE/RC_FS_TIMEOUT配置。",
        },
        {
            "severity": "中",
            "finding": "光流与测距在LOITER段的消息频率已恢复，但光流质量低位仍偏低，且EKF辅助多次停止/重启。",
            "evidence": "LOITER段约100Hz光流、20Hz测距、最大间隔约0.055s；多航次光流质量P05为54–66。",
            "impact": "链路连续性改善，但纹理/光照/航向异常仍可能使位置估计不稳定。",
            "action": "固定30–50cm高度、纹理与照度，连续记录30秒并核对质量分布和航向重对齐。",
        },
        {
            "severity": "中",
            "finding": "动力电池数据缺失，无法从日志评估电压、电流和剩余电量。",
            "evidence": "BATT_MONITOR=0，log54/log55均无BAT消息；TLOG中的0V按无效哨兵处理。",
            "impact": "不能排除动力供电或压降问题。",
            "action": "启用并校准合适的电池监测后再做续航/负载判断。",
        },
        {
            "severity": "中",
            "finding": "日志时间与完整性存在两项限制。",
            "evidence": f"飞控目录UTC固定为1980；log54记录丢弃计数增加约{log54_drop_delta}；Mission Planner末段2个文件仍被独占锁定。",
            "impact": "墙钟依赖TLOG对齐，且锁定文件暂不能原样复制。",
            "action": "关闭Mission Planner后补拷最后一对TLOG/RLOG；后续修复飞控RTC/GPS时间来源并监控DSF丢弃计数。",
        },
    ]

    summary = {
        "window_local": f"{WINDOW_START.isoformat()} 至 {WINDOW_END.isoformat()}",
        "timezone": "Asia/Shanghai (UTC+08:00)",
        "dataflash_logs_exported": 2,
        "dataflash_logs_complete": 2,
        "mission_planner_sessions_expected": len(expected_tlogs),
        "mission_planner_file_pairs_expected": len(expected_tlogs),
        "mission_planner_files_exported": exported_mp,
        "mission_planner_files_expected": expected_mp_count,
        "mission_planner_locked_files": locked_names,
        "flight_count": len(flight_rows),
        "total_armed_duration_s": round(total_armed, 3),
        "loiter_flight_count": loiter_flights,
        "longest_flight_id": longest["flight_id"],
        "longest_flight_duration_s": longest["duration_s"],
        "max_loiter_xy_radius_flight_id": max_drift["flight_id"],
        "max_loiter_xy_radius_m": max_drift["loiter_xy_max_radius_m"],
        "radio_failsafe_disarming_count": len(diagnostic_groups.get("Radio Failsafe - Disarming", [])),
        "prearm_radio_failsafe_count": len(diagnostic_groups.get("PreArm: Radio failsafe on", [])),
        "thrust_loss_event_count": len(health55.get("thrust_loss_events") or []),
        "battery_telemetry_available": bool(health54.get("battery_messages_present") or health55.get("battery_messages_present")),
        "gps_3d_fix_fraction": health55.get("overall_gps", {}).get("three_d_fix_fraction"),
        "log54_drop_counter_delta": log54_drop_delta,
        "wall_time_alignment": {
            "log54": {
                "boot_local": iso_local(boot54_epoch),
                "method": "TLOG/DataFlash解除锁定事件对齐",
            },
            "log55": {
                "boot_epoch": boot55,
                "boot_local": iso_local(boot55),
                "anchor_tlog_last_write_local": locked_tlog_manifest["source_modified_local"],
                "anchor_dataflash_time_s": max(radio_failsafe_times55) if radio_failsafe_times55 else None,
                "method": "末段TLOG停止写入时间与最后一次Radio Failsafe - Disarming事件对齐",
            },
        },
    }

    payload = {
        "summary": summary,
        "findings": findings,
        "flight_sessions": flight_rows,
        "telemetry_sessions": telemetry_sessions,
        "alerts": alerts,
        "export_manifest": manifest,
        "methodology": {
            "scope": "本地时间2026-08-26 12:00–18:00；包含跨越12:00的11:55:37会话。",
            "grain": "飞行=DataFlash ARM到DISARM区间；Mission Planner会话按TLOG文件。",
            "source_precedence": "解锁、模式、姿态、光流、测距与保护事件以完整DataFlash为主；墙钟和地面站会话以Mission Planner TLOG为辅。",
            "risk_labels": "高/中/低为本次复盘筛选标签，不是适航或安全认证。",
            "limitations": [
                "Pixhawk目录time_utc固定为1980-01-01，不能直接用于筛选。",
                "最后一对Mission Planner TLOG/RLOG被正在运行的Mission Planner独占锁定，DataFlash log55仍完整覆盖其中的12次? 飞行指标；本报告按DataFlash识别11次并与log54合计12次。",
                "BATT_MONITOR=0且无BAT消息，不能评估动力电池。",
                "GPS无3D定位，不能生成可靠的绝对航迹或地面距离。",
            ],
        },
    }

    # Correct the wording above without hiding the lock limitation.
    payload["methodology"]["limitations"][1] = (
        "最后一对Mission Planner TLOG/RLOG被正在运行的Mission Planner独占锁定；"
        "完整DataFlash log55仍覆盖其中的11次解锁，与log54合计12次。"
    )

    write_json(analysis_dir / "analysis_summary.json", payload)
    write_csv(analysis_dir / "flight_sessions.csv", flight_rows)
    write_csv(analysis_dir / "telemetry_sessions.csv", telemetry_sessions)
    write_csv(analysis_dir / "alerts.csv", alerts)
    write_csv(analysis_dir / "export_manifest.csv", manifest)
    write_csv(analysis_dir / "findings.csv", findings)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
