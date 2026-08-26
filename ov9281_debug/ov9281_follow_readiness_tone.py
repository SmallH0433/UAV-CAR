#!/usr/bin/env python3
"""OV9281 dual-tag readiness tone monitor.

This process reads the existing OV9281 HTTP status and Pixhawk telemetry.  Its
only MAVLink transmissions are a telemetry stream request and PLAY_TUNE.  It
cannot arm, change mode, write parameters, land, or send motion setpoints.

The single-C prompt means that all configured non-control prerequisites have
remained valid and RC7 is still low.  After that prompt, an RC7 low-to-high
transition makes the process exit so the serial port can be handed to the
single control writer.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
import urllib.request
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v20 import common


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "imx296_debug"))

from follow_readiness import ReadinessInputs, evaluate_readiness  # noqa: E402
from follow_tone_policy import OBSERVE_READY_TUNE  # noqa: E402


REAL_FC_SYSTEM_ID = 1
REAL_FC_COMPONENT_ID = 1


@dataclass
class TelemetryState:
    armed: bool | None = None
    mode: str | None = None
    heartbeat_at_s: float | None = None
    rc_pwm: int | None = None
    rc_at_s: float | None = None
    ekf_flags: int | None = None
    ekf_at_s: float | None = None
    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None
    battery_at_s: float | None = None
    range_m: float | None = None
    range_at_s: float | None = None
    flow_quality: int | None = None
    flow_at_s: float | None = None
    attitude_at_s: float | None = None
    origin_valid: bool = False


@dataclass(frozen=True)
class VisionSnapshot:
    api_ok: bool
    acquired: bool
    age_s: float | None
    consecutive_good_frames: int
    tag_id: int | None
    tag_size_m: float | None
    role: str | None
    decision_margin: float | None
    hamming: int | None
    reprojection_error_px: float | None
    distance_m: float | None
    blockers: tuple[str, ...]


@dataclass
class VisionAcquisition:
    config: dict[str, Any]
    last_sequence: int | None = None
    last_good_at_s: float | None = None
    consecutive_good_frames: int = 0
    latest_values: dict[str, Any] = field(default_factory=dict)
    latest_blockers: tuple[str, ...] = ("APRILTAG_NOT_ACQUIRED",)
    api_ok: bool = False

    def _quality_blockers(self, status: dict[str, Any]) -> tuple[str, ...]:
        blockers: list[str] = []
        if status.get("mode") != "apriltag":
            blockers.append("VISION_NOT_IN_APRILTAG_MODE")
        if status.get("found") is not True:
            blockers.append("APRILTAG_NOT_VISIBLE")

        frame_age_ms = _optional_float(status.get("frame_age_ms"))
        if frame_age_ms is None or frame_age_ms > float(self.config["maximum_frame_age_ms"]):
            blockers.append("VISION_FRAME_STALE")

        tag_id = _optional_int(status.get("tag_id"))
        accepted_tags = self.config["accepted_tags"]
        tag_config = accepted_tags.get(str(tag_id)) if tag_id is not None else None
        if tag_config is None:
            blockers.append("APRILTAG_ID_NOT_CONFIGURED")
        else:
            actual_size = _optional_float(status.get("tag_size_m"))
            expected_size = float(tag_config["size_m"])
            tolerance = float(tag_config.get("size_tolerance_m", expected_size * 0.05))
            if actual_size is None or abs(actual_size - expected_size) > tolerance:
                blockers.append("APRILTAG_SIZE_MISMATCH")
            if str(status.get("role", "")) != str(tag_config["role"]):
                blockers.append("APRILTAG_ROLE_MISMATCH")
            area = _optional_float(status.get("area_px2"))
            if area is None or area < float(tag_config["minimum_area_px2"]):
                blockers.append("APRILTAG_AREA_TOO_SMALL")

        minimum_margin = float(self.config["minimum_decision_margin"])
        maximum_hamming = int(self.config["maximum_hamming"])
        maximum_reprojection = float(self.config["maximum_reprojection_error_px"])
        if tag_config is not None:
            minimum_margin = float(
                tag_config.get("minimum_decision_margin", minimum_margin)
            )
            maximum_hamming = int(
                tag_config.get("maximum_hamming", maximum_hamming)
            )
            maximum_reprojection = float(
                tag_config.get(
                    "maximum_reprojection_error_px",
                    maximum_reprojection,
                )
            )

        margin = _optional_float(status.get("decision_margin"))
        if margin is None or margin < minimum_margin:
            blockers.append("APRILTAG_DECISION_MARGIN_LOW")
        hamming = _optional_int(status.get("hamming"))
        if hamming is None or hamming > maximum_hamming:
            blockers.append("APRILTAG_HAMMING_HIGH")
        reprojection = _optional_float(status.get("reprojection_error_px"))
        if (
            reprojection is None
            or reprojection > maximum_reprojection
        ):
            blockers.append("APRILTAG_REPROJECTION_ERROR_HIGH")
        distance = _optional_float(status.get("distance_m"))
        if (
            distance is None
            or not float(self.config["minimum_pose_distance_m"])
            <= distance
            <= float(self.config["maximum_pose_distance_m"])
        ):
            blockers.append("APRILTAG_POSE_DISTANCE_INVALID")
        for field_name in ("x_m", "y_m", "z_m"):
            if _optional_float(status.get(field_name)) is None:
                blockers.append("APRILTAG_POSE_INCOMPLETE")
                break
        return tuple(dict.fromkeys(blockers))

    def update(self, status: dict[str, Any], now_s: float) -> VisionSnapshot:
        self.api_ok = True
        sequence = _optional_int(status.get("analysis_sequence"))
        if sequence is None:
            self.consecutive_good_frames = 0
            self.latest_blockers = ("VISION_SEQUENCE_MISSING",)
        elif sequence != self.last_sequence:
            self.last_sequence = sequence
            self.latest_blockers = self._quality_blockers(status)
            self.latest_values = {
                "tag_id": _optional_int(status.get("tag_id")),
                "tag_size_m": _optional_float(status.get("tag_size_m")),
                "role": status.get("role"),
                "decision_margin": _optional_float(status.get("decision_margin")),
                "hamming": _optional_int(status.get("hamming")),
                "reprojection_error_px": _optional_float(
                    status.get("reprojection_error_px")
                ),
                "distance_m": _optional_float(status.get("distance_m")),
            }
            if self.latest_blockers:
                self.consecutive_good_frames = 0
            else:
                self.consecutive_good_frames += 1
                self.last_good_at_s = now_s
        return self.snapshot(now_s)

    def unavailable(self, now_s: float, reason: str) -> VisionSnapshot:
        self.api_ok = False
        self.latest_blockers = (reason,)
        return self.snapshot(now_s)

    def snapshot(self, now_s: float) -> VisionSnapshot:
        age_s = monotonic_age(now_s, self.last_good_at_s)
        acquired = bool(
            self.consecutive_good_frames >= int(self.config["acquire_count"])
            and age_s is not None
            and age_s <= float(self.config["target_timeout_s"])
        )
        return VisionSnapshot(
            api_ok=self.api_ok,
            acquired=acquired,
            age_s=age_s,
            consecutive_good_frames=self.consecutive_good_frames,
            tag_id=self.latest_values.get("tag_id"),
            tag_size_m=self.latest_values.get("tag_size_m"),
            role=self.latest_values.get("role"),
            decision_margin=self.latest_values.get("decision_margin"),
            hamming=self.latest_values.get("hamming"),
            reprojection_error_px=self.latest_values.get("reprojection_error_px"),
            distance_m=self.latest_values.get("distance_m"),
            blockers=self.latest_blockers,
        )


@dataclass
class ReadyToneLatch:
    rearm_after_not_ready_s: float
    announced: bool = False
    not_ready_since_s: float | None = None

    def update(self, ready: bool, now_s: float) -> bool:
        if ready:
            self.not_ready_since_s = None
            if not self.announced:
                self.announced = True
                return True
            return False
        if self.not_ready_since_s is None:
            self.not_ready_since_s = now_s
        elif now_s - self.not_ready_since_s >= self.rearm_after_not_ready_s:
            self.announced = False
        return False


def _optional_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if result == result and abs(result) != float("inf") else None


def _optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def monotonic_age(now_s: float, timestamp_s: float | None) -> float | None:
    return None if timestamp_s is None else max(0.0, now_s - timestamp_s)


def read_vision_status(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=1.0) as response:
        return json.load(response)


def validate_config(config: dict[str, Any]) -> None:
    if config.get("scope") != "ov9281_dual_tag_follow_readiness_tone_only":
        raise RuntimeError("unexpected readiness-tone scope")
    operator = config.get("operator_start_policy", {})
    if operator.get("manual_start_only") is not True:
        raise RuntimeError("readiness tone must remain manual-start only")
    if operator.get("autostart_forbidden") is not True:
        raise RuntimeError("readiness tone must remain autostart-forbidden")
    if operator.get("exit_on_rc7_high_after_ready") is not True:
        raise RuntimeError("readiness tone must release serial after RC7 goes high")
    safety = config.get("safety", {})
    for name in ("tone_only", "telemetry_stream_request", "play_tune"):
        if safety.get(name) is not True:
            raise RuntimeError(f"safety.{name} must be true")
    for name in (
        "parameter_write",
        "mode_change",
        "movement_setpoint",
        "arm_command",
        "takeoff_command",
        "land_command",
        "motor_command",
    ):
        if safety.get(name) is not False:
            raise RuntimeError(f"safety.{name} must be false")
    accepted_tags = config.get("vision", {}).get("accepted_tags", {})
    if set(accepted_tags) != {"0", "1"}:
        raise RuntimeError("dual-tag readiness requires exactly tag IDs 0 and 1")
    if float(accepted_tags["0"]["size_m"]) != 0.100:
        raise RuntimeError("outer tag must be 0.100 m")
    if float(accepted_tags["1"]["size_m"]) != 0.020:
        raise RuntimeError("inner tag must be 0.020 m")


def ensure_conflicting_services_inactive(config: dict[str, Any]) -> None:
    for service in config["operator_start_policy"].get("conflicting_services", []):
        result = subprocess.run(
            ["systemctl", "--user", "is-active", "--quiet", str(service)],
            check=False,
            timeout=3.0,
            capture_output=True,
        )
        if result.returncode == 0:
            raise RuntimeError(f"conflicting MAVLink writer is active: {service}")
        if result.returncode not in (3, 4):
            detail = result.stderr.decode(errors="replace").strip()
            raise RuntimeError(
                f"cannot prove conflicting service {service} is inactive: "
                f"{detail or result.returncode}"
            )


def is_real_fc_heartbeat(message: Any) -> bool:
    return bool(
        message is not None
        and message.get_type() == "HEARTBEAT"
        and message.get_srcSystem() == REAL_FC_SYSTEM_ID
        and message.get_srcComponent() == REAL_FC_COMPONENT_ID
    )


def ingest_message(
    message: Any,
    now_s: float,
    state: TelemetryState,
    rc_channel: int,
) -> None:
    if message is None or message.get_srcSystem() != REAL_FC_SYSTEM_ID:
        return
    name = message.get_type()
    if name == "HEARTBEAT" and message.get_srcComponent() == REAL_FC_COMPONENT_ID:
        state.armed = bool(
            int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        state.mode = mavutil.mode_string_v10(message).upper()
        state.heartbeat_at_s = now_s
    elif name == "RC_CHANNELS":
        state.rc_pwm = _optional_int(getattr(message, f"chan{rc_channel}_raw", None))
        state.rc_at_s = now_s
    elif name == "EKF_STATUS_REPORT":
        state.ekf_flags = int(message.flags)
        state.ekf_at_s = now_s
    elif name == "SYS_STATUS":
        voltage_mv = int(message.voltage_battery)
        remaining = int(message.battery_remaining)
        state.battery_voltage_v = None if voltage_mv in (-1, 65535) else voltage_mv / 1000.0
        state.battery_remaining_pct = None if remaining < 0 else remaining
        state.battery_at_s = now_s
    elif name == "DISTANCE_SENSOR" and int(message.orientation) == 25:
        state.range_m = float(message.current_distance) / 100.0
        state.range_at_s = now_s
    elif name in {"OPTICAL_FLOW", "OPTICAL_FLOW_RAD"}:
        state.flow_quality = int(message.quality)
        state.flow_at_s = now_s
    elif name == "ATTITUDE" and message.get_srcComponent() == REAL_FC_COMPONENT_ID:
        state.attitude_at_s = now_s
    elif name in {"GPS_GLOBAL_ORIGIN", "GLOBAL_POSITION_INT"}:
        latitude = int(getattr(message, "latitude", getattr(message, "lat", 0)))
        longitude = int(getattr(message, "longitude", getattr(message, "lon", 0)))
        if latitude != 0 and longitude != 0:
            state.origin_valid = True


def evaluate_operational_readiness(
    config: dict[str, Any],
    state: TelemetryState,
    vision: VisionSnapshot,
    *,
    now_s: float,
    rc7_low_cycle_seen: bool,
) -> tuple[bool, tuple[str, ...], tuple[str, ...]]:
    readiness_cfg = config["readiness"]
    telemetry_timeout_s = float(readiness_cfg["telemetry_timeout_s"])
    result = evaluate_readiness(
        ReadinessInputs(
            heartbeat_age_s=monotonic_age(now_s, state.heartbeat_at_s),
            armed=state.armed,
            mode=state.mode,
            rc7_pwm=state.rc_pwm,
            rc_age_s=monotonic_age(now_s, state.rc_at_s),
            ekf_flags=state.ekf_flags,
            ekf_age_s=monotonic_age(now_s, state.ekf_at_s),
            battery_voltage_v=state.battery_voltage_v,
            battery_remaining_pct=state.battery_remaining_pct,
            battery_age_s=monotonic_age(now_s, state.battery_at_s),
            range_m=state.range_m,
            range_age_s=monotonic_age(now_s, state.range_at_s),
            flow_quality=state.flow_quality,
            flow_age_s=monotonic_age(now_s, state.flow_at_s),
            origin_valid=state.origin_valid,
            target_acquired=vision.acquired,
            target_age_s=vision.age_s,
            camera_ok=vision.api_ok,
        ),
        minimum_voltage_v=float(readiness_cfg["minimum_voltage_v"]),
        minimum_remaining_pct=int(readiness_cfg["minimum_remaining_pct"]),
        battery_telemetry_required=bool(readiness_cfg["battery_telemetry_required"]),
        minimum_range_m=float(readiness_cfg["minimum_height_m"]),
        maximum_range_m=float(readiness_cfg["maximum_height_m"]),
        minimum_flow_quality=int(readiness_cfg["minimum_flow_quality"]),
        telemetry_timeout_s=telemetry_timeout_s,
        target_timeout_s=float(config["vision"]["target_timeout_s"]),
        allowed_modes=tuple(readiness_cfg["allowed_entry_modes"]),
    )
    blockers = list(result.blockers)
    if bool(readiness_cfg["require_armed_for_ready_tone"]) and state.armed is not True:
        blockers.append("VEHICLE_NOT_ARMED")
    if not rc7_low_cycle_seen:
        blockers.append("RC7_LOW_CYCLE_REQUIRED")
    rc_cfg = config["rc_authorization"]
    rc_age_s = monotonic_age(now_s, state.rc_at_s)
    if (
        state.rc_pwm is not None
        and rc_age_s is not None
        and rc_age_s <= telemetry_timeout_s
        and state.rc_pwm > int(rc_cfg["disable_pwm_max"])
    ):
        blockers.append("RC7_NOT_LOW_FOR_READY_TONE")
    if bool(readiness_cfg["attitude_required"]):
        attitude_age_s = monotonic_age(now_s, state.attitude_at_s)
        if attitude_age_s is None or attitude_age_s > telemetry_timeout_s:
            blockers.append("ATTITUDE_STALE")
    if not vision.acquired:
        blockers.extend(vision.blockers)
    blockers = list(dict.fromkeys(blockers))
    return not blockers, tuple(blockers), result.warnings


def send_tune(link: Any, tune: bytes) -> None:
    link.mav.send(
        common.MAVLink_play_tune_message(
            REAL_FC_SYSTEM_ID,
            REAL_FC_COMPONENT_ID,
            tune,
            b"",
        )
    )


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def make_record(
    config: dict[str, Any],
    state: TelemetryState,
    vision: VisionSnapshot,
    *,
    ready: bool,
    blockers: tuple[str, ...],
    warnings: tuple[str, ...],
    rc7_low_cycle_seen: bool,
    tone_announced: bool,
    tone_packets: int,
    process_state: str,
) -> dict[str, Any]:
    return {
        "timestamp_unix": time.time(),
        "state": process_state,
        "scope": config["scope"],
        "ready_for_follow_switch": ready,
        "ready_tone_meaning": "SINGLE_C_ALL_PREREQUISITES_READY_RC7_MAY_BE_ENABLED",
        "blockers": list(blockers),
        "warnings": list(warnings),
        "armed": state.armed,
        "mode": state.mode,
        "rc7_pwm": state.rc_pwm,
        "rc7_low_cycle_seen": rc7_low_cycle_seen,
        "range_m": state.range_m,
        "flow_quality": state.flow_quality,
        "ekf_flags": state.ekf_flags,
        "origin_valid": state.origin_valid,
        "vision": {
            "api_ok": vision.api_ok,
            "acquired": vision.acquired,
            "age_s": vision.age_s,
            "consecutive_good_frames": vision.consecutive_good_frames,
            "tag_id": vision.tag_id,
            "tag_size_m": vision.tag_size_m,
            "role": vision.role,
            "decision_margin": vision.decision_margin,
            "hamming": vision.hamming,
            "reprojection_error_px": vision.reprojection_error_px,
            "distance_m": vision.distance_m,
            "quality_blockers": list(vision.blockers),
        },
        "ready_tone_announced": tone_announced,
        "play_tune_packets_transmitted": tone_packets,
        "telemetry_stream_request_packets_transmitted": 1,
        "mavlink_movement_packets_transmitted": 0,
        "parameter_writes": 0,
        "mode_commands": 0,
        "arm_commands": 0,
        "takeoff_commands": 0,
        "land_commands": 0,
        "motor_commands": 0,
        "exit_on_rc7_high_after_ready": True,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OV9281 dual-tag follow readiness tone")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration-s", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    validate_config(config)
    ensure_conflicting_services_inactive(config)

    telemetry_cfg = config["telemetry"]
    link = mavutil.mavlink_connection(
        telemetry_cfg["device"],
        baud=int(telemetry_cfg["baud"]),
        autoreconnect=False,
        source_system=int(telemetry_cfg["source_system"]),
        source_component=int(telemetry_cfg["source_component"]),
    )
    heartbeat = link.wait_heartbeat(timeout=float(telemetry_cfg["heartbeat_timeout_s"]))
    if not is_real_fc_heartbeat(heartbeat):
        link.close()
        raise RuntimeError("real flight-controller heartbeat not received")
    link.mav.request_data_stream_send(
        REAL_FC_SYSTEM_ID,
        REAL_FC_COMPONENT_ID,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        int(telemetry_cfg["stream_rate_hz"]),
        1,
    )

    stopped = False

    def stop_handler(*_: Any) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    vision_gate = VisionAcquisition(config["vision"])
    state = TelemetryState()
    ingest_message(heartbeat, time.monotonic(), state, int(config["rc_authorization"]["channel"]))
    tone_latch = ReadyToneLatch(float(config["tone"]["rearm_after_not_ready_s"]))
    counters: Counter[str] = Counter()
    rc7_low_cycle_seen = False
    started_s = time.monotonic()
    next_vision_s = started_s
    last_status_s = 0.0
    last_log_s = 0.0
    latest_vision = vision_gate.snapshot(started_s)
    status_path = Path(config["output"]["status"])
    log_path = Path(config["output"]["log"])
    log_path.parent.mkdir(parents=True, exist_ok=True)
    last_record: dict[str, Any] | None = None
    exit_reason = "STOP_REQUESTED"

    try:
        with log_path.open("a", encoding="utf-8") as log_file:
            while not stopped:
                now_s = time.monotonic()
                if args.duration_s is not None and now_s - started_s >= args.duration_s:
                    exit_reason = "DURATION_COMPLETE"
                    break
                for _ in range(300):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    ingest_message(
                        message,
                        now_s,
                        state,
                        int(config["rc_authorization"]["channel"]),
                    )
                    counters[f"rx_{message.get_type()}"] += 1

                rc_age_s = monotonic_age(now_s, state.rc_at_s)
                if (
                    state.rc_pwm is not None
                    and rc_age_s is not None
                    and rc_age_s <= float(config["readiness"]["telemetry_timeout_s"])
                    and state.rc_pwm <= int(config["rc_authorization"]["disable_pwm_max"])
                ):
                    rc7_low_cycle_seen = True

                if now_s >= next_vision_s:
                    try:
                        latest_vision = vision_gate.update(
                            read_vision_status(config["vision"]["status_url"]),
                            now_s,
                        )
                    except Exception:
                        latest_vision = vision_gate.unavailable(now_s, "VISION_API_UNAVAILABLE")
                    next_vision_s = now_s + 1.0 / float(config["vision"]["poll_hz"])

                ready, blockers, warnings = evaluate_operational_readiness(
                    config,
                    state,
                    latest_vision,
                    now_s=now_s,
                    rc7_low_cycle_seen=rc7_low_cycle_seen,
                )
                tone_sent_this_cycle = tone_latch.update(ready, now_s)
                if tone_sent_this_cycle:
                    send_tune(link, OBSERVE_READY_TUNE)
                    counters["tone_OBSERVE_READY"] += 1

                if ready and tone_latch.announced:
                    process_state = "READY_TONE_SENT"
                elif tone_latch.announced:
                    process_state = "READINESS_LOST_AFTER_TONE"
                else:
                    process_state = "WAITING_FOR_READINESS"
                record = make_record(
                    config,
                    state,
                    latest_vision,
                    ready=ready,
                    blockers=blockers,
                    warnings=warnings,
                    rc7_low_cycle_seen=rc7_low_cycle_seen,
                    tone_announced=tone_latch.announced,
                    tone_packets=counters["tone_OBSERVE_READY"],
                    process_state=process_state,
                )
                last_record = record
                if (
                    now_s - last_status_s >= 1.0 / float(config["output"]["status_rate_hz"])
                    or tone_sent_this_cycle
                ):
                    write_json(status_path, record)
                    last_status_s = now_s
                if now_s - last_log_s >= 1.0 or tone_sent_this_cycle:
                    log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                    log_file.flush()
                    last_log_s = now_s

                rc_high = bool(
                    state.rc_pwm is not None
                    and rc_age_s is not None
                    and rc_age_s <= float(config["readiness"]["telemetry_timeout_s"])
                    and state.rc_pwm >= int(config["rc_authorization"]["enable_pwm_min"])
                )
                if tone_latch.announced and rc7_low_cycle_seen and rc_high:
                    exit_reason = "RC7_ENABLED_AFTER_READY_TONE"
                    record["state"] = exit_reason
                    record["ready_for_follow_switch"] = False
                    write_json(status_path, record)
                    log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                    log_file.flush()
                    last_record = record
                    break
                time.sleep(0.01)
    finally:
        link.close()

    summary = {
        "scope": config["scope"],
        "duration_s": time.monotonic() - started_s,
        "exit_reason": exit_reason,
        "counters": dict(counters),
        "last_record": last_record,
        "play_tune_packets_transmitted": counters["tone_OBSERVE_READY"],
        "telemetry_stream_request_packets_transmitted": 1,
        "mavlink_movement_packets_transmitted": 0,
        "parameter_writes": 0,
        "mode_commands": 0,
        "arm_commands": 0,
        "takeoff_commands": 0,
        "land_commands": 0,
        "motor_commands": 0,
    }
    write_json(status_path.with_suffix(".summary.json"), summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
