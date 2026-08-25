#!/usr/bin/env python3
"""Props-removed OV9281 -> GUIDED motor-mix integration test.

This is deliberately not a flight runtime.  It may request GUIDED and transmit
bounded horizontal velocity setpoints to a real flight controller, so it is
guarded by a dedicated props-removed configuration.  Boot startup is accepted
only when that configuration contains an explicit props-removed acknowledgement.
It never arms, takes off, lands, writes parameters, or sends direct motor/servo
commands.  The pilot must arm manually and can revoke authorization with CH7 or
the normal flight-mode switch.

Audible states:

* one low note: AprilTag observation is acquired while CH7 is authorized;
* rising C-E-G: GUIDED is confirmed and the flight controller has echoed the
  transmitted local-NED velocity target;
* falling G-E-C: a previously confirmed session is confirmed out of GUIDED or
  disarmed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import signal
import subprocess
import sys
import time
import urllib.request
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from pymavlink.dialects.v20 import common


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "imx296_debug"))

from follow_controller import HorizontalFollowController  # noqa: E402
from follow_mode_manager import (  # noqa: E402
    FollowModeManager,
    ModeManagerInputs,
    PilotStickOverrideDetector,
)
from follow_readiness import ReadinessInputs, evaluate_readiness  # noqa: E402
from follow_tone_policy import FollowTonePolicy, TUNES  # noqa: E402
from mavlink_guided_velocity import GuidedVelocitySetpoint, make_message  # noqa: E402
from rc_follow_gate import RcFollowGate  # noqa: E402
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement  # noqa: E402


REAL_FC_SYSTEM_ID = 1
REAL_FC_COMPONENT_ID = 1
POSITION_TARGET_LOCAL_NED_MESSAGE_ID = 85


def install_pymavlink_instance_guard() -> None:
    """Work around pymavlink 2.4.x instance-cache reuse after a fast reopen."""
    current = mavutil.add_message
    if getattr(current, "_ov9281_instance_guard", False):
        return

    def guarded_add_message(messages: dict[str, Any], mtype: str, message: Any) -> None:
        instance_field = getattr(message, "_instance_field", None)
        instance_value = (
            getattr(message, instance_field, None) if instance_field is not None else None
        )
        existing = messages.get(mtype)
        if (
            instance_field is not None
            and instance_value is not None
            and existing is not None
            and getattr(existing, "_instances", None) is None
        ):
            messages.pop(mtype, None)
        current(messages, mtype, message)

    guarded_add_message._ov9281_instance_guard = True  # type: ignore[attr-defined]
    mavutil.add_message = guarded_add_message


@dataclass
class TelemetryState:
    armed: bool | None = None
    mode: str | None = None
    heartbeat_at_s: float | None = None
    heartbeat_id: int = 0
    yaw_rad: float | None = None
    attitude_at_s: float | None = None
    time_boot_ms: int | None = None
    rc7_pwm: int | None = None
    rc_at_s: float | None = None
    pilot_override: bool = False
    ekf_flags: int | None = None
    ekf_at_s: float | None = None
    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None
    battery_at_s: float | None = None
    range_m: float | None = None
    range_at_s: float | None = None
    flow_quality: int | None = None
    flow_at_s: float | None = None
    origin_valid: bool = False
    origin_latitude_deg: float | None = None
    origin_longitude_deg: float | None = None
    target_echo_at_s: float | None = None
    target_echo_velocity_ned_mps: tuple[float, float, float] | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Props-removed OV9281 GUIDED motor-mix test")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--status", type=Path)
    return parser.parse_args()


def monotonic_age(now_s: float, timestamp_s: float | None) -> float | None:
    return None if timestamp_s is None else max(0.0, now_s - timestamp_s)


def transform_camera_to_body(
    position: tuple[float, float, float], extrinsics: dict[str, Any]
) -> tuple[float, float, float]:
    rotation = extrinsics["rotation_camera_optical_to_body_frd"]
    translation = extrinsics["translation_m"]
    return tuple(
        sum(float(rotation[row][column]) * position[column] for column in range(3))
        + float(translation[row])
        for row in range(3)
    )


def body_to_ned(forward: float, right: float, yaw_rad: float) -> tuple[float, float]:
    return (
        math.cos(yaw_rad) * forward - math.sin(yaw_rad) * right,
        math.sin(yaw_rad) * forward + math.cos(yaw_rad) * right,
    )


def observation_ready_without_ch7(
    *,
    armed: bool | None,
    current_mode: str | None,
    allowed_entry_modes: tuple[str, ...],
    non_ch7_prerequisites_ok: bool,
) -> bool:
    """Return readiness for the single-C prompt; CH7 is intentionally absent."""
    return bool(
        armed
        and current_mode in allowed_entry_modes
        and non_ch7_prerequisites_ok
    )


def is_armed(message: Any) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def is_real_fc_heartbeat(message: Any) -> bool:
    return (
        message is not None
        and message.get_type() == "HEARTBEAT"
        and message.get_srcSystem() == REAL_FC_SYSTEM_ID
        and message.get_srcComponent() == REAL_FC_COMPONENT_ID
    )


def read_vision_status(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=1.0) as response:
        return json.load(response)


def validate_props_off_config(
    config: dict[str, Any], requested_duration_s: float | None
) -> float | None:
    expected_scope = "props_removed_handheld_guided_motor_mix_test_only"
    if config.get("scope") != expected_scope:
        raise RuntimeError(f"props-off runtime requires scope={expected_scope}")
    if config.get("flight_use_approved") is not False:
        raise RuntimeError("props-off runtime requires flight_use_approved=false")

    operator = config.get("operator_start_policy", {})
    manual_start_only = operator.get("manual_start_only")
    autostart_forbidden = operator.get("autostart_forbidden")
    manual_policy = manual_start_only is True and autostart_forbidden is True
    props_removed_autostart_policy = (
        manual_start_only is False
        and autostart_forbidden is False
        and operator.get("props_removed_autostart_acknowledged") is True
        and operator.get("autostart_scope") == "props_removed_only"
    )
    if not (manual_policy or props_removed_autostart_policy):
        raise RuntimeError(
            "props-off runtime requires either manual-only startup or an explicit "
            "props-removed autostart acknowledgement"
        )
    if not operator.get("conflicting_services"):
        raise RuntimeError("props-off runtime requires an explicit conflicting-service list")
    safety = config.get("safety", {})
    required_true = ("props_removed_test_only", "control_enabled", "mavlink_transmit", "mode_change")
    required_false = (
        "parameter_write",
        "arm_command",
        "takeoff_command",
        "land_command",
        "motor_command",
    )
    for name in required_true:
        if safety.get(name) is not True:
            raise RuntimeError(f"props-off runtime requires safety.{name}=true")
    for name in required_false:
        if safety.get(name) is not False:
            raise RuntimeError(f"props-off runtime requires safety.{name}=false")

    pixhawk = config.get("pixhawk", {})
    if pixhawk.get("receive_only") is not False:
        raise RuntimeError("props-off runtime requires pixhawk.receive_only=false")

    rc_authorization = config.get("rc_authorization", {})
    if rc_authorization.get("low_at_start_required") is not True:
        raise RuntimeError("props-off runtime requires an RC7 low cycle at startup")

    extrinsics = config.get("camera_to_body", {})
    if (
        not extrinsics.get("enabled")
        or not extrinsics.get("rotation_enabled_for_props_off_test")
        or extrinsics.get("translation_m") != [0.0, 0.0, 0.0]
        or extrinsics.get("approved_scope") != "props_removed_test_only"
    ):
        raise RuntimeError("camera transform is not approved for the props-removed test")

    controller = config.get("controller", {})
    max_speed = float(controller.get("max_speed_mps", math.inf))
    max_accel = float(controller.get("max_accel_mps2", math.inf))
    max_feedforward = float(controller.get("max_feedforward_mps", math.inf))
    rate_hz = float(controller.get("command_rate_hz", 0.0))
    if not 0.0 < max_speed <= 0.10:
        raise RuntimeError("props-off max_speed_mps must be in (0, 0.10]")
    if not 0.0 < max_accel <= 0.20:
        raise RuntimeError("props-off max_accel_mps2 must be in (0, 0.20]")
    if not 0.0 < max_feedforward <= 0.10:
        raise RuntimeError("props-off max_feedforward_mps must be in (0, 0.10]")
    if not 5.0 <= rate_hz <= 10.0:
        raise RuntimeError("props-off command_rate_hz must be between 5 and 10 Hz")

    configured_max_duration_s = operator.get("max_duration_s")
    max_duration_s = (
        None
        if configured_max_duration_s is None
        else float(configured_max_duration_s)
    )
    if max_duration_s is not None and max_duration_s <= 0.0:
        raise RuntimeError("max_duration_s must be positive or null")
    duration_s = (
        max_duration_s if requested_duration_s is None else float(requested_duration_s)
    )
    if duration_s is not None and duration_s <= 0.0:
        raise RuntimeError("duration_s must be positive")
    if (
        duration_s is not None
        and max_duration_s is not None
        and duration_s > max_duration_s
    ):
        raise RuntimeError("requested duration exceeds configured maximum")
    return duration_s


def ensure_conflicting_services_inactive(config: dict[str, Any]) -> None:
    for service in config.get("operator_start_policy", {}).get("conflicting_services", []):
        try:
            result = subprocess.run(
                ["systemctl", "--user", "is-active", "--quiet", str(service)],
                check=False,
                timeout=3.0,
                capture_output=True,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError(f"cannot verify conflicting service {service}: {exc}") from exc
        if result.returncode == 0:
            raise RuntimeError(f"conflicting MAVLink/camera service is active: {service}")
        if result.returncode not in (3, 4):
            detail = result.stderr.decode(errors="replace").strip()
            raise RuntimeError(
                f"cannot prove conflicting service {service} is inactive: {detail or result.returncode}"
            )


def send_tune(link: Any, tune: bytes) -> None:
    link.mav.send(
        common.MAVLink_play_tune_message(
            REAL_FC_SYSTEM_ID, REAL_FC_COMPONENT_ID, tune, b""
        )
    )


def send_companion_heartbeat(link: Any) -> None:
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def request_streams(link: Any, rate_hz: int) -> None:
    link.mav.request_data_stream_send(
        REAL_FC_SYSTEM_ID,
        REAL_FC_COMPONENT_ID,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )
    # ArduCopter only emits this target while actually in a guided submode.  It
    # provides stronger confirmation than assuming a serial write was accepted.
    link.mav.command_long_send(
        REAL_FC_SYSTEM_ID,
        REAL_FC_COMPONENT_ID,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        POSITION_TARGET_LOCAL_NED_MESSAGE_ID,
        200_000,  # 5 Hz
        0,
        0,
        0,
        0,
        0,
    )


def send_mode_request(link: Any, mode: str) -> None:
    mapping = link.mode_mapping()
    if mapping is None or mode not in mapping:
        raise RuntimeError(f"flight controller does not expose mode {mode}")
    link.mav.set_mode_send(
        REAL_FC_SYSTEM_ID,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )


def send_velocity(
    link: Any,
    *,
    time_boot_ms: int,
    velocity_ned_mps: tuple[float, float, float],
    max_speed_mps: float,
) -> None:
    link.mav.send(
        make_message(
            GuidedVelocitySetpoint(
                time_boot_ms,
                velocity_ned_mps[0],
                velocity_ned_mps[1],
                velocity_ned_mps[2],
            ),
            target_system=REAL_FC_SYSTEM_ID,
            target_component=REAL_FC_COMPONENT_ID,
            max_speed_mps=max_speed_mps,
        )
    )


def ingest_message(
    message: Any,
    now_s: float,
    state: TelemetryState,
    gate: RcFollowGate,
    override_detector: PilotStickOverrideDetector,
) -> None:
    if message is None or message.get_srcSystem() != REAL_FC_SYSTEM_ID:
        return
    name = message.get_type()
    if name == "HEARTBEAT" and message.get_srcComponent() == REAL_FC_COMPONENT_ID:
        state.armed = is_armed(message)
        state.mode = mavutil.mode_string_v10(message).upper()
        state.heartbeat_at_s = now_s
        state.heartbeat_id += 1
    elif name == "RC_CHANNELS":
        gate.update_from_rc_channels(message, now_s)
        state.rc7_pwm = int(getattr(message, "chan7_raw", 0))
        state.rc_at_s = now_s
        channels = {
            channel: int(getattr(message, f"chan{channel}_raw"))
            for channel in (1, 2, 4)
            if hasattr(message, f"chan{channel}_raw")
        }
        state.pilot_override = override_detector.update(channels, now_s)
    elif name == "ATTITUDE" and message.get_srcComponent() == REAL_FC_COMPONENT_ID:
        state.yaw_rad = float(message.yaw)
        state.attitude_at_s = now_s
        state.time_boot_ms = int(message.time_boot_ms)
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
    elif name in {"GPS_GLOBAL_ORIGIN", "GLOBAL_POSITION_INT"}:
        latitude = int(getattr(message, "latitude", getattr(message, "lat", 0)))
        longitude = int(getattr(message, "longitude", getattr(message, "lon", 0)))
        if latitude != 0 and longitude != 0:
            state.origin_valid = True
            state.origin_latitude_deg = latitude / 1e7
            state.origin_longitude_deg = longitude / 1e7
    elif name == "POSITION_TARGET_LOCAL_NED":
        state.target_echo_at_s = now_s
        state.target_echo_velocity_ned_mps = (
            float(message.vx),
            float(message.vy),
            float(message.vz),
        )


def target_echo_matches(
    state: TelemetryState,
    *,
    sent_at_s: float | None,
    sent_velocity_ned_mps: tuple[float, float, float] | None,
    tolerance_mps: float,
) -> bool:
    if (
        sent_at_s is None
        or sent_velocity_ned_mps is None
        or state.target_echo_at_s is None
        or state.target_echo_at_s <= sent_at_s
        or state.target_echo_velocity_ned_mps is None
    ):
        return False
    return all(
        abs(actual - expected) <= tolerance_mps
        for actual, expected in zip(
            state.target_echo_velocity_ned_mps,
            sent_velocity_ned_mps,
        )
    )


def initial_disarmed_gate(
    link: Any,
    state: TelemetryState,
    gate: RcFollowGate,
    override_detector: PilotStickOverrideDetector,
) -> None:
    # Ignore companion/GCS heartbeats that may be routed back immediately after
    # a previous process closes; only system 1/component 1 satisfies the gate.
    deadline_s = time.monotonic() + 8.0
    heartbeat = None
    while time.monotonic() < deadline_s:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if is_real_fc_heartbeat(candidate):
            heartbeat = candidate
            break
    if heartbeat is None:
        raise RuntimeError("startup did not receive a real flight-controller heartbeat")
    if is_armed(heartbeat):
        raise RuntimeError("startup requires the real flight controller to be disarmed")
    now_s = time.monotonic()
    ingest_message(heartbeat, now_s, state, gate, override_detector)
    request_streams(link, 10)

    deadline_s = now_s + 12.0
    while time.monotonic() < deadline_s:
        now_s = time.monotonic()
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is not None:
            ingest_message(message, now_s, state, gate, override_detector)
        if state.armed:
            raise RuntimeError("flight controller armed during startup gate")
        rc_status = gate.status(now_s)
        if state.heartbeat_id >= 5 and rc_status.reason == "RC_DISABLED":
            return
    raise RuntimeError("startup requires five disarmed heartbeats and CH7 low (<=1200)")


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    install_pymavlink_instance_guard()
    config_bytes = args.config.read_bytes()
    config = json.loads(config_bytes.decode("utf-8"))
    duration_s = validate_props_off_config(config, args.duration_s)
    output_path = args.output or Path(config["output"]["log"])
    status_path = args.status or Path(config["output"]["status"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.parent.mkdir(parents=True, exist_ok=True)

    tracker_cfg = config["tracker"]
    tracker = AlphaBetaTargetTracker(
        alpha=float(tracker_cfg["alpha"]),
        beta=float(tracker_cfg["beta"]),
        max_residual_m=float(tracker_cfg["max_residual_m"]),
        min_dt_s=float(tracker_cfg["min_dt_s"]),
        max_dt_s=float(tracker_cfg["max_dt_s"]),
        acquire_count=int(tracker_cfg["acquire_count"]),
    )
    controller_cfg = config["controller"]
    controller = HorizontalFollowController(
        kp_xy=float(controller_cfg["kp_xy"]),
        deadband_m=float(controller_cfg["deadband_m"]),
        max_speed_mps=float(controller_cfg["max_speed_mps"]),
        max_accel_mps2=float(controller_cfg["max_accel_mps2"]),
        max_feedforward_mps=float(controller_cfg["max_feedforward_mps"]),
    )
    rc_cfg = config["rc_authorization"]
    gate = RcFollowGate(
        channel=int(rc_cfg["channel"]),
        enable_pwm_min=int(rc_cfg["enable_pwm_min"]),
        disable_pwm_max=int(rc_cfg["disable_pwm_max"]),
        timeout_s=float(rc_cfg["timeout_s"]),
    )
    mode_cfg = config["mode_manager"]
    mode_manager = FollowModeManager(
        guided_confirmations=int(mode_cfg["guided_confirmation_heartbeats"]),
        mode_request_timeout_s=float(mode_cfg["guided_request_timeout_s"]),
        mode_request_retry_s=float(mode_cfg["guided_request_retry_s"]),
        allowed_entry_modes=tuple(mode_cfg["allowed_entry_modes"]),
        allow_preexisting_guided=False,
    )
    stick_cfg = mode_cfg["pilot_stick_override"]
    override_detector = PilotStickOverrideDetector(
        threshold_pwm=int(stick_cfg["threshold_pwm"]),
        debounce_s=float(stick_cfg["debounce_s"]),
        centres_pwm={int(key): int(value) for key, value in stick_cfg["centres_pwm"].items()},
    )
    tone_policy = FollowTonePolicy()

    pixhawk = config["pixhawk"]
    ensure_conflicting_services_inactive(config)
    link = mavutil.mavlink_connection(
        pixhawk["serial"],
        baud=int(pixhawk["baud"]),
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    state = TelemetryState()
    try:
        initial_disarmed_gate(link, state, gate, override_detector)
    except Exception:
        link.close()
        raise

    stopped = False

    def stop_handler(*_: Any) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    counters: Counter[str] = Counter()
    started_s = time.monotonic()
    next_cycle_s = started_s
    next_heartbeat_s = started_s
    latest_track = None
    last_valid_target_at_s: float | None = None
    last_vision_sequence: int | None = None
    camera_ok = False
    last_vision: dict[str, Any] = {}
    previous_armed = False
    rc_cycle_required = False
    last_sent_at_s: float | None = None
    last_sent_velocity: tuple[float, float, float] | None = None
    target_echo_confirmed = False
    last_record: dict[str, Any] = {}
    error: str | None = None

    rate_hz = float(controller_cfg["command_rate_hz"])
    period_s = 1.0 / rate_hz
    telemetry_timeout_s = float(config["readiness"]["telemetry_timeout_s"])
    target_timeout_s = float(config["readiness"]["target_timeout_s"])
    echo_tolerance_mps = float(mode_cfg["target_echo_tolerance_mps"])
    allowed_readiness_modes = tuple(mode_cfg["allowed_entry_modes"]) + ("GUIDED",)

    try:
        with output_path.open("w", encoding="utf-8") as log_file:
            while not stopped and (
                duration_s is None or time.monotonic() - started_s < duration_s
            ):
                now_s = time.monotonic()
                for _ in range(300):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    counters[f"rx_{message.get_type()}"] += 1
                    ingest_message(message, now_s, state, gate, override_detector)

                if previous_armed and state.armed is False:
                    rc_cycle_required = True
                previous_armed = bool(state.armed)

                if now_s >= next_heartbeat_s:
                    send_companion_heartbeat(link)
                    counters["companion_heartbeat_tx"] += 1
                    next_heartbeat_s = now_s + 1.0

                if now_s < next_cycle_s:
                    time.sleep(min(0.01, next_cycle_s - now_s))
                    continue
                next_cycle_s = now_s + period_s

                try:
                    last_vision = read_vision_status(config["vision"]["status_url"])
                    frame_age_ms = float(last_vision.get("frame_age_ms", math.inf))
                    camera_ok = (
                        last_vision.get("sensor") == "ov9281"
                        and last_vision.get("mode") == "apriltag"
                        and frame_age_ms <= float(config["vision"]["maximum_frame_age_ms"])
                    )
                except Exception as exc:  # network/API failure is a fail-closed input
                    camera_ok = False
                    last_vision = {"vision_error": f"{type(exc).__name__}: {exc}"}

                sequence = last_vision.get("analysis_sequence")
                fresh_frame = isinstance(sequence, int) and sequence != last_vision_sequence
                if fresh_frame:
                    last_vision_sequence = sequence
                    if (
                        camera_ok
                        and last_vision.get("found")
                        and all(last_vision.get(key) is not None for key in ("x_m", "y_m", "z_m"))
                    ):
                        body_position = transform_camera_to_body(
                            (
                                float(last_vision["x_m"]),
                                float(last_vision["y_m"]),
                                float(last_vision["z_m"]),
                            ),
                            config["camera_to_body"],
                        )
                        candidate = tracker.update(
                            TargetMeasurement(
                                now_s,
                                body_position,
                                float(last_vision.get("decision_margin", 0.0)),
                                int(last_vision.get("hamming", 0)),
                                float(last_vision.get("reprojection_error_px", 0.0)),
                            )
                        )
                        if candidate.accepted:
                            latest_track = candidate
                            last_valid_target_at_s = now_s

                predicted_track = tracker.predict(now_s)
                if predicted_track is not None:
                    latest_track = predicted_track

                rc_status = gate.status(now_s)
                if rc_status.reason == "RC_DISABLED":
                    rc_cycle_required = False
                effective_rc_enabled = rc_status.enabled and not rc_cycle_required
                target_age_s = monotonic_age(now_s, last_valid_target_at_s)
                target_acquired = bool(latest_track is not None and latest_track.acquired)

                readiness_cfg = config["readiness"]
                readiness = evaluate_readiness(
                    ReadinessInputs(
                        heartbeat_age_s=monotonic_age(now_s, state.heartbeat_at_s),
                        armed=state.armed,
                        mode=state.mode,
                        rc7_pwm=rc_status.pwm,
                        rc_age_s=rc_status.age_s,
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
                        target_acquired=target_acquired,
                        target_age_s=target_age_s,
                        camera_ok=camera_ok,
                    ),
                    minimum_voltage_v=float(readiness_cfg["minimum_voltage_v"]),
                    minimum_remaining_pct=int(readiness_cfg["minimum_remaining_pct"]),
                    battery_telemetry_required=bool(
                        readiness_cfg.get("battery_telemetry_required", True)
                    ),
                    minimum_range_m=float(readiness_cfg["minimum_height_m"]),
                    maximum_range_m=float(readiness_cfg["maximum_height_m"]),
                    minimum_flow_quality=int(readiness_cfg["minimum_flow_quality"]),
                    telemetry_timeout_s=telemetry_timeout_s,
                    target_timeout_s=target_timeout_s,
                    allowed_modes=allowed_readiness_modes,
                )
                blockers = list(readiness.blockers)
                if not rc_status.fresh:
                    blockers.append("CH7_TELEMETRY_STALE")
                if (
                    state.yaw_rad is None
                    or monotonic_age(now_s, state.attitude_at_s) is None
                    or monotonic_age(now_s, state.attitude_at_s) > telemetry_timeout_s
                ):
                    blockers.append("ATTITUDE_STALE")
                if rc_cycle_required:
                    blockers.append("CH7_LOW_CYCLE_REQUIRED")
                prerequisites_ok = not blockers

                # Compare the newest echo with the preceding follow command
                # before this cycle replaces the command timestamp.
                if not target_echo_confirmed and target_echo_matches(
                    state,
                    sent_at_s=last_sent_at_s,
                    sent_velocity_ned_mps=last_sent_velocity,
                    tolerance_mps=echo_tolerance_mps,
                ):
                    target_echo_confirmed = True
                    counters["target_echo_confirmed"] += 1

                decision = mode_manager.update(
                    ModeManagerInputs(
                        timestamp_s=now_s,
                        armed=bool(state.armed),
                        current_mode=state.mode or "UNKNOWN",
                        rc_enable=effective_rc_enabled,
                        prerequisites_ok=prerequisites_ok,
                        pilot_stick_override=state.pilot_override,
                        mode_sample_id=state.heartbeat_id,
                    )
                )

                if not decision.allow_follow_velocity and not tone_policy.follow_confirmed:
                    target_echo_confirmed = False
                    last_sent_at_s = None
                    last_sent_velocity = None

                if decision.send_zero_velocity and state.mode == "GUIDED":
                    zero_velocity = (0.0, 0.0, 0.0)
                    send_velocity(
                        link,
                        time_boot_ms=state.time_boot_ms or (int(now_s * 1000) & 0xFFFFFFFF),
                        velocity_ned_mps=zero_velocity,
                        max_speed_mps=float(controller_cfg["max_speed_mps"]),
                    )
                    counters["zero_velocity_tx"] += 1
                    counters["movement_setpoint_tx"] += 1

                if decision.request_mode is not None:
                    send_mode_request(link, decision.request_mode)
                    counters[f"mode_request_{decision.request_mode}"] += 1

                candidate_body = (0.0, 0.0, 0.0)
                candidate_ned = (0.0, 0.0, 0.0)
                movement_sent_this_cycle = False
                if decision.allow_follow_velocity:
                    if latest_track is None or state.yaw_rad is None:
                        raise RuntimeError("active control reached without a target track and attitude")
                    target_north, target_east = body_to_ned(
                        latest_track.position_m[0], latest_track.position_m[1], state.yaw_rad
                    )
                    velocity_north, velocity_east = body_to_ned(
                        latest_track.velocity_mps[0], latest_track.velocity_mps[1], state.yaw_rad
                    )
                    command = controller.update(
                        timestamp_s=now_s,
                        vehicle_position_ned_m=(0.0, 0.0),
                        target_position_ned_m=(target_north, target_east),
                        target_velocity_ned_mps=(velocity_north, velocity_east),
                    )
                    candidate_ned = command.velocity_ned_mps
                    candidate_body = (
                        math.cos(state.yaw_rad) * candidate_ned[0]
                        + math.sin(state.yaw_rad) * candidate_ned[1],
                        -math.sin(state.yaw_rad) * candidate_ned[0]
                        + math.cos(state.yaw_rad) * candidate_ned[1],
                        0.0,
                    )
                    send_velocity(
                        link,
                        time_boot_ms=state.time_boot_ms or (int(now_s * 1000) & 0xFFFFFFFF),
                        velocity_ned_mps=candidate_ned,
                        max_speed_mps=float(controller_cfg["max_speed_mps"]),
                    )
                    counters["movement_setpoint_tx"] += 1
                    movement_sent_this_cycle = True
                    last_sent_at_s = now_s
                    last_sent_velocity = candidate_ned
                else:
                    controller.reset()

                observe_ready = observation_ready_without_ch7(
                    armed=state.armed,
                    current_mode=state.mode,
                    allowed_entry_modes=tuple(mode_cfg["allowed_entry_modes"]),
                    non_ch7_prerequisites_ok=prerequisites_ok,
                )
                heartbeat_fresh = (
                    monotonic_age(now_s, state.heartbeat_at_s) is not None
                    and monotonic_age(now_s, state.heartbeat_at_s) <= telemetry_timeout_s
                )
                exit_confirmed = bool(
                    heartbeat_fresh
                    and (state.armed is False or state.mode != "GUIDED")
                )
                tone_events = tone_policy.update(
                    rc_enabled=effective_rc_enabled,
                    observe_ready=observe_ready,
                    control_active=decision.allow_follow_velocity and movement_sent_this_cycle,
                    target_echo_confirmed=target_echo_confirmed,
                    exit_confirmed=exit_confirmed,
                )
                for tone_event in tone_events:
                    send_tune(link, TUNES[tone_event])
                    counters[f"tone_{tone_event.value}"] += 1
                    if tone_event.value == "EXIT_CONFIRMED":
                        target_echo_confirmed = False
                        last_sent_at_s = None
                        last_sent_velocity = None

                record = {
                    "timestamp_unix": time.time(),
                    "elapsed_s": now_s - started_s,
                    "fc_time_boot_ms": state.time_boot_ms,
                    "scope": config["scope"],
                    "flight_use_approved": False,
                    "props_removed_test_only": True,
                    "armed": state.armed,
                    "mode": state.mode,
                    "heartbeat_id": state.heartbeat_id,
                    "rc7_pwm": rc_status.pwm,
                    "rc7_reason": rc_status.reason,
                    "rc7_effective_authorization": effective_rc_enabled,
                    "pilot_stick_override": state.pilot_override,
                    "manager_state": decision.state.value,
                    "manager_reason": decision.reason,
                    "mode_request": decision.request_mode,
                    "readiness_ok": readiness.ready_for_follow_request,
                    "readiness_blockers": blockers,
                    "readiness_warnings": list(readiness.warnings),
                    "observe_ready": observe_ready,
                    "vision_found": bool(last_vision.get("found")),
                    "vision_sequence": last_vision_sequence,
                    "vision_frame_age_ms": last_vision.get("frame_age_ms"),
                    "target_age_s": target_age_s,
                    "target_acquired": target_acquired,
                    "target_body_frd_m": list(latest_track.position_m) if latest_track else None,
                    "estimated_target_body_velocity_mps": (
                        list(latest_track.velocity_mps) if latest_track else None
                    ),
                    "candidate_body_velocity_mps": list(candidate_body),
                    "candidate_local_ned_velocity_mps": list(candidate_ned),
                    "movement_setpoint_sent_this_cycle": movement_sent_this_cycle,
                    "movement_setpoint_tx_total": counters["movement_setpoint_tx"],
                    "target_echo_velocity_ned_mps": (
                        list(state.target_echo_velocity_ned_mps)
                        if state.target_echo_velocity_ned_mps is not None
                        else None
                    ),
                    "target_echo_confirmed": target_echo_confirmed,
                    "follow_control_confirmed": tone_policy.follow_confirmed,
                    "exit_confirmation_pending": tone_policy.exit_pending,
                    "tone_events": [event.value for event in tone_events],
                    "mode_requests_total": sum(
                        count for name, count in counters.items() if name.startswith("mode_request_")
                    ),
                    "arm_commands": 0,
                    "takeoff_commands": 0,
                    "land_commands": 0,
                    "direct_motor_commands": 0,
                }
                log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                log_file.flush()
                write_json(status_path, record)
                last_record = record

    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        # Only a previously confirmed session owns an exit operation.  Never
        # disarm; send zero and ask the mode manager to restore the entry mode.
        if tone_policy.follow_confirmed or tone_policy.exit_pending:
            shutdown_deadline_s = time.monotonic() + 2.5
            while time.monotonic() < shutdown_deadline_s:
                now_s = time.monotonic()
                for _ in range(100):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    ingest_message(message, now_s, state, gate, override_detector)
                if state.mode == "GUIDED" and state.armed:
                    try:
                        send_velocity(
                            link,
                            time_boot_ms=state.time_boot_ms or (int(now_s * 1000) & 0xFFFFFFFF),
                            velocity_ned_mps=(0.0, 0.0, 0.0),
                            max_speed_mps=float(controller_cfg["max_speed_mps"]),
                        )
                        counters["shutdown_zero_velocity_tx"] += 1
                        shutdown_decision = mode_manager.update(
                            ModeManagerInputs(
                                timestamp_s=now_s,
                                armed=True,
                                current_mode="GUIDED",
                                rc_enable=False,
                                prerequisites_ok=False,
                                mode_sample_id=state.heartbeat_id,
                            )
                        )
                        if shutdown_decision.request_mode is not None:
                            send_mode_request(link, shutdown_decision.request_mode)
                            counters[f"shutdown_mode_request_{shutdown_decision.request_mode}"] += 1
                    except Exception:
                        pass
                exit_confirmed = bool(state.armed is False or state.mode != "GUIDED")
                events = tone_policy.update(
                    rc_enabled=False,
                    observe_ready=False,
                    control_active=False,
                    target_echo_confirmed=target_echo_confirmed,
                    exit_confirmed=exit_confirmed,
                )
                for event in events:
                    try:
                        send_tune(link, TUNES[event])
                        counters[f"tone_{event.value}"] += 1
                    except Exception:
                        pass
                if exit_confirmed:
                    break
                time.sleep(0.1)
        link.close()

    ever_follow_confirmed = counters["tone_FOLLOW_CONFIRMED"] > 0
    all_confirmed_sessions_exited = (
        counters["tone_EXIT_CONFIRMED"] >= counters["tone_FOLLOW_CONFIRMED"]
    )
    summary = {
        "scope": config["scope"],
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "duration_s": time.monotonic() - started_s,
        "error": error,
        "counters": dict(counters),
        "final_mode": state.mode,
        "final_armed": state.armed,
        "follow_control_ever_confirmed": ever_follow_confirmed,
        "final_follow_control_confirmed": tone_policy.follow_confirmed,
        "all_confirmed_sessions_exited": all_confirmed_sessions_exited,
        "exit_confirmation_pending": tone_policy.exit_pending,
        "flight_use_approved": False,
        "props_removed_test_only": True,
        "control_enabled": True,
        "mavlink_transmit": True,
        "arm_commands": 0,
        "takeoff_commands": 0,
        "land_commands": 0,
        "direct_motor_commands": 0,
        "last_record": last_record,
    }
    write_json(output_path.with_suffix(".summary.json"), summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if error is not None or tone_policy.exit_pending or not all_confirmed_sessions_exited:
        return 2
    return 0 if ever_follow_confirmed else 3


if __name__ == "__main__":
    raise SystemExit(main())
