#!/usr/bin/env python3
"""Propeller-free follow observer with RC entry/exit tones.

This program reads the OV9281 status API and Pixhawk telemetry, estimates the
AprilTag motion, and encodes (but never transmits) candidate velocity packets.
The only outgoing MAVLink messages are a stream request and optional PLAY_TUNE
notifications. It exits immediately if the real flight controller is armed.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import time
import urllib.request
from collections import Counter
from pathlib import Path

from pymavlink import mavutil
from pymavlink.dialects.v20 import common


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "imx296_debug"))
from follow_controller import HorizontalFollowController  # noqa: E402
from follow_tone_policy import OBSERVE_ENDED_TUNE, OBSERVE_READY_TUNE  # noqa: E402
from mavlink_guided_velocity import GuidedVelocitySetpoint, pack_message  # noqa: E402
from rc_follow_gate import RcFollowGate  # noqa: E402
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement  # noqa: E402


ENTER_TUNE = OBSERVE_READY_TUNE
EXIT_TUNE = OBSERVE_ENDED_TUNE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=180.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--status", type=Path, required=True)
    parser.add_argument("--buzzer", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def read_status(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=1.0) as response:
        return json.load(response)


def is_real_fc(message) -> bool:
    return (
        message is not None
        and message.get_srcSystem() == 1
        and message.get_srcComponent() == 1
    )


def is_armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def transform(position: tuple[float, float, float], extrinsics: dict) -> tuple[float, float, float]:
    rotation = extrinsics["rotation_camera_optical_to_body_frd"]
    translation = extrinsics["translation_m"]
    return tuple(
        sum(float(rotation[row][column]) * position[column] for column in range(3))
        + float(translation[row])
        for row in range(3)
    )


def body_to_ned(forward: float, right: float, yaw: float) -> tuple[float, float]:
    return (
        math.cos(yaw) * forward - math.sin(yaw) * right,
        math.sin(yaw) * forward + math.cos(yaw) * right,
    )


def send_tune(link, encoder, tune: bytes) -> int:
    message = common.MAVLink_play_tune_message(1, 1, tune, b"")
    link.write(message.pack(encoder))
    return 1


def numeric_stats(values: list[float]) -> dict | None:
    if not values:
        return None
    return {
        "count": len(values), "min": min(values),
        "mean": statistics.fmean(values), "max": max(values),
    }


def self_test() -> int:
    config = {
        "rotation_camera_optical_to_body_frd": [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
        "translation_m": [0, 0, 0],
    }
    assert transform((0.2, -0.3, 0.7), config) == (-0.2, 0.3, 0.7)
    assert body_to_ned(1.0, 0.0, math.pi / 2)[1] > 0.999
    tracker = AlphaBetaTargetTracker(acquire_count=5)
    track = None
    for index in range(5):
        track = tracker.update(TargetMeasurement(index * 0.1, (0.1 + index * 0.01, 0.0, 0.7)))
    assert track is not None and track.acquired and track.velocity_mps[0] > 0
    controller = HorizontalFollowController(max_speed_mps=1.0, max_accel_mps2=1.0)
    command = controller.update(
        timestamp_s=0.5,
        vehicle_position_ned_m=(0, 0),
        target_position_ned_m=track.position_m[:2],
        target_velocity_ned_mps=track.velocity_mps[:2],
    )
    packet = pack_message(
        GuidedVelocitySetpoint(500, command.velocity_ned_mps[0], command.velocity_ned_mps[1]),
        max_speed_mps=1.0,
    )
    assert packet and math.hypot(*command.velocity_ned_mps[:2]) <= 1.0
    print("SELF_TEST_OK TRANSFORM=1 TRACKER=1 CONTROLLER=1 ENCODER=1 TRANSMIT=0")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return self_test()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if config["safety"]["control_enabled"] or config["safety"]["mavlink_transmit"]:
        raise RuntimeError("observer requires real control and MAVLink movement transmission disabled")
    extrinsics = config["camera_to_body"]
    if not extrinsics["enabled"] or extrinsics["translation_m"] != [0.0, 0.0, 0.0]:
        raise RuntimeError("confirmed OV9281 zero-translation transform is required")

    controller_config = config["controller_candidate"]
    tracker_config = config["tracker"]
    tracker = AlphaBetaTargetTracker(
        alpha=float(tracker_config["alpha"]), beta=float(tracker_config["beta"]),
        max_residual_m=float(tracker_config["max_residual_m"]),
        min_dt_s=0.02, max_dt_s=0.5,
        acquire_count=int(tracker_config["acquire_count"]),
    )
    controller = HorizontalFollowController(
        kp_xy=float(controller_config["kp_xy"]),
        deadband_m=float(controller_config["deadband_m"]),
        max_speed_mps=float(controller_config["max_speed_mps"]),
        max_accel_mps2=float(controller_config["max_accel_mps2"]),
        max_feedforward_mps=float(controller_config["max_feedforward_mps"]),
    )
    rc = config["rc_authorization"]
    gate = RcFollowGate(
        channel=int(rc["channel"]), enable_pwm_min=int(rc["enable_pwm_min"]),
        disable_pwm_max=int(rc["disable_pwm_max"]), timeout_s=float(rc["timeout_s"]),
    )

    link = mavutil.mavlink_connection(
        config["pixhawk"]["serial"], baud=int(config["pixhawk"]["baud"]),
        autoreconnect=False, source_system=191, source_component=191,
    )
    heartbeat = link.wait_heartbeat(timeout=8)
    if not is_real_fc(heartbeat) or is_armed(heartbeat):
        raise RuntimeError("safety stop: five-second real FC disarmed gate failed")
    confirmed = 1
    while confirmed < 5:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if not is_real_fc(heartbeat) or is_armed(heartbeat):
            raise RuntimeError("safety stop: real FC did not remain disarmed")
        confirmed += 1
    link.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    tune_encoder = common.MAVLink(None, srcSystem=191, srcComponent=191)

    stopped = False
    def stop_handler(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.status.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    next_api = started
    last_sequence = None
    latest_track = None
    last_valid_at = None
    latest_yaw = None
    active = False
    hold_latched = False
    state = "DISABLED"
    message_counts: Counter[str] = Counter()
    state_counts: Counter[str] = Counter()
    valid_frames = 0
    tune_packets = 0
    encoded_packets = 0
    target_speeds: list[float] = []
    candidate_speeds: list[float] = []

    with args.output.open("w", encoding="utf-8") as log:
        try:
            while not stopped and time.monotonic() - started < args.duration_s:
                for _ in range(100):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    if message.get_srcSystem() != 1:
                        continue
                    name = message.get_type()
                    message_counts[name] += 1
                    if name == "HEARTBEAT" and message.get_srcComponent() == 1:
                        if is_armed(message):
                            if active and args.buzzer:
                                tune_packets += send_tune(link, tune_encoder, EXIT_TUNE)
                            raise RuntimeError("safety stop: real FC became armed")
                    elif name == "RC_CHANNELS":
                        gate.update_from_rc_channels(message, time.monotonic())
                    elif name == "ATTITUDE" and message.get_srcComponent() == 1:
                        latest_yaw = float(message.yaw)

                now = time.monotonic()
                if now < next_api:
                    time.sleep(min(0.01, next_api - now))
                    continue
                vision = read_status(config["vision"]["status_url"])
                sequence = vision.get("analysis_sequence")
                fresh_frame = sequence is not None and sequence != last_sequence
                if fresh_frame:
                    last_sequence = sequence
                    if vision.get("found") and all(vision.get(key) is not None for key in ("x_m", "y_m", "z_m")):
                        position = transform(
                            (float(vision["x_m"]), float(vision["y_m"]), float(vision["z_m"])),
                            extrinsics,
                        )
                        candidate = tracker.update(TargetMeasurement(
                            now, position,
                            float(vision.get("decision_margin", 0)),
                            int(vision.get("hamming", 0)),
                            float(vision.get("reprojection_error_px", 0)),
                        ))
                        if candidate.accepted:
                            latest_track = candidate
                            last_valid_at = now
                            valid_frames += 1

                rc_status = gate.status(now)
                target_age = None if last_valid_at is None else now - last_valid_at
                candidate_body = (0.0, 0.0, 0.0)
                candidate_ned = None
                packet_hex = None

                if not rc_status.enabled:
                    state = "DISABLED"
                    hold_latched = False
                    controller.reset()
                elif hold_latched:
                    state = "HOLD_LATCHED"
                    controller.reset()
                elif latest_track is None or not latest_track.acquired:
                    state = "ACQUIRE"
                    controller.reset()
                elif target_age is not None and target_age <= float(tracker_config["predict_until_s"]):
                    state = "FOLLOW_OBSERVE"
                    command = controller.update(
                        timestamp_s=now,
                        vehicle_position_ned_m=(0.0, 0.0),
                        target_position_ned_m=latest_track.position_m[:2],
                        target_velocity_ned_mps=latest_track.velocity_mps[:2],
                    )
                    candidate_body = command.velocity_ned_mps
                elif target_age is not None and target_age <= float(tracker_config["hold_after_s"]):
                    state = "PREDICT_DECEL"
                    scale = 1.0 - (
                        (target_age - float(tracker_config["predict_until_s"]))
                        / (float(tracker_config["hold_after_s"]) - float(tracker_config["predict_until_s"]))
                    )
                    command = controller.update(
                        timestamp_s=now,
                        vehicle_position_ned_m=(0.0, 0.0),
                        target_position_ned_m=latest_track.position_m[:2],
                        target_velocity_ned_mps=latest_track.velocity_mps[:2],
                        velocity_scale=max(0.0, min(1.0, scale)),
                    )
                    candidate_body = command.velocity_ned_mps
                else:
                    state = "HOLD_LATCHED"
                    hold_latched = True
                    controller.reset()

                should_be_active = state == "FOLLOW_OBSERVE"
                if should_be_active and not active and args.buzzer:
                    tune_packets += send_tune(link, tune_encoder, ENTER_TUNE)
                elif active and not should_be_active and args.buzzer:
                    tune_packets += send_tune(link, tune_encoder, EXIT_TUNE)
                active = should_be_active

                if state in ("FOLLOW_OBSERVE", "PREDICT_DECEL") and latest_yaw is not None:
                    north, east = body_to_ned(candidate_body[0], candidate_body[1], latest_yaw)
                    candidate_ned = (north, east, 0.0)
                    setpoint = GuidedVelocitySetpoint(int(now * 1000) & 0xFFFFFFFF, north, east)
                    packet_hex = pack_message(
                        setpoint, max_speed_mps=float(controller_config["max_speed_mps"])
                    ).hex()
                    encoded_packets += 1

                target_speed = None
                if latest_track is not None:
                    target_speed = math.hypot(latest_track.velocity_mps[0], latest_track.velocity_mps[1])
                    target_speeds.append(target_speed)
                candidate_speed = math.hypot(candidate_body[0], candidate_body[1])
                candidate_speeds.append(candidate_speed)
                state_counts[state] += 1
                record = {
                    "elapsed_s": now - started,
                    "state": state,
                    "rc7_pwm": rc_status.pwm,
                    "rc_enabled": rc_status.enabled,
                    "vision_found": bool(vision.get("found")),
                    "analysis_sequence": sequence,
                    "body_frd_m": list(latest_track.position_m) if latest_track else None,
                    "estimated_target_body_velocity_mps": list(latest_track.velocity_mps) if latest_track else None,
                    "estimated_target_horizontal_speed_mps": target_speed,
                    "candidate_body_velocity_mps": list(candidate_body),
                    "candidate_local_ned_velocity_mps": list(candidate_ned) if candidate_ned else None,
                    "would_send_mavlink_v2_hex": packet_hex,
                    "mavlink_movement_transmitted": False,
                    "buzzer_tune_packets_total": tune_packets,
                    "flight_controller_armed": False,
                }
                log.write(json.dumps(record, ensure_ascii=False) + "\n")
                log.flush()
                args.status.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                next_api = now + 1.0 / float(config["vision"]["poll_hz"])
        finally:
            if active and args.buzzer:
                tune_packets += send_tune(link, tune_encoder, EXIT_TUNE)
                time.sleep(0.2)
            link.close()

    summary = {
        "scope": "props_off_disarmed_follow_observer",
        "duration_s": time.monotonic() - started,
        "valid_vision_frames": valid_frames,
        "state_counts": dict(state_counts),
        "target_horizontal_speed_mps": numeric_stats(target_speeds),
        "candidate_horizontal_speed_mps": numeric_stats(candidate_speeds),
        "incoming_message_counts": dict(message_counts),
        "buzzer_tune_packets_transmitted": tune_packets,
        "stream_request_packets_transmitted": 1,
        "mavlink_movement_packets_transmitted": 0,
        "arm_commands": 0, "mode_commands": 0, "motor_commands": 0,
        "real_follow_control_enabled": False,
    }
    args.output.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("OBSERVE_ONLY=1 MOVEMENT_TRANSMITTED=0 ARM=0 MODE=0 MOTOR=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
