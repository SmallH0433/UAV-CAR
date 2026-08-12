#!/usr/bin/env python3
"""Disarmed AprilTag follow-command preview with a receive-only FC link.

This stage-3 bench tool reads IMX296 frames and incoming Pixhawk telemetry,
calculates bounded BODY_FRD and LOCAL_NED velocity proposals, and logs them.
It never transmits MAVLink, changes mode, arms, or controls an actuator.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter
from pathlib import Path

import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

from follow_controller import HorizontalFollowController
from landing_observer import AprilTagObserver, load_calibration, load_range_correction
from mavlink_guided_velocity import GuidedVelocitySetpoint, pack_message
from mavlink_landing_target import load_body_extrinsics
from rc_follow_gate import RcFollowGate
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive-only AprilTag command preview")
    parser.add_argument("--serial", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--range-correction", type=Path, required=True)
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--annotated-output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--detector-threads", type=int, default=4)
    parser.add_argument("--quad-decimate", type=float, default=3.0)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size-m", type=float, default=0.135)
    parser.add_argument("--rc-enable-channel", type=int, default=7)
    parser.add_argument("--rc-enable-pwm-min", type=int, default=1800)
    parser.add_argument("--rc-disable-pwm-max", type=int, default=1200)
    parser.add_argument("--rc-timeout-s", type=float, default=0.5)
    return parser.parse_args()


def is_armed(heartbeat) -> bool:
    return bool(
        int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def wait_for_disarmed_gate(link, samples: int = 5, timeout_s: float = 12.0) -> int:
    confirmed = 0
    deadline = time.monotonic() + timeout_s
    while confirmed < samples and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is None
            or message.get_srcSystem() != 1
            or message.get_srcComponent() != 1
        ):
            continue
        if is_armed(message):
            raise RuntimeError("Safety stop: real flight controller is armed")
        confirmed += 1
    if confirmed < samples:
        raise RuntimeError("Safety stop: five disarmed FC heartbeats were not received")
    return confirmed


def rotate_body_to_ned(
    forward_mps: float, right_mps: float, yaw_rad: float
) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * forward_mps - sin_yaw * right_mps,
        sin_yaw * forward_mps + cos_yaw * right_mps,
    )


def update_preview_velocity(
    controller,
    preview_state: str,
    timestamp_s: float,
    latest_track,
    velocity_scale: float,
) -> tuple[float, float, float]:
    """Return a preview velocity, forcing safety states to an immediate stop."""
    if preview_state in {"RC_DISABLED", "ACQUIRE", "PREVIEW_HOLD"} or latest_track is None:
        controller.reset()
        return (0.0, 0.0, 0.0)

    command = controller.update(
        timestamp_s=timestamp_s,
        vehicle_position_ned_m=(0.0, 0.0),
        target_position_ned_m=(
            latest_track.position_m[0], latest_track.position_m[1]
        ),
        target_velocity_ned_mps=(
            latest_track.velocity_mps[0], latest_track.velocity_mps[1]
        ),
        velocity_scale=velocity_scale,
    )
    return command.velocity_ned_mps


def numeric_stats(values: list[float]) -> dict | None:
    if not values:
        return None
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "max": max(values),
        "stddev": statistics.pstdev(values),
    }


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0 or args.fps <= 0:
        raise ValueError("duration and FPS must be positive")

    extrinsics = load_body_extrinsics(args.extrinsics)
    if extrinsics.allowed_scope != "disarmed_bench_and_sitl":
        raise RuntimeError("Extrinsics are not approved for disarmed bench preview")
    if extrinsics.flight_use_approved:
        raise RuntimeError("This preview expects explicitly non-flight extrinsics")

    camera_matrix, distortion = load_calibration(args.calibration)
    range_scale, range_offset = load_range_correction(args.range_correction)
    observer = AprilTagObserver(
        args.tag_id,
        args.tag_size_m,
        camera_matrix,
        distortion,
        min_area_px=150.0,
        min_decision_margin=20.0,
        max_reprojection_error_px=2.5,
        range_scale=range_scale,
        range_offset_m=range_offset,
        detector_threads=args.detector_threads,
        quad_decimate=args.quad_decimate,
    )
    tracker = AlphaBetaTargetTracker(
        alpha=0.65,
        beta=0.08,
        max_residual_m=0.25,
        min_dt_s=0.02,
        max_dt_s=0.5,
        acquire_count=5,
    )
    controller = HorizontalFollowController(
        kp_xy=0.4,
        deadband_m=0.05,
        max_speed_mps=0.20,
        max_accel_mps2=0.20,
        max_feedforward_mps=0.50,
    )
    rc_gate = RcFollowGate(
        channel=args.rc_enable_channel,
        enable_pwm_min=args.rc_enable_pwm_min,
        disable_pwm_max=args.rc_disable_pwm_max,
        timeout_s=args.rc_timeout_s,
    )

    link = mavutil.mavlink_connection(
        args.serial,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=191,
    )
    gate_samples = wait_for_disarmed_gate(link)
    print(f"DISARMED_GATE={gate_samples}/5 RECEIVE_ONLY_LINK=1", flush=True)

    camera = Picamera2()
    frame_period_us = int(1_000_000 / args.fps)
    camera.configure(
        camera.create_video_configuration(
            main={"format": "RGB888", "size": (args.width, args.height)},
            controls={"FrameDurationLimits": (frame_period_us, frame_period_us)},
            buffer_count=4,
        )
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.annotated_output.parent.mkdir(parents=True, exist_ok=True)
    state_counts: Counter[str] = Counter()
    incoming_counts: Counter[str] = Counter()
    valid_detections = 0
    rejected_detections = 0
    frames = 0
    records = 0
    last_accepted_time = -1e9
    latest_track = None
    latest_yaw = None
    max_proposed_speed = 0.0
    distances = []
    body_x_values = []
    body_y_values = []
    proposed_forward = []
    proposed_right = []
    armed_heartbeats = 0
    disarmed_heartbeats = gate_samples
    last_annotated = None
    rc_enabled_records = 0
    rc_disabled_records = 0
    rc_pwm_values = []

    camera.start()
    time.sleep(1.0)
    started = time.monotonic()
    try:
        with args.output.open("w", encoding="utf-8") as log_file:
            while time.monotonic() - started < args.duration_s:
                for _ in range(250):
                    incoming = link.recv_match(blocking=False)
                    if incoming is None:
                        break
                    name = incoming.get_type()
                    incoming_counts[name] += 1
                    if (
                        name == "HEARTBEAT"
                        and incoming.get_srcSystem() == 1
                        and incoming.get_srcComponent() == 1
                    ):
                        if is_armed(incoming):
                            armed_heartbeats += 1
                            raise RuntimeError(
                                "Safety stop: real flight controller became armed"
                            )
                        disarmed_heartbeats += 1
                    elif (
                        name == "ATTITUDE"
                        and incoming.get_srcSystem() == 1
                        and incoming.get_srcComponent() == 1
                    ):
                        latest_yaw = float(incoming.yaw)
                    elif name == "RC_CHANNELS" and incoming.get_srcSystem() == 1:
                        rc_gate.update_from_rc_channels(incoming, time.monotonic())

                frame = camera.capture_array("main")
                observations, annotated = observer.detect(frame)
                frames += 1
                now = time.monotonic()
                accepted_this_frame = None
                for observation in observations:
                    if not observation.valid:
                        rejected_detections += 1
                        continue
                    if (
                        observation.x_m is None
                        or observation.y_m is None
                        or observation.z_m is None
                    ):
                        rejected_detections += 1
                        continue
                    body_position = extrinsics.transform(
                        observation.x_m, observation.y_m, observation.z_m
                    )
                    candidate = tracker.update(
                        TargetMeasurement(
                            now,
                            body_position,
                            float(observation.decision_margin or 0.0),
                            int(observation.hamming or 0),
                            float(observation.reprojection_error_px or 0.0),
                        )
                    )
                    if candidate.accepted:
                        accepted_this_frame = observation
                        latest_track = candidate
                        last_accepted_time = now
                        valid_detections += 1
                        distances.append(float(observation.distance_m or 0.0))
                        body_x_values.append(body_position[0])
                        body_y_values.append(body_position[1])
                    else:
                        rejected_detections += 1

                predicted = tracker.predict(now)
                if predicted is not None:
                    latest_track = predicted
                age_s = now - last_accepted_time
                acquired = latest_track is not None and latest_track.acquired
                rc_status = rc_gate.status(now)
                if rc_status.pwm is not None:
                    rc_pwm_values.append(rc_status.pwm)
                if rc_status.enabled:
                    rc_enabled_records += 1
                else:
                    rc_disabled_records += 1
                if not rc_status.enabled:
                    preview_state = "RC_DISABLED"
                    velocity_scale = 0.0
                elif not acquired:
                    preview_state = "ACQUIRE"
                    velocity_scale = 0.0
                elif age_s <= 0.25:
                    preview_state = "PREVIEW_FOLLOW"
                    velocity_scale = 1.0
                elif age_s <= 0.70:
                    preview_state = "PREVIEW_DECEL"
                    velocity_scale = 1.0 - ((age_s - 0.25) / 0.45)
                else:
                    preview_state = "PREVIEW_HOLD"
                    velocity_scale = 0.0
                state_counts[preview_state] += 1

                body_velocity = update_preview_velocity(
                    controller,
                    preview_state,
                    now,
                    latest_track,
                    velocity_scale,
                )

                ned_velocity = None
                packet_hex = None
                if latest_yaw is not None:
                    velocity_n, velocity_e = rotate_body_to_ned(
                        body_velocity[0], body_velocity[1], latest_yaw
                    )
                    ned_velocity = (velocity_n, velocity_e, 0.0)
                    packet_hex = pack_message(
                        GuidedVelocitySetpoint(
                            time_boot_ms=int(now * 1000.0) & 0xFFFFFFFF,
                            vx_mps=velocity_n,
                            vy_mps=velocity_e,
                            vz_mps=0.0,
                            yaw_rate_rad_s=0.0,
                        ),
                        max_speed_mps=0.20,
                    ).hex()

                speed = math.hypot(body_velocity[0], body_velocity[1])
                max_proposed_speed = max(max_proposed_speed, speed)
                proposed_forward.append(body_velocity[0])
                proposed_right.append(body_velocity[1])
                record = {
                    "elapsed_s": now - started,
                    "preview_state": preview_state,
                    "target_visible": accepted_this_frame is not None,
                    "target_age_s": age_s,
                    "target_acquired": acquired,
                    "track_body_frd_m": (
                        list(latest_track.position_m) if latest_track is not None else None
                    ),
                    "track_body_frd_velocity_mps": (
                        list(latest_track.velocity_mps) if latest_track is not None else None
                    ),
                    "proposed_body_frd_velocity_mps": list(body_velocity),
                    "vehicle_yaw_rad": latest_yaw,
                    "proposed_local_ned_velocity_mps": (
                        list(ned_velocity) if ned_velocity is not None else None
                    ),
                    "would_send_mavlink_v2_hex": packet_hex,
                    "mavlink_transmitted": False,
                    "flight_controller_armed": False,
                    "rc_enable_channel": args.rc_enable_channel,
                    "rc_enable_pwm": rc_status.pwm,
                    "rc_enable": rc_status.enabled,
                    "rc_enable_reason": rc_status.reason,
                    "rc_sample_age_s": rc_status.age_s,
                    "scope": "disarmed_bench_command_preview_only",
                }
                log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                records += 1

                cv2.putText(
                    annotated,
                    "FOLLOW COMMAND PREVIEW ONLY - NOT SENT",
                    (20, 38),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    annotated,
                    f"state={preview_state} body_v=({body_velocity[0]:+.3f}, {body_velocity[1]:+.3f}) m/s",
                    (20, 76),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                last_annotated = annotated
    finally:
        if last_annotated is not None:
            cv2.imwrite(str(args.annotated_output), last_annotated)
        camera.stop()
        camera.close()
        link.close()

    elapsed = time.monotonic() - started
    summary = {
        "scope": "disarmed_bench_command_preview_only",
        "duration_s": elapsed,
        "frames": frames,
        "records": records,
        "valid_detections": valid_detections,
        "rejected_detections": rejected_detections,
        "valid_rate_hz": valid_detections / elapsed,
        "state_counts": dict(state_counts),
        "distance_m": numeric_stats(distances),
        "target_body_frd_x_m": numeric_stats(body_x_values),
        "target_body_frd_y_m": numeric_stats(body_y_values),
        "proposed_forward_mps": numeric_stats(proposed_forward),
        "proposed_right_mps": numeric_stats(proposed_right),
        "max_proposed_horizontal_speed_mps": max_proposed_speed,
        "speed_limit_mps": 0.20,
        "incoming_message_counts": dict(incoming_counts),
        "disarmed_heartbeat_samples": disarmed_heartbeats,
        "armed_heartbeat_samples": armed_heartbeats,
        "serial_link_receive_only": True,
        "rc_authorization": {
            "channel": args.rc_enable_channel,
            "enable_pwm_min": args.rc_enable_pwm_min,
            "disable_pwm_max": args.rc_disable_pwm_max,
            "timeout_s": args.rc_timeout_s,
            "enabled_records": rc_enabled_records,
            "disabled_records": rc_disabled_records,
            "pwm": numeric_stats(rc_pwm_values),
            "fail_closed": True,
        },
        "mavlink_packets_encoded_offline": records,
        "mavlink_packets_transmitted": 0,
        "parameter_write": False,
        "mode_change": False,
        "arm_command": False,
        "motor_command": False,
        "takeoff_command": False,
        "land_command": False,
        "flight_use_approved": False,
    }
    args.summary.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("PREVIEW_ONLY=1 MAVLINK_TRANSMITTED=0 CONTROL_COMMAND_SENT=0")
    return 0 if valid_detections > 0 and armed_heartbeats == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
