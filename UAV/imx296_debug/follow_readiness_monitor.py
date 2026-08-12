#!/usr/bin/env python3
"""Always-on, receive-only AprilTag and flight-readiness monitor.

The process owns the IMX296 camera, serves a preview on port 8765, reads
Pixhawk telemetry, and publishes a machine-readable readiness status.  It
contains no MAVLink transmit calls and cannot change mode, arm, take off,
land, or send velocity setpoints.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http import server
from pathlib import Path

import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

from follow_readiness import ReadinessInputs, evaluate_readiness
from landing_observer import AprilTagObserver, load_calibration, load_range_correction
from mavlink_landing_target import load_body_extrinsics
from rc_follow_gate import RcFollowGate
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement


class SharedPreview:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.status: dict = {"state": "STARTING", "mavlink_transmitted": False}

    def update(self, jpeg: bytes, status: dict) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.status = status
            self.condition.notify_all()


class PreviewHandler(server.BaseHTTPRequestHandler):
    shared: SharedPreview

    def do_GET(self) -> None:
        if self.path == "/status.json":
            payload = json.dumps(self.shared.status, ensure_ascii=False, indent=2).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with self.shared.condition:
                        self.shared.condition.wait(timeout=1.0)
                        jpeg = self.shared.jpeg
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        body = (
            "<!doctype html><meta charset='utf-8'><title>UAV AprilTag Monitor</title>"
            "<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center}"
            "img{max-width:95vw;max-height:82vh;border:1px solid #777}</style>"
            "<h2>AprilTag 跟随条件监控（只读，控制锁定）</h2>"
            "<img src='/stream.mjpg'><p><a href='/status.json'>status.json</a></p>"
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format: str, *_args) -> None:
        return


def monotonic_age(now: float, timestamp: float | None) -> float | None:
    return None if timestamp is None else max(0.0, now - timestamp)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    config_path = parse_args().config
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("control_enabled", False):
        raise RuntimeError("This receive-only monitor refuses control_enabled=true")

    calibration = Path(config["camera"]["calibration"])
    range_correction = Path(config["camera"]["range_correction"])
    extrinsics_path = Path(config["camera"]["extrinsics"])
    camera_matrix, distortion = load_calibration(calibration)
    range_scale, range_offset = load_range_correction(range_correction)
    extrinsics = load_body_extrinsics(extrinsics_path)
    observer = AprilTagObserver(
        int(config["target"]["id"]),
        float(config["target"]["size_m"]),
        camera_matrix,
        distortion,
        min_area_px=float(config["target"]["min_area_px"]),
        min_decision_margin=float(config["target"]["min_decision_margin"]),
        max_reprojection_error_px=float(config["target"]["max_reprojection_error_px"]),
        range_scale=range_scale,
        range_offset_m=range_offset,
        detector_threads=int(config["camera"]["detector_threads"]),
        quad_decimate=float(config["camera"]["quad_decimate"]),
    )
    tracker = AlphaBetaTargetTracker(alpha=0.65, beta=0.08, max_residual_m=0.25,
                                     min_dt_s=0.02, max_dt_s=0.5, acquire_count=5)
    rc_gate = RcFollowGate(channel=7, enable_pwm_min=1800,
                           disable_pwm_max=1200, timeout_s=0.5)

    telemetry = config["telemetry"]
    link = mavutil.mavlink_connection(
        telemetry["device"], baud=int(telemetry["baud"]), autoreconnect=False,
        source_system=191, source_component=191,
    )
    camera_cfg = config["camera"]
    camera = Picamera2()
    fps = float(camera_cfg["fps"])
    period_us = int(1_000_000 / fps)
    camera.configure(camera.create_video_configuration(
        main={"format": "RGB888", "size": (int(camera_cfg["width"]), int(camera_cfg["height"]))},
        controls={"FrameDurationLimits": (period_us, period_us)}, buffer_count=4,
    ))

    shared = SharedPreview()
    PreviewHandler.shared = shared
    httpd = server.ThreadingHTTPServer(("0.0.0.0", int(config["preview"]["port"])), PreviewHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    status_path = Path(config["output"]["status"])
    log_path = Path(config["output"]["log"])
    status_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    latest: dict[str, object] = {}
    timestamps: dict[str, float] = {}
    last_target_time: float | None = None
    latest_track = None
    last_status_write = 0.0
    camera.start()
    time.sleep(1.0)
    try:
        with log_path.open("a", encoding="utf-8") as log_file:
            while True:
                now = time.monotonic()
                for _ in range(300):
                    message = link.recv_match(blocking=False)
                    if message is None:
                        break
                    if message.get_srcSystem() != 1:
                        continue
                    name = message.get_type()
                    if name == "HEARTBEAT" and message.get_srcComponent() == 1:
                        latest["armed"] = bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        latest["mode"] = mavutil.mode_string_v10(message).upper()
                        timestamps["heartbeat"] = now
                    elif name == "RC_CHANNELS":
                        rc_gate.update_from_rc_channels(message, now)
                        latest["rc7_pwm"] = int(message.chan7_raw)
                        timestamps["rc"] = now
                    elif name == "EKF_STATUS_REPORT":
                        latest["ekf_flags"] = int(message.flags)
                        timestamps["ekf"] = now
                    elif name == "SYS_STATUS":
                        latest["battery_voltage_v"] = float(message.voltage_battery) / 1000.0
                        latest["battery_remaining_pct"] = int(message.battery_remaining)
                        timestamps["battery"] = now
                    elif name == "DISTANCE_SENSOR" and int(message.orientation) == 25:
                        latest["range_m"] = float(message.current_distance) / 100.0
                        timestamps["range"] = now
                    elif name == "OPTICAL_FLOW":
                        latest["flow_quality"] = int(message.quality)
                        timestamps["flow"] = now
                    elif name in {"GPS_GLOBAL_ORIGIN", "GLOBAL_POSITION_INT"}:
                        lat = int(getattr(message, "latitude", getattr(message, "lat", 0)))
                        lon = int(getattr(message, "longitude", getattr(message, "lon", 0)))
                        if lat != 0 and lon != 0:
                            latest["origin_valid"] = True
                            latest["origin_latitude_deg"] = lat / 1e7
                            latest["origin_longitude_deg"] = lon / 1e7

                frame_rgb = camera.capture_array("main")
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                observations, annotated = observer.detect(frame_bgr)
                accepted = None
                for observation in observations:
                    if not observation.valid or observation.x_m is None or observation.y_m is None or observation.z_m is None:
                        continue
                    body_position = extrinsics.transform(observation.x_m, observation.y_m, observation.z_m)
                    candidate = tracker.update(TargetMeasurement(
                        now, body_position, float(observation.decision_margin or 0.0),
                        int(observation.hamming or 0), float(observation.reprojection_error_px or 0.0),
                    ))
                    if candidate.accepted:
                        accepted = observation
                        latest_track = candidate
                        last_target_time = now
                predicted = tracker.predict(now)
                if predicted is not None:
                    latest_track = predicted

                target_age = monotonic_age(now, last_target_time)
                acquired = bool(latest_track is not None and latest_track.acquired)
                rc_status = rc_gate.status(now)
                inputs = ReadinessInputs(
                    heartbeat_age_s=monotonic_age(now, timestamps.get("heartbeat")),
                    armed=latest.get("armed"), mode=latest.get("mode"),
                    rc7_pwm=latest.get("rc7_pwm"), rc_age_s=monotonic_age(now, timestamps.get("rc")),
                    ekf_flags=latest.get("ekf_flags"), ekf_age_s=monotonic_age(now, timestamps.get("ekf")),
                    battery_voltage_v=latest.get("battery_voltage_v"),
                    battery_remaining_pct=latest.get("battery_remaining_pct"),
                    battery_age_s=monotonic_age(now, timestamps.get("battery")),
                    range_m=latest.get("range_m"), range_age_s=monotonic_age(now, timestamps.get("range")),
                    flow_quality=latest.get("flow_quality"), flow_age_s=monotonic_age(now, timestamps.get("flow")),
                    origin_valid=bool(latest.get("origin_valid", False)),
                    target_acquired=acquired, target_age_s=target_age, camera_ok=True,
                )
                safety = config["safety"]
                readiness = evaluate_readiness(
                    inputs,
                    minimum_voltage_v=float(safety["minimum_voltage_v"]),
                    minimum_remaining_pct=int(safety["minimum_remaining_pct"]),
                    minimum_range_m=float(safety["minimum_height_m"]),
                    maximum_range_m=float(safety["maximum_height_m"]),
                    minimum_flow_quality=int(safety["minimum_flow_quality"]),
                )
                status = {
                    "timestamp_unix": time.time(), "state": "MONITOR_ONLY_CONTROL_LOCKED",
                    "ready_for_follow_request": readiness.ready_for_follow_request,
                    "blockers": list(readiness.blockers), "warnings": list(readiness.warnings),
                    "armed": latest.get("armed"), "mode": latest.get("mode"),
                    "rc7_pwm": rc_status.pwm, "rc7_follow_permitted": rc_status.enabled,
                    "ekf_flags": latest.get("ekf_flags"), "origin_valid": bool(latest.get("origin_valid", False)),
                    "origin_latitude_deg": latest.get("origin_latitude_deg"),
                    "origin_longitude_deg": latest.get("origin_longitude_deg"),
                    "battery_voltage_v": latest.get("battery_voltage_v"),
                    "battery_remaining_pct": latest.get("battery_remaining_pct"),
                    "range_m": latest.get("range_m"), "flow_quality": latest.get("flow_quality"),
                    "target_visible_this_frame": accepted is not None,
                    "target_acquired": acquired, "target_age_s": target_age,
                    "target_body_frd_m": list(latest_track.position_m) if latest_track is not None else None,
                    "camera_continuously_open": True, "mavlink_receive_only": True,
                    "mavlink_transmitted": False, "mode_change_sent": False,
                    "arm_command_sent": False, "takeoff_command_sent": False,
                    "land_command_sent": False, "velocity_setpoint_sent": False,
                }
                cv2.putText(annotated, "MONITOR ONLY - CONTROL LOCKED", (18, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(annotated, f"mode={status['mode']} CH7={status['rc7_pwm']} range={status['range_m']}",
                            (18, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if ok:
                    shared.update(encoded.tobytes(), status)
                if now - last_status_write >= 1.0:
                    status_path.write_text(json.dumps(status, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                    log_file.write(json.dumps(status, ensure_ascii=False) + "\n")
                    log_file.flush()
                    last_status_write = now
    finally:
        httpd.shutdown()
        camera.stop()
        camera.close()
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
