#!/usr/bin/env python3
"""Closed-loop AprilTag moving-target follow acceptance test in local SITL only.

The endpoint is intentionally fixed to localhost.  This program arms and flies
an ArduCopter software simulation; it cannot connect to the Raspberry Pi or the
physical Pixhawk.
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

from pymavlink import mavutil


WORKSPACE = Path("/mnt/d/Codex/UAV")
MODULE_DIR = WORKSPACE / "imx296_debug"
sys.path.insert(0, str(MODULE_DIR))

from follow_controller import HorizontalFollowController  # noqa: E402
from follow_state_machine import (  # noqa: E402
    FollowInputs,
    FollowSafetyStateMachine,
)
from mavlink_guided_velocity import (  # noqa: E402
    GuidedVelocitySetpoint,
    make_message,
)
from target_tracker import (  # noqa: E402
    AlphaBetaTargetTracker,
    TargetMeasurement,
)


SITL_ENDPOINT = "udpin:127.0.0.1:14550"
RATE_HZ = 10.0
CONTROL_DURATION_S = 32.0
TAKEOFF_ALT_M = 3.0
MAX_COMMAND_SPEED_MPS = 0.20
MAX_FOLLOW_ERROR_M = 0.30
OUTPUT_JSONL = WORKSPACE / "output/apriltag_follow_sitl_closed_loop_20260807.jsonl"
OUTPUT_SUMMARY = WORKSPACE / "output/apriltag_follow_sitl_closed_loop_20260807_summary.json"


def set_message_interval(connection, message_id: int, rate_hz: float) -> None:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        1_000_000.0 / rate_hz,
        0,
        0,
        0,
        0,
        0,
    )


def set_mode(connection, mode: str, timeout_s: float = 10.0) -> None:
    mapping = connection.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"SITL does not expose mode {mode}")
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and mavutil.mode_string_v10(heartbeat).upper() == mode:
            return
    raise RuntimeError(f"SITL mode change timed out: {mode}")


def arm_sitl_when_ready(connection, timeout_s: float = 90.0) -> None:
    """Use normal arm requests while retaining all ArduCopter pre-arm checks."""
    deadline = time.monotonic() + timeout_s
    next_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        send_gcs_heartbeat(connection)
        request_sitl_streams(connection)
        if now >= next_request:
            connection.arducopter_arm()
            next_request = now + 3.0
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            heartbeat is not None
            and heartbeat.get_srcSystem() == connection.target_system
            and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            return
    raise RuntimeError("SITL did not pass normal pre-arm checks before timeout")


def wait_global_position(connection, timeout_s: float = 15.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        send_gcs_heartbeat(connection)
        request_sitl_streams(connection)
        message = connection.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0
        )
        if (
            message is not None
            and abs(int(message.lat)) > 1_000_000
            and abs(int(message.lon)) > 1_000_000
        ):
            return message
    raise RuntimeError("SITL GLOBAL_POSITION_INT was not received")


def global_to_local_ned(message, origin: tuple[int, int]) -> tuple[float, float, float]:
    earth_radius_m = 6_378_137.0
    lat_rad = math.radians(origin[0] / 1e7)
    north_m = math.radians((int(message.lat) - origin[0]) / 1e7) * earth_radius_m
    east_m = (
        math.radians((int(message.lon) - origin[1]) / 1e7)
        * earth_radius_m
        * math.cos(lat_rad)
    )
    down_m = -float(message.relative_alt) / 1000.0
    return north_m, east_m, down_m


def request_sitl_streams(connection, rate_hz: int = 20) -> None:
    """Support both modern message intervals and legacy SITL stream groups."""
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )


def send_gcs_heartbeat(connection) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def target_position(t: float) -> tuple[float, float, float]:
    """Continuous ground-target trajectory in LOCAL_NED metres."""
    if t < 2.0:
        return (0.0, 0.0, 0.0)
    if t < 10.0:
        return (0.10 * (t - 2.0), 0.0, 0.0)
    if t < 16.0:
        turn_t = t - 10.0
        return (0.80 + 0.08 * turn_t, 0.06 * turn_t, 0.0)
    after_t = t - 16.0
    return (1.28 + 0.08 * after_t, 0.36 - 0.04 * after_t, 0.0)


def target_visible(t: float) -> bool:
    return not (16.0 <= t < 17.2)


def rc_enabled(t: float) -> bool:
    return not (19.0 <= t < 19.4)


def body_measurement_to_local_target(
    vehicle_ned: tuple[float, float, float],
    target_ned: tuple[float, float, float],
    yaw_rad: float,
    t: float,
) -> tuple[float, float, float]:
    """Emulate BODY_FRD camera measurement and transform it back to LOCAL_NED."""
    delta_n = target_ned[0] - vehicle_ned[0]
    delta_e = target_ned[1] - vehicle_ned[1]
    delta_d = target_ned[2] - vehicle_ned[2]
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    body_x = cos_yaw * delta_n + sin_yaw * delta_e
    body_y = -sin_yaw * delta_n + cos_yaw * delta_e
    body_x += 0.003 * math.sin(3.1 * t)
    body_y += 0.003 * math.cos(2.7 * t)
    measured_n = cos_yaw * body_x - sin_yaw * body_y
    measured_e = sin_yaw * body_x + cos_yaw * body_y
    return (
        vehicle_ned[0] + measured_n,
        vehicle_ned[1] + measured_e,
        vehicle_ned[2] + delta_d,
    )


def drain_state(connection, state: dict) -> None:
    while True:
        message = connection.recv_match(blocking=False)
        if message is None:
            return
        message_type = message.get_type()
        if message_type == "GLOBAL_POSITION_INT":
            state["position"] = global_to_local_ned(message, state["origin"])
        elif message_type == "ATTITUDE":
            state["yaw"] = float(message.yaw)
        elif message_type == "HEARTBEAT" and message.get_srcSystem() == 1:
            state["mode"] = mavutil.mode_string_v10(message).upper()
            state["armed"] = bool(
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )


def send_velocity(connection, t: float, velocity: tuple[float, float, float]) -> None:
    setpoint = GuidedVelocitySetpoint(
        time_boot_ms=int(t * 1000.0) & 0xFFFFFFFF,
        vx_mps=velocity[0],
        vy_mps=velocity[1],
        vz_mps=velocity[2],
        yaw_rate_rad_s=0.0,
    )
    connection.mav.send(
        make_message(
            setpoint,
            target_system=connection.target_system,
            target_component=connection.target_component,
            max_speed_mps=MAX_COMMAND_SPEED_MPS,
        )
    )


def land_and_wait(connection, timeout_s: float = 30.0) -> bool:
    try:
        set_mode(connection, "LAND")
    except RuntimeError:
        return False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and not (
            heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            return True
    return False


def run() -> tuple[list[dict], dict]:
    if SITL_ENDPOINT != "udpin:127.0.0.1:14550":
        raise RuntimeError("non-local MAVLink endpoints are forbidden in this test")
    connection = mavutil.mavlink_connection(
        SITL_ENDPOINT,
        source_system=250,
        source_component=191,
        dialect="common",
    )
    heartbeat = connection.wait_heartbeat(timeout=20)
    if heartbeat is None or connection.target_system != 1:
        raise RuntimeError("local ArduCopter SITL heartbeat not received")
    print("SITL_ONLY_ENDPOINT_OK udpin:127.0.0.1:14550")

    for message_id in (0, 30, 33):
        set_message_interval(connection, message_id, 20.0)
    send_gcs_heartbeat(connection)
    request_sitl_streams(connection)
    initial_global = wait_global_position(connection, timeout_s=30.0)
    origin = (int(initial_global.lat), int(initial_global.lon))
    set_mode(connection, "GUIDED")
    arm_sitl_when_ready(connection)
    print("SITL_ARMED=1 PHYSICAL_VEHICLE_CONNECTED=0")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TAKEOFF_ALT_M,
    )

    deadline = time.monotonic() + 20.0
    takeoff_position = None
    while time.monotonic() < deadline:
        position = wait_global_position(connection, timeout_s=1.0)
        local_position = global_to_local_ned(position, origin)
        if -local_position[2] >= TAKEOFF_ALT_M - 0.35:
            takeoff_position = local_position
            break
    if takeoff_position is None:
        raise RuntimeError("SITL takeoff altitude was not reached")
    print(f"SITL_TAKEOFF_OK altitude_m={-takeoff_position[2]:.3f}")

    tracker = AlphaBetaTargetTracker()
    controller = HorizontalFollowController(
        max_speed_mps=MAX_COMMAND_SPEED_MPS,
        max_accel_mps2=0.20,
    )
    state_machine = FollowSafetyStateMachine(predict_s=0.25, hold_s=0.70)
    state = {
        "position": takeoff_position,
        "origin": origin,
        "yaw": 0.0,
        "mode": "GUIDED",
        "armed": True,
    }
    records: list[dict] = []
    state_counts: Counter[str] = Counter()
    last_accepted_time = -1e9
    latest_track = None
    max_speed = 0.0
    max_follow_error = 0.0
    hold_observed = False
    disabled_observed = False
    pilot_override_observed = False
    resumed_after_hold = False
    hold_has_occurred = False
    loiter_commanded = False
    guided_restored = False
    started = time.monotonic()
    next_tick = started

    while True:
        now = time.monotonic()
        t = now - started
        if t > CONTROL_DURATION_S:
            break
        if now < next_tick:
            time.sleep(next_tick - now)
            now = time.monotonic()
            t = now - started
        next_tick += 1.0 / RATE_HZ
        drain_state(connection, state)

        if t >= 25.0 and not loiter_commanded:
            set_mode(connection, "BRAKE")
            state["mode"] = "BRAKE"
            loiter_commanded = True
        if t >= 27.0 and loiter_commanded and not guided_restored:
            set_mode(connection, "GUIDED")
            state["mode"] = "GUIDED"
            guided_restored = True

        true_target = target_position(t)
        visible = target_visible(t)
        vehicle_position = state["position"]
        if visible:
            measurement = body_measurement_to_local_target(
                vehicle_position,
                true_target,
                float(state["yaw"]),
                t,
            )
            candidate = tracker.update(
                TargetMeasurement(t, measurement, 65.0, 0, 0.5)
            )
            if candidate.accepted:
                latest_track = candidate
                last_accepted_time = t
        predicted = tracker.predict(t)
        if predicted is not None:
            latest_track = predicted

        target_age = t - last_accepted_time
        acquired = latest_track is not None and latest_track.acquired
        altitude_m = -float(vehicle_position[2])
        decision = state_machine.update(
            FollowInputs(
                timestamp_s=t,
                armed=bool(state["armed"]),
                mode=str(state["mode"]),
                rc_enable=rc_enabled(t),
                ekf_position_ok=True,
                battery_ok=True,
                altitude_ok=1.0 <= altitude_m <= 5.0,
                target_acquired=acquired,
                target_age_s=target_age,
            )
        )
        state_counts[decision.state.value] += 1
        hold_observed |= decision.state.value == "HOLD"
        disabled_observed |= decision.state.value == "DISABLED"
        pilot_override_observed |= decision.state.value == "PILOT_OVERRIDE"
        hold_has_occurred |= decision.state.value == "HOLD"
        if hold_has_occurred and t > 20.0 and decision.state.value == "FOLLOW_XY":
            resumed_after_hold = True

        velocity = (0.0, 0.0, 0.0)
        if decision.may_send_velocity and latest_track is not None:
            command = controller.update(
                timestamp_s=t,
                vehicle_position_ned_m=(vehicle_position[0], vehicle_position[1]),
                target_position_ned_m=(latest_track.position_m[0], latest_track.position_m[1]),
                target_velocity_ned_mps=(latest_track.velocity_mps[0], latest_track.velocity_mps[1]),
                velocity_scale=decision.velocity_scale,
            )
            velocity = command.velocity_ned_mps
            send_velocity(connection, t, velocity)
        else:
            controller.reset()

        speed = math.hypot(velocity[0], velocity[1])
        error = math.hypot(
            true_target[0] - vehicle_position[0],
            true_target[1] - vehicle_position[1],
        )
        max_speed = max(max_speed, speed)
        if decision.state.value == "FOLLOW_XY":
            max_follow_error = max(max_follow_error, error)
        records.append(
            {
                "time_s": round(t, 6),
                "target_visible": visible,
                "target_age_s": target_age,
                "rc_enable": rc_enabled(t),
                "autopilot_mode": state["mode"],
                "follow_state": decision.state.value,
                "reason": decision.reason,
                "vehicle_position_ned_m": list(vehicle_position),
                "true_target_position_ned_m": list(true_target),
                "velocity_setpoint_ned_mps": list(velocity),
                "horizontal_error_m": error,
            }
        )

    landed_disarmed = land_and_wait(connection)
    passed = all(
        (
            max_speed <= MAX_COMMAND_SPEED_MPS + 1e-6,
            max_follow_error <= MAX_FOLLOW_ERROR_M,
            hold_observed,
            disabled_observed,
            pilot_override_observed,
            resumed_after_hold,
            landed_disarmed,
        )
    )
    summary = {
        "scope": "local_arducopter_sitl_only",
        "endpoint": SITL_ENDPOINT,
        "physical_vehicle_connected": False,
        "duration_s": CONTROL_DURATION_S,
        "rate_hz": RATE_HZ,
        "records": len(records),
        "state_counts": dict(state_counts),
        "max_command_speed_mps": max_speed,
        "max_follow_error_m": max_follow_error,
        "speed_limit_mps": MAX_COMMAND_SPEED_MPS,
        "follow_error_limit_m": MAX_FOLLOW_ERROR_M,
        "target_loss_hold_observed": hold_observed,
        "rc_disable_observed": disabled_observed,
        "pilot_mode_override_observed": pilot_override_observed,
        "follow_resumed_after_rc_cycle": resumed_after_hold,
        "landed_and_disarmed": landed_disarmed,
        "passed": passed,
    }
    return records, summary


def main() -> int:
    records: list[dict] = []
    summary: dict = {}
    try:
        records, summary = run()
    finally:
        OUTPUT_JSONL.parent.mkdir(parents=True, exist_ok=True)
        if records:
            with OUTPUT_JSONL.open("w", encoding="utf-8") as output_file:
                for record in records:
                    output_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        if summary:
            OUTPUT_SUMMARY.write_text(
                json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("SIMULATION_ONLY=1 REAL_PIXHAWK_COMMANDS=0")
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
