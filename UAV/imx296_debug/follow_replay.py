#!/usr/bin/env python3
"""Deterministic offline moving-target replay for the follow stack.

No serial, UDP, camera, arm, mode, or actuator interface is opened.  The
result contains proposed GUIDED velocity setpoints only.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path

from follow_controller import HorizontalFollowController
from follow_state_machine import FollowInputs, FollowSafetyStateMachine
from mavlink_guided_velocity import GuidedVelocitySetpoint, pack_message
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement


def true_target_position(t: float) -> tuple[float, float, float]:
    # Stationary acquisition, then a 0.10 m/s diagonal moving target.
    moving_t = max(0.0, t - 1.0)
    return 0.10 * moving_t, 0.04 * moving_t, 0.80


def target_visible(t: float) -> bool:
    # A deliberate one-second outage verifies deceleration and latched HOLD.
    return not (5.0 <= t < 6.0)


def rc_enabled(t: float) -> bool:
    # The pilot cycles the enable switch after the lost-target HOLD.
    return not (6.5 <= t < 6.8)


def simulate(duration_s: float = 10.0, rate_hz: float = 10.0) -> tuple[list[dict], dict]:
    tracker = AlphaBetaTargetTracker()
    controller = HorizontalFollowController()
    state_machine = FollowSafetyStateMachine()
    dt = 1.0 / rate_hz
    vehicle_position = [0.0, 0.0]
    last_accepted_time = -1e9
    latest_track = None
    records = []
    state_counts = Counter()
    max_speed = 0.0
    max_error = 0.0

    steps = int(round(duration_s * rate_hz)) + 1
    for index in range(steps):
        t = index * dt
        visible = target_visible(t)
        if visible:
            true_position = true_target_position(t)
            # Small deterministic image/pose noise; no random seed required.
            measured = (
                true_position[0] + 0.003 * math.sin(t * 3.1),
                true_position[1] + 0.003 * math.cos(t * 2.7),
                true_position[2],
            )
            candidate = tracker.update(TargetMeasurement(t, measured, 60.0, 0, 0.5))
            if candidate.accepted:
                latest_track = candidate
                last_accepted_time = t
        predicted = tracker.predict(t)
        if predicted is not None:
            latest_track = predicted
        target_age = t - last_accepted_time
        acquired = latest_track is not None and latest_track.acquired
        decision = state_machine.update(
            FollowInputs(
                timestamp_s=t,
                armed=True,
                mode="GUIDED",
                rc_enable=rc_enabled(t),
                ekf_position_ok=True,
                battery_ok=True,
                altitude_ok=True,
                target_acquired=acquired,
                target_age_s=target_age,
            )
        )
        state_counts[decision.state.value] += 1

        if decision.may_send_velocity and latest_track is not None:
            command = controller.update(
                timestamp_s=t,
                vehicle_position_ned_m=(vehicle_position[0], vehicle_position[1]),
                target_position_ned_m=(
                    latest_track.position_m[0],
                    latest_track.position_m[1],
                ),
                target_velocity_ned_mps=(
                    latest_track.velocity_mps[0],
                    latest_track.velocity_mps[1],
                ),
                velocity_scale=decision.velocity_scale,
            )
            velocity = command.velocity_ned_mps
        else:
            controller.reset()
            velocity = (0.0, 0.0, 0.0)

        speed = math.hypot(velocity[0], velocity[1])
        max_speed = max(max_speed, speed)
        vehicle_position[0] += velocity[0] * dt
        vehicle_position[1] += velocity[1] * dt
        true_position = true_target_position(t)
        error = math.hypot(
            true_position[0] - vehicle_position[0],
            true_position[1] - vehicle_position[1],
        )
        max_error = max(max_error, error)

        setpoint = GuidedVelocitySetpoint(
            time_boot_ms=int(round(t * 1000.0)),
            vx_mps=velocity[0],
            vy_mps=velocity[1],
            vz_mps=0.0,
            yaw_rate_rad_s=0.0,
        )
        packet_hex = pack_message(setpoint, max_speed_mps=0.2).hex()
        records.append(
            {
                "time_s": t,
                "target_visible": visible,
                "target_age_s": target_age,
                "rc_enable": rc_enabled(t),
                "state": decision.state.value,
                "reason": decision.reason,
                "velocity_scale": decision.velocity_scale,
                "vehicle_position_ned_m": list(vehicle_position),
                "true_target_position_ned_m": list(true_position[:2]),
                "velocity_setpoint_ned_mps": list(velocity),
                "mavlink_v2_hex": packet_hex,
            }
        )

    summary = {
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "records": len(records),
        "state_counts": dict(state_counts),
        "max_command_speed_mps": max_speed,
        "max_horizontal_error_m": max_error,
        "target_outage_s": 1.0,
        "hold_observed": state_counts["HOLD"] > 0,
        "rc_reenable_observed": state_counts["DISABLED"] > 0,
        "real_mavlink_sent": False,
    }
    return records, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()
    if args.duration_s <= 0 or args.rate_hz <= 0:
        raise ValueError("duration and rate must be positive")
    records, summary = simulate(args.duration_s, args.rate_hz)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output_file:
        for record in records:
            output_file.write(json.dumps(record, ensure_ascii=False) + "\n")
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("DRY_RUN=1 SERIAL_OPENED=0 CONTROL_COMMAND_SENT=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
