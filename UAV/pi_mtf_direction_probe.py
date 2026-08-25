#!/usr/bin/env python3
"""Read-only MTF-01P yaw-direction probe over the Pi-to-Pixhawk UART.

Run while disarmed with the optical centre held fixed above a textured floor.
Rock roll and pitch separately.  The probe compares the raw MTF flow axes with
Pixhawk body angular rates and scores 0/+90/-90/180 degree corrections.
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


def correlation(left: list[float], right: list[float]) -> float | None:
    if len(left) < 20 or len(right) != len(left):
        return None
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    left_delta = [value - left_mean for value in left]
    right_delta = [value - right_mean for value in right]
    denominator = math.sqrt(
        sum(value * value for value in left_delta)
        * sum(value * value for value in right_delta)
    )
    if denominator <= 1.0e-9:
        return None
    return sum(a * b for a, b in zip(left_delta, right_delta)) / denominator


def rotate(x_value: float, y_value: float, degrees: int) -> tuple[float, float]:
    angle = math.radians(degrees)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        x_value * cosine - y_value * sine,
        x_value * sine + y_value * cosine,
    )


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def read_parameter(link, name: str) -> float | None:
    for _ in range(3):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if message is not None and parameter_name(message) == name:
                return float(message.param_value)
    return None


def normalize_centidegrees(value: int) -> int:
    while value > 18000:
        value -= 36000
    while value <= -18000:
        value += 36000
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=14.0)
    parser.add_argument("--countdown", type=int, default=3)
    parser.add_argument("--min-quality", type=int, default=60)
    parser.add_argument("--min-axis-rate", type=float, default=0.20)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=191,
        source_component=192,
    )
    heartbeat = None
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is not None
            and message.get_srcSystem() == 1
            and message.get_srcComponent() == 1
        ):
            heartbeat = message
            break
    if heartbeat is None:
        raise SystemExit("No Pixhawk heartbeat")
    if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("Vehicle is ARMED; direction probe refused")

    current_orientation = read_parameter(link, "FLOW_ORIENT_YAW")
    link.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        20_000,
        0,
        0,
        0,
        0,
        0,
    )
    link.mav.request_data_stream_send(
        1,
        1,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        50,
        1,
    )

    print(
        f"MODE={mavutil.mode_string_v10(heartbeat)} ARMED=0 "
        f"FLOW_ORIENT_YAW={current_orientation}",
        flush=True,
    )
    print("ACTION=ROLL +/-15deg five times, then PITCH +/-15deg five times", flush=True)
    for remaining in range(args.countdown, 0, -1):
        print(f"START_IN={remaining}", flush=True)
        time.sleep(1)
    print("CAPTURE_STARTED", flush=True)

    latest_attitude = None
    latest_attitude_time = 0.0
    flow_messages = 0
    qualities: list[int] = []
    samples: list[tuple[float, float, float, float]] = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        now = time.monotonic()
        message_type = message.get_type()
        if message_type == "HEARTBEAT" and message.get_srcSystem() == 1:
            if int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                raise SystemExit("Vehicle became ARMED; capture aborted")
        elif message_type == "ATTITUDE" and message.get_srcSystem() == 1:
            latest_attitude = message
            latest_attitude_time = now
        elif message_type == "OPTICAL_FLOW" and message.get_srcSystem() == 200:
            flow_messages += 1
            quality = int(message.quality)
            qualities.append(quality)
            if (
                quality >= args.min_quality
                and latest_attitude is not None
                and now - latest_attitude_time <= 0.08
            ):
                samples.append(
                    (
                        float(message.flow_x),
                        float(message.flow_y),
                        float(latest_attitude.rollspeed),
                        float(latest_attitude.pitchspeed),
                    )
                )

    print("CAPTURE_FINISHED", flush=True)
    print(f"RAW_FLOW_MESSAGES={flow_messages}")
    if qualities:
        print(
            "QUALITY="
            f"min:{min(qualities)} median:{statistics.median(qualities):.1f} "
            f"max:{max(qualities)}"
        )
    print(f"SYNCHRONIZED_SAMPLES={len(samples)}")

    scores: dict[int, float] = {}
    details: dict[int, tuple[float | None, float | None, int, int]] = {}
    for candidate in (0, 90, -90, 180):
        roll_flow: list[float] = []
        roll_target: list[float] = []
        pitch_flow: list[float] = []
        pitch_target: list[float] = []
        for flow_x, flow_y, roll_rate, pitch_rate in samples:
            corrected_x, corrected_y = rotate(flow_x, flow_y, candidate)
            if (
                abs(roll_rate) >= args.min_axis_rate
                and abs(roll_rate) >= 1.4 * abs(pitch_rate)
            ):
                roll_flow.append(corrected_x)
                roll_target.append(-roll_rate)
            if (
                abs(pitch_rate) >= args.min_axis_rate
                and abs(pitch_rate) >= 1.4 * abs(roll_rate)
            ):
                pitch_flow.append(corrected_y)
                pitch_target.append(-pitch_rate)
        roll_corr = correlation(roll_flow, roll_target)
        pitch_corr = correlation(pitch_flow, pitch_target)
        valid = [value for value in (roll_corr, pitch_corr) if value is not None]
        scores[candidate] = statistics.fmean(valid) if len(valid) == 2 else -2.0
        details[candidate] = (
            roll_corr,
            pitch_corr,
            len(roll_flow),
            len(pitch_flow),
        )
        print(
            f"CORRECTION_{candidate:+d}="
            f"roll_corr:{roll_corr} pitch_corr:{pitch_corr} "
            f"roll_n:{len(roll_flow)} pitch_n:{len(pitch_flow)} "
            f"score:{scores[candidate]:.4f}"
        )

    ranked = sorted(scores, key=scores.get, reverse=True)
    best = ranked[0]
    second = ranked[1]
    best_roll, best_pitch, best_roll_n, best_pitch_n = details[best]
    strong = (
        best_roll is not None
        and best_pitch is not None
        and best_roll_n >= 20
        and best_pitch_n >= 20
        and best_roll >= 0.65
        and best_pitch >= 0.65
        and scores[best] - scores[second] >= 0.30
    )
    print(f"BEST_CORRECTION_DEG={best:+d}")
    if current_orientation is not None:
        candidate_value = normalize_centidegrees(round(current_orientation) + best * 100)
        print(f"CANDIDATE_FLOW_ORIENT_YAW={candidate_value}")
    print("RESULT=STRONG" if strong else "RESULT=AMBIGUOUS_OR_INSUFFICIENT")
    print("READ_ONLY=1 PARAMETER_WRITE=0 MODE_CHANGE=0 ARM_COMMAND=0")
    return 0 if strong else 2


if __name__ == "__main__":
    raise SystemExit(main())
