#!/usr/bin/env python3
"""Read-only optical-flow scale fit using FC-processed flow and body rates."""

from __future__ import annotations

import argparse
import math
import statistics
import time

from pymavlink import mavutil


def correlation(left: list[float], right: list[float]) -> float | None:
    if len(left) < 20 or len(left) != len(right):
        return None
    lm = statistics.fmean(left)
    rm = statistics.fmean(right)
    ld = [value - lm for value in left]
    rd = [value - rm for value in right]
    denominator = math.sqrt(sum(v * v for v in ld) * sum(v * v for v in rd))
    if denominator <= 1.0e-9:
        return None
    return sum(a * b for a, b in zip(ld, rd)) / denominator


def fit(flow: list[float], body: list[float]) -> tuple[float | None, float | None, float | None]:
    if len(flow) < 20 or len(flow) != len(body):
        return None, None, None
    denominator = sum(value * value for value in flow)
    if denominator <= 1.0e-9:
        return None, None, None
    scalar = -sum(f * b for f, b in zip(flow, body)) / denominator
    residuals = [b + scalar * f for f, b in zip(flow, body)]
    rms = math.sqrt(statistics.fmean(value * value for value in residuals))
    corr = correlation(flow, [-value for value in body])
    return scalar, rms, corr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--min-rate", type=float, default=0.35)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=194,
        autoreconnect=False,
    )
    heartbeat = link.wait_heartbeat(timeout=8)
    if heartbeat is None or heartbeat.get_srcSystem() != 1:
        raise SystemExit("No Pixhawk heartbeat")
    if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("Vehicle is ARMED; scale probe refused")

    for message_id, interval_us in (
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 20_000),
    ):
        link.mav.command_long_send(
            1,
            1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    print(
        f"MODE={mavutil.mode_string_v10(heartbeat)} ARMED=0 "
        "ACTION=ROLL then PITCH +/-15deg; keep optical centre and yaw fixed",
        flush=True,
    )
    for remaining in range(args.countdown, 0, -1):
        print(f"START_IN={remaining}", flush=True)
        time.sleep(1)
    print("CAPTURE_STARTED", flush=True)

    latest_attitude = None
    latest_attitude_time = 0.0
    fc_flow_count = 0
    qualities: list[int] = []
    roll_flow: list[float] = []
    roll_body: list[float] = []
    pitch_flow: list[float] = []
    pitch_body: list[float] = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.2)
        if message is None:
            continue
        now = time.monotonic()
        mtype = message.get_type()
        if mtype == "HEARTBEAT" and message.get_srcSystem() == 1:
            if int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                raise SystemExit("Vehicle became ARMED; scale probe aborted")
        elif mtype == "ATTITUDE" and message.get_srcSystem() == 1:
            latest_attitude = message
            latest_attitude_time = now
        elif (
            mtype == "OPTICAL_FLOW"
            and message.get_srcSystem() == 1
            and latest_attitude is not None
            and now - latest_attitude_time <= 0.08
        ):
            fc_flow_count += 1
            qualities.append(int(message.quality))
            roll_rate = float(latest_attitude.rollspeed)
            pitch_rate = float(latest_attitude.pitchspeed)
            # ArduPilot MAVLink1 output contains flowRate-bodyRate in these
            # float fields.  Add synchronized body rate to reconstruct flowRate.
            flow_x = float(message.flow_comp_m_x) + roll_rate
            flow_y = float(message.flow_comp_m_y) + pitch_rate
            if abs(roll_rate) >= args.min_rate and abs(roll_rate) >= 1.4 * abs(pitch_rate):
                roll_flow.append(flow_x)
                roll_body.append(roll_rate)
            if abs(pitch_rate) >= args.min_rate and abs(pitch_rate) >= 1.4 * abs(roll_rate):
                pitch_flow.append(flow_y)
                pitch_body.append(pitch_rate)

    scalar_x, rms_x, corr_x = fit(roll_flow, roll_body)
    scalar_y, rms_y, corr_y = fit(pitch_flow, pitch_body)
    print("CAPTURE_FINISHED")
    print(f"FC_FLOW_MESSAGES={fc_flow_count}")
    if qualities:
        print(
            f"QUALITY=min:{min(qualities)} median:{statistics.median(qualities):.1f} "
            f"max:{max(qualities)}"
        )
    print(f"X_SAMPLES={len(roll_flow)} X_SCALAR={scalar_x} X_RMS={rms_x} X_CORR={corr_x}")
    print(f"Y_SAMPLES={len(pitch_flow)} Y_SCALAR={scalar_y} Y_RMS={rms_y} Y_CORR={corr_y}")
    if scalar_x is not None:
        print(f"CANDIDATE_FLOW_FXSCALER={(scalar_x - 1.0) * 1000.0:.1f}")
    if scalar_y is not None:
        print(f"CANDIDATE_FLOW_FYSCALER={(scalar_y - 1.0) * 1000.0:.1f}")
    strong = (
        scalar_x is not None
        and scalar_y is not None
        and corr_x is not None
        and corr_y is not None
        and len(roll_flow) >= 20
        and len(pitch_flow) >= 20
        and corr_x >= 0.65
        and corr_y >= 0.65
        and 0.2 <= scalar_x <= 4.0
        and 0.2 <= scalar_y <= 4.0
    )
    print("RESULT=STRONG" if strong else "RESULT=INSUFFICIENT_OR_POOR_FIT")
    print("READ_ONLY=1 PARAMETER_WRITE=0 ARM_COMMAND=0 MODE_CHANGE=0")
    return 0 if strong else 2


if __name__ == "__main__":
    raise SystemExit(main())
