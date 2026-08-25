#!/usr/bin/env python3
"""Read-only optical-flow/rangefinder live check for a connected Pixhawk."""

from __future__ import annotations

import argparse
import statistics
import time
from collections import Counter

from pymavlink import mavutil


PARAMS = (
    "FLOW_TYPE",
    "FLOW_FXSCALER",
    "FLOW_FYSCALER",
    "FLOW_ORIENT_YAW",
    "RNGFND1_TYPE",
    "RNGFND1_ORIENT",
    "RNGFND1_MIN",
    "RNGFND1_MAX",
    "RNGFND1_GNDCLR",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
)


def pname(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def read_param(link, name: str) -> float | None:
    for _ in range(3):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is not None and pname(msg) == name:
                return float(msg.param_value)
    return None


def stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"n": 0, "min": None, "mean": None, "max": None}
    return {
        "n": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "max": max(values),
    }


def print_axis_stats(label: str, values: list[float]) -> None:
    print(f"  {label}={stats(values)}")
    if len(values) >= 4:
        half = len(values) // 2
        print(f"  {label}_first_half_mean={statistics.fmean(values[:half])}")
        print(f"  {label}_second_half_mean={statistics.fmean(values[half:])}")
        print(f"  {label}_sum={sum(values)}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=250,
        source_component=191,
        autoreconnect=False,
    )
    hb = link.wait_heartbeat(timeout=10)
    if hb is None:
        raise SystemExit("No heartbeat")
    armed = bool(int(hb.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        raise SystemExit("Vehicle is ARMED; live check refused")

    link.target_system = hb.get_srcSystem()
    link.target_component = hb.get_srcComponent()
    link.mav.request_data_stream_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )

    print(f"MODE={mavutil.mode_string_v10(hb)} ARMED=0")
    print("PARAMETERS")
    for name in PARAMS:
        print(f"  {name}={read_param(link, name)}")

    counts: Counter[str] = Counter()
    quality: list[float] = []
    distance_m: list[float] = []
    ground_distance_m: list[float] = []
    flow_x: list[float] = []
    flow_y: list[float] = []
    latest_ekf = None
    deadline = time.monotonic() + args.seconds

    while time.monotonic() < deadline:
        msg = link.recv_match(blocking=True, timeout=0.25)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        mtype = msg.get_type()
        counts[mtype] += 1
        if mtype == "OPTICAL_FLOW":
            quality.append(float(msg.quality))
            flow_x.append(float(msg.flow_x))
            flow_y.append(float(msg.flow_y))
            if float(msg.ground_distance) >= 0:
                ground_distance_m.append(float(msg.ground_distance))
        elif mtype == "DISTANCE_SENSOR":
            distance_m.append(float(msg.current_distance) / 100.0)
        elif mtype == "EKF_STATUS_REPORT":
            latest_ekf = msg

    print("SAMPLES")
    print(f"  OPTICAL_FLOW={counts['OPTICAL_FLOW']} DISTANCE_SENSOR={counts['DISTANCE_SENSOR']}")
    print(f"  quality={stats(quality)}")
    print(f"  distance_m={stats(distance_m)}")
    print(f"  optical_flow_ground_distance_m={stats(ground_distance_m)}")
    print_axis_stats("raw_flow_x", flow_x)
    print_axis_stats("raw_flow_y", flow_y)
    if latest_ekf is not None:
        print(
            "  ekf="
            f"flags={latest_ekf.flags} "
            f"vel_var={latest_ekf.velocity_variance:.4f} "
            f"pos_h_var={latest_ekf.pos_horiz_variance:.4f} "
            f"pos_v_var={latest_ekf.pos_vert_variance:.4f}"
        )
    print("READ_ONLY=1 ARM_COMMAND=0 MODE_CHANGE=0 PARAMETER_WRITE=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
