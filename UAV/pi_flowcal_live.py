#!/usr/bin/env python3
"""Run ArduPilot's built-in optical-flow scale calibration while disarmed."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


FLOW_CAL_AUX_FUNCTION = 158
SWITCH_LOW = 0
SWITCH_HIGH = 2


def param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def read_param(link, name: str) -> float | None:
    for _ in range(3):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            message = link.recv_match(blocking=True, timeout=0.2)
            if message is not None and message.get_type() == "PARAM_VALUE":
                if param_name(message) == name:
                    return float(message.param_value)
    return None


def send_aux(link, switch_position: int) -> None:
    link.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
        0,
        FLOW_CAL_AUX_FUNCTION,
        switch_position,
        0,
        0,
        0,
        0,
        0,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--timeout", type=float, default=100.0)
    parser.add_argument("--countdown", type=int, default=5)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=191,
        source_component=193,
        autoreconnect=False,
    )
    heartbeat = link.wait_heartbeat(timeout=8)
    if heartbeat is None or heartbeat.get_srcSystem() != 1:
        raise SystemExit("No Pixhawk heartbeat")
    if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("Vehicle is ARMED; FlowCal refused")

    original_x = read_param(link, "FLOW_FXSCALER")
    original_y = read_param(link, "FLOW_FYSCALER")
    orientation = read_param(link, "FLOW_ORIENT_YAW")
    print(
        f"MODE={mavutil.mode_string_v10(heartbeat)} ARMED=0 "
        f"FLOW_ORIENT_YAW={orientation} "
        f"ORIGINAL_FX={original_x} ORIGINAL_FY={original_y}",
        flush=True,
    )
    print(
        "ACTION=Rock ROLL and PITCH +/-15deg repeatedly; keep yaw and optical centre fixed",
        flush=True,
    )
    for remaining in range(args.countdown, 0, -1):
        print(f"START_IN={remaining}", flush=True)
        time.sleep(1)

    started = False
    success = False
    failure = False
    try:
        send_aux(link, SWITCH_HIGH)
        print("FLOWCAL_START_COMMAND_SENT", flush=True)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            message = link.recv_match(blocking=True, timeout=0.25)
            if message is None:
                continue
            mtype = message.get_type()
            if mtype == "HEARTBEAT" and message.get_srcSystem() == 1:
                if int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    failure = True
                    print("ABORTED=vehicle_became_armed", flush=True)
                    break
            elif mtype == "COMMAND_ACK" and int(message.command) == 218:
                print(f"AUX_ACK_RESULT={int(message.result)}", flush=True)
            elif mtype == "STATUSTEXT" and message.get_srcSystem() == 1:
                text = str(message.text).rstrip("\x00")
                if "FlowCal:" not in text:
                    continue
                print(f"STATUS={text}", flush=True)
                if "Started" in text:
                    started = True
                if "FLOW_FXSCALER=" in text:
                    success = True
                    break
                if any(word in text for word in ("timeout", "failed", "too low", "too high")):
                    failure = True
                    break
    finally:
        send_aux(link, SWITCH_LOW)
        print("FLOWCAL_STOP_COMMAND_SENT", flush=True)
        time.sleep(0.5)

    final_x = read_param(link, "FLOW_FXSCALER")
    final_y = read_param(link, "FLOW_FYSCALER")
    print(f"FINAL_FX={final_x} FINAL_FY={final_y}")
    print(f"START_CONFIRMED={int(started)} SUCCESS={int(success)} FAILURE={int(failure)}")
    print("ARM_COMMAND=0 MODE_CHANGE=0")
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
