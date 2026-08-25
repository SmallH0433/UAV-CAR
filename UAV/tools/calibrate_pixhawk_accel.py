#!/usr/bin/env python3
"""Stepwise six-position ArduPilot accelerometer calibration over MAVLink."""

import argparse
import time

from pymavlink import mavutil


POSITION_NAMES = {
    1: "LEVEL（正常水平放置，机脚朝下）",
    2: "LEFT（机体左侧朝下）",
    3: "RIGHT（机体右侧朝下）",
    4: "NOSE DOWN（机头垂直朝下）",
    5: "NOSE UP（机头垂直朝上）",
    6: "BACK（机背朝下、机脚朝上）",
}
SUCCESS = 16777215
FAILED = 16777216


def send_position(link, position):
    link.mav.command_int_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
        0,
        0,
        float(position),
        0,
        0,
        0,
        0,
        0,
        0,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--start", action="store_true")
    action.add_argument("--position", type=int, choices=range(1, 7))
    action.add_argument("--listen", action="store_true")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=250,
        source_component=190,
    )
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit("ERROR: no Pixhawk heartbeat")
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise SystemExit("ERROR: vehicle is armed; calibration refused")

    print(f"Connected: sys={link.target_system} comp={link.target_component}; DISARMED")
    if args.start:
        print("Starting six-position accelerometer calibration...")
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0,
            0,
            0,
            0,
            0,
            1,
            0,
            0,
        )
    elif args.position is not None:
        print(f"Acknowledging position {args.position}: {POSITION_NAMES[args.position]}")
        send_position(link, args.position)
    else:
        print("Listening for current calibration state...")

    deadline = time.time() + 25
    while time.time() < deadline:
        msg = link.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(f"FC: {msg.text}")
            continue
        if msg_type != "COMMAND_LONG" or msg.command != mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
            continue

        position = int(round(msg.param1))
        if position == SUCCESS:
            print("CALIBRATION_SUCCESS")
            return
        if position == FAILED:
            raise SystemExit("CALIBRATION_FAILED")
        if position not in POSITION_NAMES:
            print(f"Ignoring unknown calibration position: {position}")
            continue
        print(f"NEXT_POSITION={position}: {POSITION_NAMES[position]}")
        return

    raise SystemExit("NO_CALIBRATION_REQUEST_WITHIN_TIMEOUT")


if __name__ == "__main__":
    main()
