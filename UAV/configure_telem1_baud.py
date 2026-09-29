#!/usr/bin/env python3
"""Set Pixhawk TELEM1 to 115200 only while disarmed; back up before writing."""

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil

NAME = "SERIAL1_BAUD"
TARGET = 115.0


def read_param(link):
    link.mav.param_request_read_send(1, 1, NAME.encode("ascii"), -1)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and str(message.param_id).rstrip("\x00") == NAME:
            return float(message.param_value), int(message.param_type)
    raise RuntimeError(f"{NAME} not received")


def disarmed(link):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and heartbeat.get_srcSystem() == 1 and heartbeat.get_srcComponent() == 1:
            return not bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="COM10")
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--reboot", action="store_true")
    args = parser.parse_args()
    link = mavutil.mavlink_connection(args.device, baud=115200, autoreconnect=False)
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None or heartbeat.get_srcSystem() != 1 or heartbeat.get_srcComponent() != 1:
        raise SystemExit("FC_HEARTBEAT=NOT_RECEIVED")
    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
    before, param_type = read_param(link)
    args.backup.parent.mkdir(parents=True, exist_ok=True)
    args.backup.write_text(json.dumps({"parameter": NAME, "before": before, "type": param_type, "target": TARGET, "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=2), encoding="utf-8")
    print(f"BACKUP={args.backup} BEFORE={before} TYPE={param_type}")
    if not args.apply:
        return
    if not disarmed(link):
        raise SystemExit("APPLY_ABORTED=FC_ARMED_OR_HEARTBEAT_UNAVAILABLE")
    if before != TARGET:
        link.mav.param_set_send(1, 1, NAME.encode("ascii"), TARGET, param_type)
        time.sleep(0.5)
    after, _ = read_param(link)
    print(f"AFTER={after}")
    if after != TARGET:
        raise SystemExit("SET_FAILED")
    if args.reboot:
        if not disarmed(link):
            raise SystemExit("REBOOT_ABORTED=FC_ARMED_OR_HEARTBEAT_UNAVAILABLE")
        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        print("FC_REBOOT_REQUESTED")


if __name__ == "__main__":
    main()
