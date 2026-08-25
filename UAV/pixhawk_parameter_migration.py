#!/usr/bin/env python3
"""Controlled Pixhawk parameter reset/apply helper for the QAV280 migration.

The helper always refuses to operate on an armed vehicle.  It never changes
flight mode, arm state, missions, or actuator outputs.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil


def clean_name(value) -> str:
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


def connect(port: str, baud: int):
    link = mavutil.mavlink_connection(
        port,
        baud=baud,
        source_system=255,
        source_component=190,
        autoreconnect=False,
    )
    heartbeat = None
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if candidate is not None and int(candidate.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            heartbeat = candidate
            break
    if heartbeat is None:
        raise RuntimeError("No autopilot heartbeat received")
    armed = bool(int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        raise RuntimeError("Vehicle is ARMED; operation refused")
    link.target_system = heartbeat.get_srcSystem()
    link.target_component = heartbeat.get_srcComponent()
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    return link, heartbeat


def wait_command_ack(link, command: int, timeout: float = 6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.4)
        if message is None:
            continue
        if message.get_type() == "COMMAND_ACK" and int(message.command) == command:
            return int(message.result)
    return None


def request_parameter(link, name: str, timeout: float = 2.5):
    for _attempt in range(3):
        link.mav.param_request_read_send(
            link.target_system,
            link.target_component,
            name.encode("ascii"),
            -1,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.35)
            if message is not None and clean_name(message.param_id) == name:
                return float(message.param_value), int(message.param_type)
    return None


def set_parameter(link, name: str, value: float, tolerance: float = 1.0e-5):
    current = request_parameter(link, name)
    if current is None:
        return {"name": name, "requested": value, "status": "MISSING"}
    old_value, param_type = current
    if math.isclose(old_value, value, rel_tol=tolerance, abs_tol=tolerance):
        return {
            "name": name,
            "before": old_value,
            "requested": value,
            "after": old_value,
            "status": "UNCHANGED",
        }
    for _attempt in range(3):
        link.mav.param_set_send(
            link.target_system,
            link.target_component,
            name.encode("ascii"),
            float(value),
            param_type,
        )
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.35)
            if message is None or clean_name(message.param_id) != name:
                continue
            actual = float(message.param_value)
            if math.isclose(actual, value, rel_tol=tolerance, abs_tol=tolerance):
                return {
                    "name": name,
                    "before": old_value,
                    "requested": value,
                    "after": actual,
                    "status": "WRITTEN",
                }
    return {
        "name": name,
        "before": old_value,
        "requested": value,
        "status": "VERIFY_FAILED",
    }


def reboot(link) -> None:
    command = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        command,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    result = wait_command_ack(link, command, timeout=2.0)
    print(json.dumps({"reboot_command_ack": result}, ensure_ascii=False))


def factory_reset(link) -> int:
    command = mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        command,
        0,
        2,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    result = wait_command_ack(link, command)
    accepted = result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(json.dumps({"factory_reset_ack": result, "accepted": accepted}, ensure_ascii=False))
    if not accepted:
        return 3
    reboot(link)
    return 0


def apply_file(link, path: Path, result_path: Path | None, reboot_after: bool) -> int:
    document = json.loads(path.read_text(encoding="utf-8"))
    raw_parameters = document.get("parameters", document)
    if not isinstance(raw_parameters, dict):
        raise ValueError("Parameter file must contain an object named 'parameters'")
    results = []
    for name, raw_value in raw_parameters.items():
        results.append(set_parameter(link, str(name), float(raw_value)))
        print(json.dumps(results[-1], ensure_ascii=False))
    failed = [item for item in results if item["status"] in {"MISSING", "VERIFY_FAILED"}]
    summary = {
        "source_file": str(path.resolve()),
        "requested": len(results),
        "written": sum(item["status"] == "WRITTEN" for item in results),
        "unchanged": sum(item["status"] == "UNCHANGED" for item in results),
        "failed": len(failed),
        "results": results,
    }
    if result_path:
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: summary[key] for key in ("requested", "written", "unchanged", "failed")}, ensure_ascii=False))
    if failed:
        return 4
    if reboot_after:
        reboot(link)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    subparsers = parser.add_subparsers(dest="operation", required=True)
    subparsers.add_parser("factory-reset")
    apply_parser = subparsers.add_parser("apply")
    apply_parser.add_argument("--file", type=Path, required=True)
    apply_parser.add_argument("--result", type=Path)
    apply_parser.add_argument("--reboot", action="store_true")
    args = parser.parse_args()

    link, heartbeat = connect(args.port, args.baud)
    print(
        json.dumps(
            {
                "system": heartbeat.get_srcSystem(),
                "component": heartbeat.get_srcComponent(),
                "armed": False,
                "mode": mavutil.mode_string_v10(heartbeat),
            },
            ensure_ascii=False,
        )
    )
    try:
        if args.operation == "factory-reset":
            return factory_reset(link)
        return apply_file(link, args.file, args.result, args.reboot)
    finally:
        link.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"MIGRATION_FAILED: {error}", file=sys.stderr)
        raise
