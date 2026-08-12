#!/usr/bin/env python3
"""Controlled real-Pixhawk no-prop command-path bench test.

The script never force-arms, never sends takeoff/land while armed, and always
attempts to return the vehicle to disarmed STABILIZE before exiting.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


def is_real_fc(message) -> bool:
    return (
        message is not None
        and message.get_srcSystem() == 1
        and message.get_srcComponent() == 1
    )


def is_armed(message) -> bool:
    return bool(
        int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def wait_heartbeats(link, count: int, require_armed: bool | None, timeout_s: float = 15.0):
    records = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and len(records) < count:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_real_fc(message):
            continue
        armed = is_armed(message)
        record = {
            "armed": armed,
            "base_mode": int(message.base_mode),
            "custom_mode": int(message.custom_mode),
            "system_status": int(message.system_status),
        }
        records.append(record)
        print(f"HEARTBEAT_{len(records)}={record}")
        if require_armed is not None and armed != require_armed:
            return records, False
    return records, len(records) == count


def send_command(link, command: int, params: list[float], timeout_s: float = 5.0):
    link.mav.command_long_send(1, 1, command, 0, *params)
    deadline = time.monotonic() + timeout_s
    status_texts = []
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "STATUSTEXT":
            value = message.text
            if isinstance(value, bytes):
                value = value.decode("utf-8", errors="replace")
            status_texts.append(str(value).rstrip("\x00"))
        elif (
            message.get_type() == "COMMAND_ACK"
            and int(message.command) == command
        ):
            return int(message.result), status_texts
    return None, status_texts


def set_stabilize(link) -> bool:
    link.mav.set_mode_send(
        1,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
    )
    consecutive = 0
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and consecutive < 3:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_real_fc(message):
            continue
        armed = is_armed(message)
        custom_mode = int(message.custom_mode)
        print(
            f"STABILIZE_CONFIRM armed={int(armed)} custom_mode={custom_mode} "
            f"consecutive={consecutive}"
        )
        if not armed and custom_mode == 0:
            consecutive += 1
        else:
            consecutive = 0
    return consecutive == 3


def disarm_and_confirm(link) -> tuple[int | None, bool, list[str]]:
    ack, texts = send_command(
        link,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    _records, complete = wait_heartbeats(link, 3, require_armed=False, timeout_s=10.0)
    return ack, complete, texts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--armed-hold-s", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        print("SAFETY_STOP=EXECUTE_FLAG_REQUIRED")
        return 2
    if not 0.5 <= args.armed_hold_s <= 3.0:
        print("SAFETY_STOP=ARMED_HOLD_OUT_OF_RANGE")
        return 2

    result = {
        "scope": "real_pixhawk_propellers_removed_bench",
        "force_arm": False,
        "takeoff_or_land_while_armed": False,
        "armed_hold_s_requested": args.armed_hold_s,
    }
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    initial, initial_ok = wait_heartbeats(link, 5, require_armed=False)
    result["initial_heartbeats"] = initial
    if not initial_ok:
        print("SAFETY_STOP=INITIAL_STATE_NOT_DISARMED")
        return 3
    if not all(record["system_status"] == mavutil.mavlink.MAV_STATE_STANDBY for record in initial):
        print("SAFETY_STOP=FC_NOT_STANDBY")
        return 3

    # Collect the latest power reports without requesting a stream change.
    power_deadline = time.monotonic() + 4.0
    while time.monotonic() < power_deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "SYS_STATUS":
            result["battery"] = {
                "voltage_v": float(message.voltage_battery) / 1000.0,
                "current_a": None
                if int(message.current_battery) < 0
                else float(message.current_battery) / 100.0,
            }
        elif message.get_type() == "POWER_STATUS":
            result["power"] = {
                "vcc_v": float(message.Vcc) / 1000.0,
                "vservo_v": float(message.Vservo) / 1000.0,
                "flags": int(message.flags),
            }
        if "battery" in result and "power" in result:
            break
    print(f"POWER={json.dumps({k: result[k] for k in ('battery', 'power') if k in result})}")

    result["stabilize_before_arm"] = set_stabilize(link)
    if not result["stabilize_before_arm"]:
        print("SAFETY_STOP=STABILIZE_NOT_CONFIRMED")
        return 4

    armed_seen = False
    try:
        arm_ack, arm_texts = send_command(
            link,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        result["arm_ack"] = arm_ack
        result["arm_statustext"] = arm_texts
        print(f"ARM_ACK={arm_ack} FORCE=0 STATUSTEXT={arm_texts}")
        arm_deadline = time.monotonic() + 5.0
        while time.monotonic() < arm_deadline:
            heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if is_real_fc(heartbeat) and is_armed(heartbeat):
                armed_seen = True
                break
        result["armed_seen"] = armed_seen
        if armed_seen:
            print("ARMED_CONFIRMED=1 MOTOR_OUTPUT_WINDOW_STARTED=1")
            hold_deadline = time.monotonic() + args.armed_hold_s
            while time.monotonic() < hold_deadline:
                message = link.recv_match(blocking=True, timeout=0.2)
                if message is not None and message.get_srcSystem() == 1:
                    if message.get_type() == "SERVO_OUTPUT_RAW":
                        result.setdefault("servo_output_samples", []).append(
                            [int(getattr(message, f"servo{i}_raw")) for i in range(1, 5)]
                        )
                    elif message.get_type() == "HEARTBEAT" and not is_armed(message):
                        break
        else:
            print("ARMED_CONFIRMED=0 MOTOR_OUTPUT_WINDOW_STARTED=0")
    finally:
        disarm_ack, disarmed, disarm_texts = disarm_and_confirm(link)
        result["disarm_ack"] = disarm_ack
        result["disarm_confirmed"] = disarmed
        result["disarm_statustext"] = disarm_texts
        print(f"DISARM_ACK={disarm_ack} DISARMED_CONFIRMED={int(disarmed)}")
        if not disarmed:
            args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            print("SAFETY_STOP=DISARM_NOT_CONFIRMED")
            return 5

    # Navigation command transport is tested only while disarmed. No command is
    # sent while armed, and STABILIZE is restored after each attempt.
    for label, command, params in (
        (
            "takeoff_disarmed",
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 1.0],
        ),
        (
            "land_disarmed",
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 0.0],
        ),
    ):
        ack, texts = send_command(link, command, params)
        result[label] = {"ack": ack, "statustext": texts}
        print(f"{label.upper()}_ACK={ack} STATUSTEXT={texts}")
        if not set_stabilize(link):
            print(f"SAFETY_STOP=STABILIZE_RESTORE_FAILED_AFTER_{label}")
            return 6

    final, final_ok = wait_heartbeats(link, 5, require_armed=False)
    result["final_heartbeats"] = final
    result["final_disarmed_stabilize"] = final_ok and all(
        record["custom_mode"] == 0 for record in final
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"FINAL_DISARMED_STABILIZE={int(result['final_disarmed_stabilize'])}")
    print(f"OUTPUT={args.output}")
    print("FORCE_ARM=0 TAKEOFF_WHILE_ARMED=0 LAND_WHILE_ARMED=0")
    return 0 if result["final_disarmed_stabilize"] else 7


if __name__ == "__main__":
    raise SystemExit(main())
