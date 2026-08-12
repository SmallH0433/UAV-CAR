#!/usr/bin/env python3
"""Safely run short, disarmed Pixhawk motor tests on a propeller-free bench.

This uses MAV_CMD_DO_MOTOR_TEST only.  It never arms the vehicle and never
sends mode, takeoff, landing, or actuator-override commands.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


FC_SYSTEM = 1
FC_COMPONENT = 1
GCS_SYSTEM = 255
GCS_COMPONENT = 191


def is_real_fc(message) -> bool:
    return (
        message is not None
        and message.get_srcSystem() == FC_SYSTEM
        and message.get_srcComponent() == FC_COMPONENT
    )


def is_armed(message) -> bool:
    return bool(
        int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def heartbeat_record(message) -> dict:
    return {
        "armed": is_armed(message),
        "base_mode": int(message.base_mode),
        "custom_mode": int(message.custom_mode),
        "system_status": int(message.system_status),
    }


def wait_disarmed_heartbeats(link, count: int, timeout_s: float) -> tuple[list, bool]:
    records = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and len(records) < count:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_real_fc(message):
            continue
        record = heartbeat_record(message)
        records.append(record)
        print(f"HEARTBEAT_{len(records)}={record}")
        if record["armed"]:
            return records, False
    return records, len(records) == count


def message_text(message) -> str:
    value = message.text
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


def run_motor_test(link, motor: int, throttle_percent: float, duration_s: float) -> dict:
    command = mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST
    record = {
        "motor": motor,
        "throttle_type": "percent",
        "throttle_percent": throttle_percent,
        "duration_s": duration_s,
        "ack": None,
        "statustext": [],
        "servo_output_raw": [],
        "transient_armed_during_test": False,
    }
    print(
        f"MOTOR_TEST_START motor={motor} throttle_percent={throttle_percent:.1f} "
        f"duration_s={duration_s:.1f}"
    )
    link.mav.command_long_send(
        FC_SYSTEM,
        FC_COMPONENT,
        command,
        0,
        float(motor),
        0.0,  # MOTOR_TEST_THROTTLE_PERCENT
        float(throttle_percent),
        float(duration_s),
        1.0,  # one motor
        0.0,  # default motor order
        0.0,
    )

    deadline = time.monotonic() + duration_s + 3.0
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.1)
        if message is None or message.get_srcSystem() != FC_SYSTEM:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT" and is_real_fc(message):
            if is_armed(message):
                # ArduPilot temporarily reports the armed flag while its
                # disarmed Motor Test command is actively driving an output.
                # The caller must confirm DISARMED again before another motor.
                record["transient_armed_during_test"] = True
        elif message_type == "STATUSTEXT":
            text = message_text(message)
            record["statustext"].append(text)
            print(f"STATUSTEXT={text}")
        elif message_type == "COMMAND_ACK" and int(message.command) == command:
            record["ack"] = int(message.result)
            print(f"MOTOR_TEST_ACK motor={motor} result={record['ack']}")
            if record["ack"] not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                break
        elif message_type == "SERVO_OUTPUT_RAW":
            record["servo_output_raw"].append(
                [int(getattr(message, f"servo{i}_raw")) for i in range(1, 5)]
            )

    # The autopilot stops the test at timeout.  Allow a short settling interval;
    # the caller independently requires DISARMED heartbeats before continuing.
    settle_deadline = time.monotonic() + 0.5
    while time.monotonic() < settle_deadline:
        message = link.recv_match(blocking=True, timeout=0.1)
        if is_real_fc(message) and message.get_type() == "HEARTBEAT" and is_armed(message):
            record["transient_armed_during_test"] = True
    record["accepted"] = (
        record["ack"]
        in (mavutil.mavlink.MAV_RESULT_ACCEPTED, mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
    )
    print(
        f"MOTOR_TEST_END motor={motor} accepted={int(record['accepted'])} "
        f"transient_armed={int(record['transient_armed_during_test'])}"
    )
    return record


def write_result(path: Path, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"OUTPUT={path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--motors", default="1,2,3,4")
    parser.add_argument("--throttle-percent", type=float, default=5.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--props-removed", action="store_true")
    parser.add_argument("--power-connected", action="store_true")
    args = parser.parse_args()

    if not (args.execute and args.props_removed and args.power_connected):
        print("SAFETY_STOP=EXECUTE_PROPS_REMOVED_POWER_CONNECTED_FLAGS_REQUIRED")
        return 2
    if not 3.0 <= args.throttle_percent <= 20.0:
        print("SAFETY_STOP=THROTTLE_PERCENT_OUT_OF_RANGE_3_TO_20")
        return 2
    if not 0.5 <= args.duration_s <= 2.0:
        print("SAFETY_STOP=DURATION_OUT_OF_RANGE_0_5_TO_2_SECONDS")
        return 2
    try:
        motors = [int(value.strip()) for value in args.motors.split(",") if value.strip()]
    except ValueError:
        print("SAFETY_STOP=INVALID_MOTOR_LIST")
        return 2
    if not motors or len(motors) > 4 or len(set(motors)) != len(motors):
        print("SAFETY_STOP=INVALID_MOTOR_LIST")
        return 2
    if any(motor < 1 or motor > 4 for motor in motors):
        print("SAFETY_STOP=MOTOR_INDEX_OUT_OF_RANGE_1_TO_4")
        return 2

    result = {
        "scope": "real_pixhawk_propellers_removed_disarmed_motor_test",
        "command": "MAV_CMD_DO_MOTOR_TEST",
        "arm_command_sent": False,
        "mode_command_sent": False,
        "takeoff_or_land_command_sent": False,
        "props_removed_confirmed": True,
        "power_connected_confirmed": True,
        "motors_requested": motors,
        "throttle_percent": args.throttle_percent,
        "duration_s": args.duration_s,
        "tests": [],
    }
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=GCS_SYSTEM,
        source_component=GCS_COMPONENT,
    )

    initial, initial_ok = wait_disarmed_heartbeats(link, 5, 15.0)
    result["initial_heartbeats"] = initial
    if not initial_ok:
        result["safety_stop"] = "INITIAL_STATE_NOT_DISARMED"
        write_result(args.output, result)
        return 3
    if not all(
        record["system_status"] == mavutil.mavlink.MAV_STATE_STANDBY
        for record in initial
    ):
        result["safety_stop"] = "FC_NOT_STANDBY"
        write_result(args.output, result)
        return 3

    for motor in motors:
        test = run_motor_test(link, motor, args.throttle_percent, args.duration_s)
        post_test, post_test_ok = wait_disarmed_heartbeats(link, 3, 10.0)
        test["post_test_heartbeats"] = post_test
        test["post_test_disarmed"] = post_test_ok
        result["tests"].append(test)
        if not test["accepted"] or not post_test_ok:
            result["safety_stop"] = f"MOTOR_{motor}_TEST_NOT_ACCEPTED"
            break

    final, final_ok = wait_disarmed_heartbeats(link, 8, 15.0)
    result["final_heartbeats"] = final
    result["final_disarmed"] = final_ok
    result["all_requested_tests_accepted"] = (
        len(result["tests"]) == len(motors)
        and all(test["accepted"] for test in result["tests"])
    )
    write_result(args.output, result)
    print(f"FINAL_DISARMED={int(final_ok)}")
    print("ARM_COMMAND_SENT=0 MODE_COMMAND_SENT=0 TAKEOFF_LAND_COMMAND_SENT=0")
    return 0 if final_ok and result["all_requested_tests_accepted"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
