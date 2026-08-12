#!/usr/bin/env python3
"""No-prop low-throttle diagonal-bias and all-motor bench pattern.

The diagonal stages use the normal flight mixer: the selected diagonal is
biased faster while the other diagonal remains at armed idle.  This script
never force-arms and always releases RC overrides after disarming.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


def is_fc(message) -> bool:
    return (
        message is not None
        and message.get_srcSystem() == 1
        and message.get_srcComponent() == 1
    )


def is_armed(message) -> bool:
    return bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_override(link, throttle: int, yaw: int) -> None:
    link.mav.rc_channels_override_send(
        1,
        1,
        1500,
        1500,
        throttle,
        yaw,
        65535,
        65535,
        65535,
        65535,
    )


def release_override(link) -> None:
    link.mav.rc_channels_override_send(1, 1, 0, 0, 0, 0, 0, 0, 0, 0)


def wait_heartbeats(link, count: int, wanted_armed: bool, timeout_s: float) -> tuple[list, bool]:
    records = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and len(records) < count:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if not is_fc(message):
            continue
        record = {
            "armed": is_armed(message),
            "base_mode": int(message.base_mode),
            "custom_mode": int(message.custom_mode),
            "system_status": int(message.system_status),
        }
        records.append(record)
        print(f"HEARTBEAT_{len(records)}={record}")
        if record["armed"] != wanted_armed:
            return records, False
    return records, len(records) == count


def command_arm(link, arm: bool) -> None:
    link.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm else 0.0,
        0.0,  # never force arm/disarm
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def run_stage(
    link,
    name: str,
    duration_s: float,
    throttle: int,
    yaw: int,
    output_cap: int,
) -> tuple[dict, bool]:
    stage = {
        "name": name,
        "duration_s": duration_s,
        "throttle_override": throttle,
        "yaw_override": yaw,
        "servo_output_raw": [],
        "rc_channels": [],
        "statustext": [],
    }
    print(
        f"STAGE_START name={name} duration_s={duration_s:.1f} "
        f"throttle={throttle} yaw={yaw}"
    )
    started = time.monotonic()
    next_send = started
    safe = True
    while time.monotonic() - started < duration_s:
        now = time.monotonic()
        if now >= next_send:
            send_override(link, throttle, yaw)
            next_send = now + 0.1
        message = link.recv_match(blocking=True, timeout=0.03)
        if message is None or message.get_srcSystem() != 1:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT" and is_fc(message) and not is_armed(message):
            stage["abort_reason"] = "UNEXPECTED_DISARM"
            safe = False
            break
        if message_type == "SERVO_OUTPUT_RAW":
            values = [int(getattr(message, f"servo{i}_raw")) for i in range(1, 5)]
            stage["servo_output_raw"].append(values)
            if max(values) > output_cap:
                stage["abort_reason"] = f"OUTPUT_CAP_EXCEEDED_{max(values)}"
                safe = False
                break
        elif message_type == "RC_CHANNELS":
            stage["rc_channels"].append(
                [int(getattr(message, f"chan{i}_raw")) for i in range(1, 5)]
            )
        elif message_type == "STATUSTEXT":
            value = message.text
            if isinstance(value, bytes):
                value = value.decode("utf-8", errors="replace")
            stage["statustext"].append(str(value).rstrip("\x00"))

    stage["elapsed_s"] = time.monotonic() - started
    stage["safe"] = safe
    print(f"STAGE_END name={name} elapsed_s={stage['elapsed_s']:.3f} safe={int(safe)}")
    return stage, safe


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--props-removed", action="store_true")
    parser.add_argument("--power-connected", action="store_true")
    args = parser.parse_args()
    if not (args.execute and args.props_removed and args.power_connected):
        print("SAFETY_STOP=CONFIRMATION_FLAGS_REQUIRED")
        return 2

    result = {
        "scope": "real_pixhawk_no_prop_low_throttle_mixer_pattern",
        "force_arm": False,
        "throttle_pwm": 1150,
        "output_cap_pwm": 1250,
        "stages": [],
    }
    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    initial, initial_ok = wait_heartbeats(link, 5, False, 15.0)
    result["initial_heartbeats"] = initial
    if not initial_ok or not all(record["custom_mode"] == 0 for record in initial):
        result["safety_stop"] = "NOT_DISARMED_STABILIZE"
        return 3

    arm_confirmed = False
    try:
        # Keep all primary controls neutral and throttle low before normal arm.
        for _ in range(10):
            send_override(link, 1000, 1500)
            time.sleep(0.05)
        command_arm(link, True)
        armed_records, arm_confirmed = wait_heartbeats(link, 3, True, 8.0)
        result["armed_heartbeats"] = armed_records
        result["normal_arm_confirmed"] = arm_confirmed
        if not arm_confirmed:
            result["safety_stop"] = "NORMAL_ARM_NOT_CONFIRMED"
        else:
            print("NORMAL_ARM_CONFIRMED=1 FORCE=0")

            pattern = (
                ("diagonal_bias_positive", 2.0, 1150, 1600),
                ("neutral_gap_1", 0.5, 1000, 1500),
                ("diagonal_bias_negative", 2.0, 1150, 1400),
                ("neutral_gap_2", 0.5, 1000, 1500),
                ("all_four_low_throttle", 5.0, 1150, 1500),
            )
            for name, duration_s, throttle, yaw in pattern:
                stage, safe = run_stage(link, name, duration_s, throttle, yaw, 1250)
                result["stages"].append(stage)
                if not safe:
                    result["safety_stop"] = stage.get("abort_reason", "STAGE_FAILED")
                    break
    finally:
        # Reduce throttle before normal (non-force) disarm, then release all
        # overrides even when an earlier stage failed.
        for _ in range(5):
            send_override(link, 1000, 1500)
            time.sleep(0.05)
        command_arm(link, False)
        time.sleep(0.2)
        for _ in range(5):
            release_override(link)
            time.sleep(0.05)

    final, final_ok = wait_heartbeats(link, 8, False, 15.0)
    result["final_heartbeats"] = final
    result["final_disarmed"] = final_ok
    result["rc_overrides_released"] = True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"FINAL_DISARMED={int(final_ok)}")
    print(f"OUTPUT={args.output}")
    print("FORCE_ARM=0 TAKEOFF_LAND_COMMANDS=0 RC_OVERRIDES_RELEASED=1")
    return 0 if final_ok and "safety_stop" not in result else 5


if __name__ == "__main__":
    raise SystemExit(main())
