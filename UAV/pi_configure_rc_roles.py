#!/usr/bin/env python3
"""Safely configure the Pixhawk RC role and EKF source-set mapping.

Requested mapping:

* CH5: ArduCopter flight-mode channel
* CH6: raw companion-computer follow authorization (RC6_OPTION remains 0)
* CH7: EKF source-set selector (RC7_OPTION=90)
* CH8: raw companion-computer landing request (RC8_OPTION remains 0)

CH7 low selects optical flow.  CH7 middle and high select duplicate GPS
source sets, so no physical switch position can select an unconfigured EKF
source.  The tool never arms, changes mode, or sends motor commands.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


# Order matters: configure every EKF source set before enabling RC7 option 90.
DESIRED = {
    "EK3_SRC_OPTIONS": 0.0,
    # Source set 1 / CH7 low: optical flow + barometer + compass.
    "EK3_SRC1_POSXY": 0.0,
    "EK3_SRC1_VELXY": 5.0,
    "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELZ": 0.0,
    "EK3_SRC1_YAW": 1.0,
    # Source set 2 / CH7 middle: GPS + barometer + compass.
    "EK3_SRC2_POSXY": 3.0,
    "EK3_SRC2_VELXY": 3.0,
    "EK3_SRC2_POSZ": 1.0,
    "EK3_SRC2_VELZ": 3.0,
    "EK3_SRC2_YAW": 1.0,
    # Source set 3 / CH7 high: duplicate GPS set for a safe two-position use.
    "EK3_SRC3_POSXY": 3.0,
    "EK3_SRC3_VELXY": 3.0,
    "EK3_SRC3_POSZ": 1.0,
    "EK3_SRC3_VELZ": 3.0,
    "EK3_SRC3_YAW": 1.0,
    "FLTMODE_CH": 5.0,
    "RC5_OPTION": 0.0,
    "RC6_OPTION": 0.0,
    "RC8_OPTION": 0.0,
    # Enable only after all three source sets are valid.
    "RC7_OPTION": 90.0,
}


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def wait_safe_gate(
    link,
    *,
    heartbeat_count: int = 5,
    rc_sample_count: int = 10,
    low_maximum: int = 1200,
    timeout_s: float = 25.0,
) -> bool:
    disarmed_heartbeats = 0
    samples: list[tuple[int, int, int]] = []
    rc_stream_requested = False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        if message.get_type() == "HEARTBEAT" and message.get_srcComponent() == 1:
            armed = bool(
                int(message.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            if armed:
                print("SAFETY_STOP=ARMED")
                return False
            disarmed_heartbeats += 1
            if not rc_stream_requested:
                # A Pixhawk reboot clears the stream interval previously
                # requested by MAVROS.  Request RC_CHANNELS (message 65) at
                # 10 Hz; this changes telemetry output only, never flight state.
                link.mav.command_long_send(
                    1,
                    1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    65,
                    100_000,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                rc_stream_requested = True
                print("RC_CHANNELS_STREAM_REQUESTED_HZ=10", flush=True)
        elif message.get_type() == "RC_CHANNELS":
            samples.append(
                (
                    int(message.chan6_raw),
                    int(message.chan7_raw),
                    int(message.chan8_raw),
                )
            )
        if (
            disarmed_heartbeats >= heartbeat_count
            and len(samples) >= rc_sample_count
        ):
            break

    if disarmed_heartbeats < heartbeat_count or len(samples) < rc_sample_count:
        print(
            "SAFETY_STOP=INSUFFICIENT_TELEMETRY "
            f"HEARTBEATS={disarmed_heartbeats} RC_SAMPLES={len(samples)}"
        )
        return False

    for index, label in enumerate(("RC6", "RC7", "RC8")):
        values = [sample[index] for sample in samples]
        print(f"{label}_MIN={min(values)} {label}_MAX={max(values)}")
        if max(values) > low_maximum:
            print(f"SAFETY_STOP={label}_NOT_LOW")
            return False
    print("SAFETY_GATE=DISARMED_RC6_RC7_RC8_LOW")
    return True


def read_parameter(link, name: str) -> dict | None:
    for _attempt in range(4):
        link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return {
                    "value": float(message.param_value),
                    "type": int(message.param_type),
                }
    return None


def write_and_confirm(link, name: str, value: float, param_type: int) -> float | None:
    for _attempt in range(4):
        link.mav.param_set_send(1, 1, name.encode("ascii"), value, param_type)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is not None and parameter_name(message) == name:
                return float(message.param_value)
    return None


def read_current(link) -> dict[str, dict]:
    current: dict[str, dict] = {}
    for name in DESIRED:
        record = read_parameter(link, name)
        if record is None:
            raise RuntimeError(f"PARAMETER_NOT_RECEIVED:{name}")
        current[name] = record
        print(f"BEFORE {name}={record['value']} TYPE={record['type']}", flush=True)
    return current


def write_target(link, target: dict[str, dict | float], current: dict[str, dict]) -> bool:
    changed: list[str] = []
    for name in DESIRED:
        target_record = target[name]
        desired = (
            float(target_record["value"])
            if isinstance(target_record, dict)
            else float(target_record)
        )
        before = current[name]
        if abs(float(before["value"]) - desired) <= 0.001:
            print(f"UNCHANGED {name}={desired}", flush=True)
            continue
        confirmed = write_and_confirm(link, name, desired, int(before["type"]))
        print(f"SET {name} requested={desired} confirmed={confirmed}", flush=True)
        if confirmed is None or abs(confirmed - desired) > 0.001:
            print(f"SAFETY_STOP=WRITE_NOT_CONFIRMED:{name}")
            return False
        changed.append(name)
    print("CHANGED=" + ",".join(changed))
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("backup", "apply", "rollback"))
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--backup", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        autoreconnect=False,
        source_system=190,
        source_component=191,
    )
    if not wait_safe_gate(link):
        return 2
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    try:
        current = read_current(link)
    except RuntimeError as error:
        print(f"SAFETY_STOP={error}")
        return 3

    if args.phase in ("backup", "apply"):
        args.backup.parent.mkdir(parents=True, exist_ok=True)
        args.backup.write_text(
            json.dumps(
                {
                    "created_utc": time.strftime(
                        "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
                    ),
                    "vehicle": "real Pixhawk system 1 component 1",
                    "armed": False,
                    "switch_gate": "RC6_RC7_RC8_LOW",
                    "roles": {
                        "CH5": "flight_mode",
                        "CH6": "companion_follow",
                        "CH7_LOW": "optical_flow_source_set_1",
                        "CH7_MIDDLE_HIGH": "gps_source_sets_2_3",
                        "CH8": "companion_auto_landing",
                    },
                    "parameters": current,
                    "desired": DESIRED,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"BACKUP={args.backup}", flush=True)

    if args.phase == "backup":
        print("READ_ONLY=1 PARAMETERS_CHANGED=0")
        return 0

    if args.phase == "rollback":
        if not args.backup.is_file():
            print(f"SAFETY_STOP=BACKUP_NOT_FOUND:{args.backup}")
            return 4
        saved = json.loads(args.backup.read_text(encoding="utf-8"))
        target = saved.get("parameters", {})
        if set(DESIRED) - set(target):
            print("SAFETY_STOP=BACKUP_PARAMETER_SET_INCOMPLETE")
            return 4
    else:
        target = DESIRED

    if not write_target(link, target, current):
        return 5
    print(
        "SAFETY=DISARMED_PARAMETER_SCOPE_ONLY MODE_CHANGE=0 ARM_COMMAND=0 "
        "MOTOR_COMMAND=0 REBOOT_REQUIRED=1"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
