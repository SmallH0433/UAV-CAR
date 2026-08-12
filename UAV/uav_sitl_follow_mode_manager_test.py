#!/usr/bin/env python3
"""Real ArduCopter SITL acceptance test for CH7 follow mode arbitration.

The endpoint is hard-coded to localhost.  It never connects to the physical
Pixhawk or Raspberry Pi.  Arming, takeoff and landing happen only in software
simulation so actual HEARTBEAT-confirmed ArduCopter mode transitions can be
validated.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


WORKSPACE = Path("/mnt/d/Codex/UAV")
sys.path.insert(0, str(WORKSPACE / "imx296_debug"))
sys.path.insert(0, str(WORKSPACE))

from follow_mode_manager import (  # noqa: E402
    FollowModeManager,
    ModeManagerInputs,
    ModeManagerState,
)
from mavlink_guided_velocity import GuidedVelocitySetpoint, make_message  # noqa: E402
from uav_sitl_apriltag_follow_test import (  # noqa: E402
    arm_sitl_when_ready,
    global_to_local_ned,
    land_and_wait,
    request_sitl_streams,
    send_gcs_heartbeat,
    set_message_interval,
    set_mode,
    wait_global_position,
)


SITL_ENDPOINT = "udpin:127.0.0.1:14550"
OUTPUT_SUMMARY = WORKSPACE / "output/follow_mode_manager_sitl_20260807_summary.json"
OUTPUT_JSONL = WORKSPACE / "output/follow_mode_manager_sitl_20260807.jsonl"


def send_mode_request(connection, mode: str) -> None:
    mapping = connection.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"SITL does not expose mode {mode}")
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )


def send_zero_velocity(connection, time_s: float) -> None:
    connection.mav.send(
        make_message(
            GuidedVelocitySetpoint(int(time_s * 1000.0) & 0xFFFFFFFF, 0.0, 0.0),
            target_system=connection.target_system,
            target_component=connection.target_component,
            max_speed_mps=0.2,
        )
    )


def update_heartbeat(connection, state: dict) -> None:
    while True:
        message = connection.recv_match(type="HEARTBEAT", blocking=False)
        if message is None:
            return
        if message.get_srcSystem() != connection.target_system:
            continue
        state["mode"] = mavutil.mode_string_v10(message).upper()
        state["armed"] = bool(
            message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        state["heartbeat_id"] += 1


def drive_manager(
    connection,
    manager: FollowModeManager,
    state: dict,
    records: list[dict],
    *,
    rc_enable: bool,
    pilot_stick_override: bool = False,
):
    now = time.monotonic()
    send_gcs_heartbeat(connection)
    request_sitl_streams(connection)
    update_heartbeat(connection, state)
    decision = manager.update(
        ModeManagerInputs(
            timestamp_s=now,
            armed=bool(state["armed"]),
            current_mode=str(state["mode"]),
            rc_enable=rc_enable,
            prerequisites_ok=True,
            pilot_stick_override=pilot_stick_override,
            mode_sample_id=int(state["heartbeat_id"]),
        )
    )
    if decision.send_zero_velocity and state["mode"] == "GUIDED":
        send_zero_velocity(connection, now)
    if decision.request_mode is not None:
        send_mode_request(connection, decision.request_mode)
    records.append(
        {
            "time_monotonic_s": now,
            "rc_enable": rc_enable,
            "pilot_stick_override": pilot_stick_override,
            "autopilot_mode": state["mode"],
            "heartbeat_id": state["heartbeat_id"],
            "manager_state": decision.state.value,
            "reason": decision.reason,
            "request_mode": decision.request_mode,
            "allow_follow_velocity": decision.allow_follow_velocity,
            "send_zero_velocity": decision.send_zero_velocity,
            "lockout": decision.lockout,
        }
    )
    return decision


def wait_for(
    description: str,
    timeout_s: float,
    predicate,
    step,
):
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = step()
        if predicate(last):
            return last
        time.sleep(0.1)
    raise RuntimeError(f"timeout waiting for {description}; last={last}")


def run() -> tuple[list[dict], dict]:
    if SITL_ENDPOINT != "udpin:127.0.0.1:14550":
        raise RuntimeError("non-local MAVLink endpoints are forbidden")
    connection = mavutil.mavlink_connection(
        SITL_ENDPOINT,
        source_system=250,
        source_component=191,
        dialect="common",
    )
    heartbeat = connection.wait_heartbeat(timeout=20)
    if heartbeat is None or connection.target_system != 1:
        raise RuntimeError("local ArduCopter SITL heartbeat not received")
    for message_id in (0, 33):
        set_message_interval(connection, message_id, 10.0)
    initial_global = wait_global_position(connection, timeout_s=30.0)
    origin = (int(initial_global.lat), int(initial_global.lon))
    set_mode(connection, "GUIDED")
    arm_sitl_when_ready(connection)
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        2.0,
    )
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        position = wait_global_position(connection, timeout_s=1.0)
        if -global_to_local_ned(position, origin)[2] >= 1.7:
            break
    else:
        raise RuntimeError("SITL takeoff altitude was not reached")

    set_mode(connection, "ALT_HOLD")
    state = {"mode": "ALT_HOLD", "armed": True, "heartbeat_id": 0}
    manager = FollowModeManager()
    records: list[dict] = []

    def step_high(stick: bool = False):
        return drive_manager(
            connection,
            manager,
            state,
            records,
            rc_enable=True,
            pilot_stick_override=stick,
        )

    def step_low():
        return drive_manager(
            connection, manager, state, records, rc_enable=False
        )

    # CH7 high is the sole normal operator action: request and confirm GUIDED.
    wait_for(
        "initial automatic GUIDED entry",
        8.0,
        lambda d: d.state == ModeManagerState.ACTIVE,
        step_high,
    )
    initial_auto_guided = state["mode"] == "GUIDED"

    # CH7 low must stop setpoints and restore the mode present at engagement.
    wait_for(
        "CH7-low ALT_HOLD restoration",
        8.0,
        lambda _d: state["mode"] == "ALT_HOLD",
        step_low,
    )
    ch7_low_restored = state["mode"] == "ALT_HOLD"
    wait_for(
        "automatic GUIDED re-entry",
        8.0,
        lambda d: d.state == ModeManagerState.ACTIVE,
        step_high,
    )

    # A deliberate stick override returns to the entry mode and latches out.
    stick_decision = step_high(True)
    wait_for(
        "stick-override ALT_HOLD restoration",
        8.0,
        lambda _d: state["mode"] == "ALT_HOLD",
        step_high,
    )
    requests_before = sum(r["request_mode"] == "GUIDED" for r in records)
    lockout_deadline = time.monotonic() + 1.2
    while time.monotonic() < lockout_deadline:
        step_high()
        time.sleep(0.1)
    requests_after = sum(r["request_mode"] == "GUIDED" for r in records)
    stick_lockout_held = requests_after == requests_before
    step_low()
    wait_for(
        "post-stick-cycle GUIDED re-entry",
        8.0,
        lambda d: d.state == ModeManagerState.ACTIVE,
        step_high,
    )

    # A physical flight-mode switch always wins and must not be fought.
    set_mode(connection, "LOITER")
    state["mode"] = "LOITER"
    mode_override_decision = step_high()
    requests_before = sum(r["request_mode"] == "GUIDED" for r in records)
    lockout_deadline = time.monotonic() + 1.2
    while time.monotonic() < lockout_deadline:
        step_high()
        time.sleep(0.1)
    requests_after = sum(r["request_mode"] == "GUIDED" for r in records)
    mode_lockout_held = requests_after == requests_before and state["mode"] == "LOITER"
    step_low()
    wait_for(
        "post-mode-cycle GUIDED re-entry",
        8.0,
        lambda d: d.state == ModeManagerState.ACTIVE,
        step_high,
    )

    landed_disarmed = land_and_wait(connection)
    passed = all(
        (
            initial_auto_guided,
            ch7_low_restored,
            stick_decision.reason == "PILOT_STICK_OVERRIDE",
            stick_lockout_held,
            mode_override_decision.reason == "PILOT_MODE_OVERRIDE",
            mode_lockout_held,
            landed_disarmed,
        )
    )
    summary = {
        "scope": "local_arducopter_sitl_only",
        "endpoint": SITL_ENDPOINT,
        "physical_vehicle_connected": False,
        "initial_ch7_auto_guided_confirmed": initial_auto_guided,
        "ch7_low_zero_and_entry_mode_restore_confirmed": ch7_low_restored,
        "stick_override_reason": stick_decision.reason,
        "stick_override_lockout_held_until_ch7_cycle": stick_lockout_held,
        "flight_mode_switch_override_reason": mode_override_decision.reason,
        "flight_mode_switch_not_fought_while_ch7_high": mode_lockout_held,
        "ch7_cycle_reenabled_follow": records[-1]["manager_state"] == "ACTIVE",
        "landed_and_disarmed": landed_disarmed,
        "guided_requests": sum(r["request_mode"] == "GUIDED" for r in records),
        "zero_velocity_events": sum(r["send_zero_velocity"] for r in records),
        "records": len(records),
        "passed": passed,
    }
    return records, summary


def main() -> int:
    records: list[dict] = []
    summary: dict = {}
    try:
        records, summary = run()
    finally:
        OUTPUT_JSONL.parent.mkdir(parents=True, exist_ok=True)
        if records:
            with OUTPUT_JSONL.open("w", encoding="utf-8") as output_file:
                for record in records:
                    output_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        if summary:
            OUTPUT_SUMMARY.write_text(
                json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print("SIMULATION_ONLY=1 REAL_PIXHAWK_COMMANDS=0")
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
