#!/usr/bin/env python3
"""Repeatedly request normal (non-force) disarm and release RC overrides."""

import time

from pymavlink import mavutil


link = mavutil.mavlink_connection(
    "/dev/serial0",
    baud=57600,
    autoreconnect=False,
    source_system=255,
    source_component=191,
)

for _ in range(30):
    # Explicit low throttle and neutral controls before each normal disarm.
    link.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535)
    link.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    time.sleep(0.1)

for _ in range(10):
    link.mav.rc_channels_override_send(1, 1, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.05)

disarmed_count = 0
deadline = time.monotonic() + 15.0
while time.monotonic() < deadline and disarmed_count < 10:
    message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
    if message is None or message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
        continue
    armed = bool(int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"HEARTBEAT armed={int(armed)} base_mode={int(message.base_mode)}")
    if armed:
        disarmed_count = 0
        link.mav.command_long_send(
            1,
            1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    else:
        disarmed_count += 1

print(f"FINAL_DISARMED_COUNT={disarmed_count}")
print("FORCE_DISARM=0 OVERRIDES_RELEASED=1")
raise SystemExit(0 if disarmed_count >= 10 else 2)
