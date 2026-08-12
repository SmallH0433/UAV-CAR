#!/usr/bin/env python3
import math
import sys
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common


connection = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5760", source_system=250, dialect="common"
)
heartbeat = connection.wait_heartbeat(timeout=15)
if heartbeat is None:
    print("SITL_HEARTBEAT=NOT_RECEIVED")
    sys.exit(1)

print(
    "SITL_HEARTBEAT=RECEIVED "
    f"system={connection.target_system} component={connection.target_component}"
)

for name, value in (("PLND_ENABLED", 1), ("PLND_TYPE", 1)):
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    print(f"PARAM_SET_SENT {name}={value}")

for name, expected in (("PLND_ENABLED", 1.0), ("PLND_TYPE", 1.0)):
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        -1,
    )
    value = None
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        response = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if response is None:
            continue
        raw_name = response.param_id
        response_name = (
            raw_name.decode(errors="ignore")
            if isinstance(raw_name, bytes)
            else str(raw_name)
        ).rstrip("\x00")
        if response_name == name:
            value = float(response.param_value)
            break
    if value is None or abs(value - expected) > 0.01:
        print(f"PARAM_READBACK_FAILED {name} value={value}")
        sys.exit(2)
    print(f"PARAM_READBACK_OK {name}={value:g}")

# A synthetic target in BODY_FRD: 10 cm right, 10 cm forward, 5 m down.
x = 0.10
y = 0.10
z = 5.00
distance = math.sqrt(x * x + y * y + z * z)
angle_x = math.atan2(y, x)
angle_y = math.atan2(z, math.sqrt(x * x + y * y))
frame = mavutil.mavlink.MAV_FRAME_BODY_FRD

sent = 0
deadline = time.monotonic() + 5.0
next_send = time.monotonic()
while time.monotonic() < deadline:
    now = time.monotonic()
    if now < next_send:
        time.sleep(next_send - now)
    message = common.MAVLink_landing_target_message(
        time_usec=time.time_ns() // 1000,
        target_num=0,
        frame=frame,
        angle_x=angle_x,
        angle_y=angle_y,
        distance=distance,
        size_x=0.20,
        size_y=0.20,
        x=x,
        y=y,
        z=z,
        q=(0.0, 0.0, 0.0, 0.0),
        type=0,
        position_valid=1,
    )
    connection.mav.send(message)
    sent += 1
    next_send += 0.05

print(
    f"LANDING_TARGET_SENT={sent} rate_hz={sent / 5.0:.1f} "
    f"distance_m={distance:.3f} frame=BODY_FRD position_valid=1"
)
print("SIMULATION_ONLY=1 ARMED=0")
