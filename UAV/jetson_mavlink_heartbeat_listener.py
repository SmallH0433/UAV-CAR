#!/usr/bin/env python3
import sys

from pymavlink import mavutil


connection = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
heartbeat = connection.wait_heartbeat(timeout=18)

if heartbeat is None:
    print("HEARTBEAT=NOT_RECEIVED")
    sys.exit(1)

print(
    "HEARTBEAT=RECEIVED "
    f"system={connection.target_system} component={connection.target_component}"
)
sys.exit(0)
