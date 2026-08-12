#!/usr/bin/env python3
import sys
import time

from pymavlink import mavutil


source = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
heartbeat = source.wait_heartbeat(timeout=15)
if heartbeat is None:
    print("SITL_HEARTBEAT=NOT_RECEIVED")
    sys.exit(1)

print(
    "SITL_HEARTBEAT=RECEIVED "
    f"system={source.target_system} component={source.target_component}"
)

destination = mavutil.mavlink_connection("udpout:192.168.1.103:14550")
deadline = time.time() + 10
sent = 0
while time.time() < deadline:
    message = source.recv_match(blocking=True, timeout=1)
    if message is not None:
        destination.write(message.get_msgbuf())
        sent += 1

print(f"UDP_MESSAGES_SENT={sent}")
