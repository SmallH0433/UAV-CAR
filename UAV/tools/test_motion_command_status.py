"""Test read-only status geometry without importing ROS or touching hardware."""
import ast
import math
import pathlib
import threading
import time
from typing import Optional

path = pathlib.Path('air_ground_open_source/08_air_ground_landing/ros2_ws/src/air_ground_landing_ros2/air_ground_landing_ros2/flight_status_http.py')
tree = ast.parse(path.read_text(encoding='utf-8-sig'))
subset = ast.Module(body=[n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in ('axis_direction', 'command_to_body_flu', 'FlightStatusState')], type_ignores=[])
scope = dict(math=math, threading=threading, time=time, Optional=Optional)
exec(compile(subset, str(path), 'exec'), scope)
convert = scope['command_to_body_flu']
for yaw in (0, .6, math.pi/2, math.pi, -math.pi/2):
    body = (.12, -.07, -.03)
    c, s = math.cos(yaw), math.sin(yaw)
    command = dict(coordinate_frame=1,type_mask=3527,velocity=dict(x=c*body[0]-s*body[1], y=s*body[0]+c*body[1], z=body[2]))
    result = convert(command, (0, 0, math.sin(yaw/2), math.cos(yaw/2)))
    assert all(abs(result[k]-v)<1e-9 for k,v in zip(('x','y','z'),body))
assert convert(command, None) is None
assert convert(dict(command, coordinate_frame=99), (0,0,0,1)) is None
assert convert(dict(command, type_mask=8), (0,0,0,1)) is None
assert convert(dict(command, coordinate_frame=8), None) == command['velocity']
state = scope['FlightStatusState'](.05)
state.vehicle = dict(connected=True,armed=True,mode='GUIDED',system_status=4)
state.vehicle_received_s = state.command_received_s = state.pose_received_s = time.monotonic()
state.command = command
state.pose_quaternion = (0,0,0,1)
assert state.snapshot()['motion_command']['state'] == 'READY'
state.command_received_s -= 1
assert state.snapshot()['motion_command']['body_velocity_mps'] is None
state.command_received_s = time.monotonic()
state.vehicle['mode'] = 'LOITER'
assert state.snapshot()['motion_command']['body_velocity_mps'] is None
print('PASS: ENU/FLU conversion at five headings, body commands, missing pose/masked/unknown frame, expired commands and mode exit')
