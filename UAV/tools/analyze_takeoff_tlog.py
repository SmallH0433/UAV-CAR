"""Extract dated, real-FCU telemetry and commanded actions for takeoff review."""
import json
import sys
from pathlib import Path
from datetime import datetime
from pymavlink import mavutil

path = Path(sys.argv[1])
link = mavutil.mavlink_connection(str(path), notimestamps=False)
latest = {}
events = []
samples = []
last_hb = None
last_second = None
types = ['HEARTBEAT','RC_CHANNELS','RC_CHANNELS_RAW','SERVO_OUTPUT_RAW','VFR_HUD',
         'SYS_STATUS','POWER_STATUS','GLOBAL_POSITION_INT','RANGEFINDER','ATTITUDE',
         'STATUSTEXT','COMMAND_LONG','SET_MODE','RC_CHANNELS_OVERRIDE',
         'SET_POSITION_TARGET_LOCAL_NED']
while (msg := link.recv_match(type=types)) is not None:
    ts = getattr(msg, '_timestamp', 0)
    if not 1789833600 <= ts <= 1790006400:
        continue
    kind = msg.get_type()
    if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
        if kind in ('COMMAND_LONG','SET_MODE','RC_CHANNELS_OVERRIDE','SET_POSITION_TARGET_LOCAL_NED'):
            events.append({'time':datetime.fromtimestamp(ts).isoformat(), 'source':[msg.get_srcSystem(),msg.get_srcComponent()], **msg.to_dict()})
        continue
    if kind == 'HEARTBEAT':
        state = (bool(msg.base_mode & 128), mavutil.mode_string_v10(msg))
        if state != last_hb:
            events.append({'time':datetime.fromtimestamp(ts).isoformat(), 'armed':state[0], 'mode':state[1]})
            last_hb = state
    elif kind == 'STATUSTEXT':
        events.append({'time':datetime.fromtimestamp(ts).isoformat(), **msg.to_dict()})
    if kind in ('HEARTBEAT','RC_CHANNELS','RC_CHANNELS_RAW','SERVO_OUTPUT_RAW','VFR_HUD','SYS_STATUS','POWER_STATUS','GLOBAL_POSITION_INT','RANGEFINDER','ATTITUDE'):
        latest[kind] = {'time':ts, **msg.to_dict()}
        if last_hb and last_hb[0] and int(ts) != last_second:
            last_second = int(ts)
            samples.append({'time':datetime.fromtimestamp(ts).isoformat(), **latest})
result = {'source':str(path), 'events':events, 'armed_samples_1hz':samples}
path.with_suffix('.takeoff.json').write_text(json.dumps(result, indent=2),encoding='utf-8')
print(json.dumps({'events':[e for e in events if e.get('mavpackettype') not in ('SET_POSITION_TARGET_LOCAL_NED',)], 'armed_sample_count':len(samples)},indent=2))
