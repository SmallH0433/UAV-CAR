"""Read-only takeoff evidence extraction from a complete DataFlash log."""
import json
import sys
from pathlib import Path
from collections import defaultdict
from pymavlink import DFReader

path = Path(sys.argv[1])
with path.open('rb') as handle:
    if handle.read(3) != bytes((0xA3, 0x95, 0x80)):
        raise SystemExit('Missing initial DataFlash FMT record; refuse to analyse corrupt recording')
reader = DFReader.DFReader_binary(str(path))
rows = defaultdict(list)
params = {}
selected = {'ARM', 'MODE', 'MSG', 'ERR', 'EV', 'RCIN', 'RCOU', 'CTUN', 'BAT', 'MOTB', 'POWR', 'ATT', 'RFND', 'RATE'}
while (msg := reader.recv_msg()) is not None:
    kind = msg.get_type()
    data = msg.to_dict()
    if kind == 'PARM':
        params[data['Name']] = data['Value']
    elif kind in selected:
        rows[kind].append(data)

def ranges(data):
    result = {}
    for key in data[0] if data else []:
        values = [x[key] for x in data if isinstance(x.get(key), (int, float))]
        if values:
            result[key] = [min(values), max(values)]
    return result

summary = {'log': str(path), 'parameters': {k:v for k,v in params.items() if k.startswith(('MOT_', 'RC3_', 'SERVO', 'BATT_', 'ATC_THR', 'PILOT_THR', 'THR_', 'FRAME_', 'RCMAP'))},
           'events': {k:rows[k] for k in ['ARM', 'MODE', 'MSG', 'ERR', 'EV']},
           'ranges': {k:ranges(rows[k]) for k in selected if k not in ['ARM', 'MODE', 'MSG', 'ERR', 'EV']}}
timeline = []
latest = {}
last_second = -1
all_rows = sorted((x for k in ['RCIN','RCOU','CTUN','BAT','MOTB','POWR','ATT','RFND'] for x in rows[k]), key=lambda x:x.get('TimeUS',0))
for data in all_rows:
    latest[data['mavpackettype']] = data
    sec = int(data.get('TimeUS', 0) / 1e6)
    if sec != last_second and 'CTUN' in latest:
        last_second = sec
        timeline.append({'time_s':sec, **latest})
output = path.with_suffix('.takeoff.json')
output.write_text(json.dumps({**summary, 'timeline_1hz':timeline}, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))
