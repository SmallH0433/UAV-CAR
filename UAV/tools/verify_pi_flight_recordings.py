"""Summarize received evidence without replacing missing data with zero."""
import json
from pathlib import Path
import sys
from collections import Counter

root=Path(sys.argv[1])
output=[]
for directory in sorted(root.iterdir()):
    if not directory.is_dir() or not (directory/'status.json').exists():continue
    counts=Counter();first={};last={};latest={};invalid=0
    for file in sorted(directory.glob('telemetry_*.jsonl')):
        for line in file.open(encoding='utf-8'):
            try:r=json.loads(line)
            except ValueError:
                invalid+=1;continue
            topic=r['topic'];counts[topic]+=1
            first.setdefault(topic,r['monotonic_ns']);last[topic]=r['monotonic_ns']
            latest[topic]=r['data']
    selected=('/mavros/state','/mavros/imu/data','/mavros/battery',
              '/mavros/local_position/pose','/mavros/rc/in','/landing/sensor_range')
    output.append({'session':directory.name,'status':json.loads((directory/'status.json').read_text()),
                   'invalid_lines':invalid,'received_counts':dict(counts),
                   'rates_hz':{k:round((counts[k]-1)*1e9/(last[k]-first[k]),2)
                               for k in selected if counts[k]>1 and last[k]>first[k]},
                   'latest_samples':{k:latest.get(k) for k in selected},
                   'bytes':sum(f.stat().st_size for f in directory.glob('*.jsonl'))})
print(json.dumps(output,indent=2,ensure_ascii=False))
