"""Read-only capture of live tag body headings; does not connect to MAVLink."""
import json
import math
import statistics
import time
import urllib.request
from pathlib import Path

rows=[]
for i in range(40):
    with urllib.request.urlopen('http://127.0.0.1:8765/api/status',timeout=2) as response:
        s=json.load(response)
    rows.append(dict(unix_s=time.time(),sequence=s['analysis_sequence'],frame_age_ms=s['frame_age_ms'],detections=s['detections'],contract=s.get('body_orientation_contract'),armed=s.get('flight',{}).get('armed')))
    time.sleep(.2)
summary={}
for tag_id in (0,1):
    data=[d for row in rows for d in row['detections'] if d['tag_id']==tag_id and d.get('orientation',{}).get('valid')]
    if data:
        angles=[d['orientation']['pad_heading_body_deg'] for d in data]
        mean=math.degrees(math.atan2(sum(math.sin(math.radians(a)) for a in angles),sum(math.cos(math.radians(a)) for a in angles)))
        summary[str(tag_id)]={'samples':len(data),'mean_heading_body_deg':mean,'min_deg':min(angles),'max_deg':max(angles),'distance_m':statistics.mean(d['distance_m'] for d in data),'mean_forward_body_frd':[statistics.mean(d['orientation']['pad_forward_body_frd'][j] for d in data) for j in range(3)]}
deltas=[]
for row in rows:
    headings={d['tag_id']:d['orientation']['pad_heading_body_deg'] for d in row['detections'] if d.get('orientation',{}).get('valid')}
    if 0 in headings and 1 in headings:
        deltas.append((headings[1]-headings[0]+180)%360-180)
summary['simultaneous_pairs']=len(deltas)
summary['mean_inner_minus_outer_deg']=statistics.mean(deltas) if deltas else None
summary['any_armed']=any(r['armed'] is True for r in rows)
path=Path('body_orientation_live_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
path.write_text(json.dumps(dict(summary=summary,rows=rows),ensure_ascii=False,indent=2))
print(json.dumps(dict(file=str(path.resolve()),summary=summary),ensure_ascii=False))
