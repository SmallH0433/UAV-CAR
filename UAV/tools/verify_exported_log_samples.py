"""Cross-check complete MAVFTP files against read-only LOG_DATA samples."""
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink import mavutil
from flight_log_export import wait_for_disarmed_fc

root = Path(sys.argv[1])
entries = json.loads((root/'full/manifest.json').read_text())
catalog = {e['id']:e for e in json.loads((root/'catalog.json').read_text())['logs']}
assert {e['id'] for e in entries} == set(range(82,89)), 'Expected all seven complete files'
link = mavutil.mavlink_connection(sys.argv[2],baud=115200,source_system=191,source_component=191)
results = []
try:
    wait_for_disarmed_fc(link)
    for entry in entries:
        log_id, size = entry['id'], entry['size_bytes']
        assert size == catalog[log_id]['size_bytes'], 'Directory and download size mismatch'
        wait_for_disarmed_fc(link,count=1)
        for offset in (0,(size//2//90)*90,max(0,size-90)):
            link.mav.log_request_data_send(1,1,log_id,offset,90)
            deadline = time.monotonic()+5
            received = None
            while time.monotonic()<deadline:
                msg = link.recv_match(blocking=True,timeout=0.5)
                if msg is None or (msg.get_srcSystem(),msg.get_srcComponent()) != (1,1):
                    continue
                if msg.get_type()=='HEARTBEAT' and msg.base_mode & 128:
                    raise RuntimeError('FC armed; verification stopped')
                if msg.get_type()=='LOG_DATA' and msg.id==log_id and msg.ofs==offset:
                    received = bytes(msg.data[:msg.count])
                    break
            with (root/'full'/f'pixhawk_log_{log_id:03d}.BIN').open('rb') as handle:
                handle.seek(offset)
                expected = handle.read(90)
            result = {'id':log_id,'offset':offset,'received_bytes':len(received or b''),
                      'matches_ftp':received==expected,'all_zero':received is not None and not any(received)}
            results.append(result)
            (root/'cross_protocol_verification.json').write_text(json.dumps(results,indent=2))
            assert received==expected, result
    print(json.dumps({'samples':len(results),'all_match':all(r['matches_ftp'] for r in results)},indent=2))
finally:
    link.mav.log_request_end_send(1,1)
    link.close()
