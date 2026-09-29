"""Read-only, disarmed MAVFTP export of explicitly selected onboard logs."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink import mavutil, mavftp
from flight_log_export import wait_for_disarmed_fc, check_dataflash_content

parser = argparse.ArgumentParser()
parser.add_argument('--port', required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--ids', nargs='+', type=int, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
link = mavutil.mavlink_connection(args.port, baud=115200, source_system=191, source_component=191)
hb = wait_for_disarmed_fc(link)
target = (hb.get_srcSystem(), hb.get_srcComponent())
last_heartbeat = time.monotonic()

def guard(_link, msg):
    global last_heartbeat
    if msg.get_type() == 'HEARTBEAT' and (msg.get_srcSystem(), msg.get_srcComponent()) == target:
        if msg.base_mode & 128:
            raise RuntimeError('FC armed: stopping read-only export')
        last_heartbeat = time.monotonic()
    if time.monotonic() - last_heartbeat > 10:
        raise RuntimeError('FC heartbeat stale: stopping export')

link.message_hooks.append(guard)
results = []
try:
    ftp = mavftp.MAVFTP(link, *target)
    ftp.ftp_settings.burst_read_size = 239
    for log_id in args.ids:
        output = args.output / f'pixhawk_log_{log_id:03d}.BIN'
        if output.exists():
            raise RuntimeError(f'Refusing to overwrite {output}')
        ftp.temp_filename = str(output.with_suffix('.ftp-partial'))
        print(f'START log {log_id}', flush=True)
        started = time.monotonic()
        ftp.cmd_get([f'/APM/LOGS/{log_id:08d}.BIN', str(output)])
        result = ftp.process_ftp_reply('get', timeout=1800)
        result.display_message()
        if not output.exists() or not ftp.done or ftp.read_gaps:
            raise RuntimeError(f'Incomplete FTP download for log {log_id}')
        data = output.read_bytes()
        entry = {'id': log_id, 'path': str(output), 'size_bytes': len(data),
                 'sha256': hashlib.sha256(data).hexdigest(),
                 'nonzero_bytes': sum(x != 0 for x in data),
                 'duration_s': round(time.monotonic()-started,2),
                 'content_check':check_dataflash_content(output)}
        results.append(entry)
        (args.output/'manifest.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(json.dumps(entry),flush=True)
finally:
    link.close()
