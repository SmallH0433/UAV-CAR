"""Read-only FC storage diagnostics over a temporary local MAVROS UDP endpoint."""
import argparse
from pathlib import Path
import time
import json
from pymavlink import mavutil,mavftp

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
link=mavutil.mavlink_connection('udpin:127.0.0.1:14601',source_system=191,source_component=191)
hb=None
deadline=time.monotonic()+15
while time.monotonic()<deadline:
    m=link.recv_match(type='HEARTBEAT',blocking=True,timeout=1)
    if m and m.get_srcSystem()==1 and m.get_srcComponent()==1 and m.autopilot!=8:
        hb=m;break
if hb is None or hb.base_mode&128:raise RuntimeError('No disarmed FC heartbeat')
last=time.monotonic()
def guard(_link,m):
    global last
    if m.get_type()=='HEARTBEAT' and m.get_srcSystem()==1 and m.get_srcComponent()==1:
        if m.base_mode&128:raise RuntimeError('FC armed; stopping')
        last=time.monotonic()
    if time.monotonic()-last>10:raise RuntimeError('FC heartbeat stale')
link.message_hooks.append(guard)
try:
    ftp=mavftp.MAVFTP(link,1,1)
    for directory in ('/','/APM','/APM/LOGS','/@SYS'):
        print('DIRECTORY',directory,flush=True)
        ftp.cmd_list([directory]);res=ftp.process_ftp_reply('list',timeout=15)
        res.display_message()
    for path in ('/@SYS/sysinfo.txt','/@SYS/version.txt','/@SYS/threads.txt','/APM/LOGS/LASTLOG.TXT'):
        dest=a.output/path.rsplit('/',1)[1]
        if dest.exists():raise RuntimeError('Refusing overwrite '+str(dest))
        ftp.temp_filename=str(dest.with_suffix('.partial'))
        print('FILE',path,flush=True)
        ftp.cmd_get([path,str(dest)]);res=ftp.process_ftp_reply('get',timeout=20)
        res.display_message()
        if dest.exists():print(dest.read_text(errors='replace'),flush=True)
finally:link.close()
