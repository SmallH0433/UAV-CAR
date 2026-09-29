"""Read logging parameters and FC files through existing MAVROS, disarmed only."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import FileList, FileOpen, FileRead, FileClose

rclpy.init()
n = rclpy.create_node('diagnose_fc_logging')
state = None
state_at = 0
out = {'parameters': {}, 'messages': [], 'directories': {}, 'files': {}}
def on_state(m):
    global state, state_at
    state, state_at = m, time.monotonic()
subs = [n.create_subscription(State, '/mavros/state', on_state, qos_profile_sensor_data),
        n.create_subscription(StatusText, '/mavros/statustext/recv',
                              lambda m: out['messages'].append(m.text), qos_profile_sensor_data)]
def safe():
    if state is None or not state.connected or state.armed or time.monotonic()-state_at > 3:
        raise RuntimeError('FC state absent, disconnected, armed or stale')
def call(typ, name, req):
    safe()
    client = n.create_client(typ, name)
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Missing service: ' + name)
        # Refresh state after service discovery, before sending.
        rclpy.spin_once(n, timeout_sec=0.1)
        safe()
        f = client.call_async(req)
        deadline = time.monotonic()+20
        while not f.done() and time.monotonic()<deadline:
            rclpy.spin_once(n, timeout_sec=.1)
            safe()
        if not f.done():
            raise RuntimeError('Timeout: ' + name)
        return f.result()
    finally:
        n.destroy_client(client)
try:
    deadline=time.monotonic()+8
    while state is None and time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.2)
    safe()
    out['state']={'connected':state.connected,'armed':state.armed,'mode':state.mode}
    names=['LOG_BACKEND_TYPE','LOG_DISARMED','LOG_BITMASK','LOG_FILE_BUFSIZE',
           'LOG_FILE_DSRMROT','LOG_FILE_MB_FREE','LOG_MAX_FILES','LOG_REPLAY',
           'LOG_MAV_BUFSIZE','BRD_SD_SLOWDOWN','BRD_TYPE','STAT_BOOTCNT']
    req=GetParameters.Request(); req.names=names
    res=call(GetParameters,'/mavros/param/get_parameters',req)
    for k,v in zip(names,res.values):
        out['parameters'][k]={'type':v.type,'value':v.integer_value if v.type==2 else v.double_value if v.type==3 else None}
    for directory in ('/','/APM','/APM/LOGS','/@SYS'):
        req=FileList.Request(); req.dir_path=directory
        res=call(FileList,'/mavros/ftp/list',req)
        out['directories'][directory]={'success':res.success,'errno':res.r_errno,
           'entries':[{'name':e.name,'type':e.type,'size':e.size} for e in res.list]}
    for path in ('/@SYS/version.txt','/@SYS/threads.txt','/APM/LOGS/LASTLOG.TXT'):
        req=FileOpen.Request(); req.file_path=path; req.mode=0
        res=call(FileOpen,'/mavros/ftp/open',req)
        info={'success':res.success,'errno':res.r_errno,'size':res.size}
        out['files'][path]=info
        if not res.success: continue
        try:
            req=FileRead.Request(); req.file_path=path; req.offset=0; req.size=min(res.size,4096)
            read=call(FileRead,'/mavros/ftp/read',req)
            info.update(read_success=read.success,text=bytes(read.data).decode(errors='replace'))
        finally:
            req=FileClose.Request(); req.file_path=path
            call(FileClose,'/mavros/ftp/close',req)
except Exception as exc:
    out['error']=str(exc)
finally:
    print(json.dumps(out,indent=2))
    n.destroy_node()
    rclpy.shutdown()
