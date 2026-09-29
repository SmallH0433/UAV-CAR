"""Explicit hardware probe: log briefly while disarmed, restore parameter, read header."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter
from mavros_msgs.msg import State, LogData, LogEntry
from mavros_msgs.srv import ParamSetV2, LogRequestEnd, LogRequestList, LogRequestData, MessageInterval

rclpy.init()
n=rclpy.create_node('logging_disarmed_verification')
state=None; state_at=0
entries={}; data=[]
out={'parameters_before':{},'actions':[],'samples':[]}
def on_state(m):
    global state,state_at
    state,state_at=m,time.monotonic()
subs=[n.create_subscription(State,'/mavros/state',on_state,qos_profile_sensor_data),
      n.create_subscription(LogEntry,'/mavros/log_transfer/raw/log_entry',lambda m:entries.update({m.id:m}),100),
      n.create_subscription(LogData,'/mavros/log_transfer/raw/log_data',lambda m:data.append(m),100)]
def safe():
    if state is None or not state.connected or state.armed or time.monotonic()-state_at>3:
        raise RuntimeError('State missing, stale, disconnected or armed')
def spin(seconds,guard=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
        if guard: safe()
def call(typ,name,req):
    client=n.create_client(typ,name)
    try:
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Unavailable: '+name)
        safe()
        f=client.call_async(req)
        end=time.monotonic()+10
        while not f.done() and time.monotonic()<end: spin(.1)
        if not f.done():raise RuntimeError('Timeout: '+name)
        res=f.result()
        if hasattr(res,'success') and not res.success:raise RuntimeError('Failed: '+name)
        return res
    finally:n.destroy_client(client)
def param(value):
    p=Parameter();p.name='LOG_DISARMED';p.value.type=2;p.value.integer_value=value
    req=SetParameters.Request();req.parameters=[p]
    res=call(SetParameters,'/mavros/param/set_parameters',req)
    out['actions'].append({'LOG_DISARMED':value,'results':[{'success':r.successful,'reason':r.reason} for r in res.results]})
    req=GetParameters.Request();req.names=['LOG_DISARMED']
    check=call(GetParameters,'/mavros/param/get_parameters',req)
    out['actions'].append({'readback':check.values[0].integer_value})
    if not res.results or not all(r.successful for r in res.results) or check.values[0].integer_value!=value:
        raise RuntimeError('Parameter update or readback failed')
base='/mavros/log_transfer/raw/'
previous=None; changed=False
try:
    spin(4,False);safe()
    out['initial_state']={'armed':state.armed,'mode':state.mode}
    req=GetParameters.Request(); req.names=['LOG_DISARMED']+[f'SR1_{s}' for s in ('RAW_SENS','EXT_STAT','RC_CHAN','POSITION','EXTRA1','EXTRA2','EXTRA3')]
    res=call(GetParameters,'/mavros/param/get_parameters',req)
    for name,v in zip(req.names,res.values):out['parameters_before'][name]={'type':v.type,'value':v.integer_value if v.type==2 else v.double_value if v.type==3 else None}
    previous=out['parameters_before']['LOG_DISARMED']['value']
    if previous not in (0,1):raise RuntimeError('Unexpected LOG_DISARMED; no change')
    call(LogRequestEnd,base+'log_request_end',LogRequestEnd.Request())
    # Restore only diagnostic telemetry streams missing from the existing connection.
    for msg_id,hz in ((30,10.0),(27,5.0),(1,1.0),(33,5.0),(32,5.0),(74,2.0),(245,1.0)):
        req=MessageInterval.Request();req.message_id=msg_id;req.message_rate=hz
        try:
            res=call(MessageInterval,'/mavros/set_message_interval',req)
            out['actions'].append({'message_id':msg_id,'hz':hz,'success':res.success})
        except RuntimeError as exc:
            out['actions'].append({'message_id':msg_id,'hz':hz,'error':str(exc)})
        spin(1)
    changed=True
    param(1)
    spin(20)
    param(previous);changed=False
    spin(5)
    req=LogRequestList.Request();req.start=0;req.end=65535
    call(LogRequestList,base+'log_request_list',req);spin(6)
    out['catalog_latest']=[{'id':m.id,'size':m.size} for m in sorted(entries.values(),key=lambda m:m.id)[-3:]]
    if not entries:raise RuntimeError('No logs')
    latest=entries[max(entries)]
    for offset in (0,90,180,max(0,latest.size//2//90*90)):
        data.clear()
        req=LogRequestData.Request();req.id=latest.id;req.offset=offset;req.count=90
        call(LogRequestData,base+'log_request_data',req);spin(2)
        match=next((m for m in data if m.id==latest.id and m.offset==offset),None)
        out['samples'].append({'id':latest.id,'offset':offset,'received':match is not None,
                               'hex':bytes(match.data).hex() if match else None,
                               'nonzero':sum(b!=0 for b in match.data) if match else None})
except Exception as exc:out['error']=str(exc)
finally:
    if changed:
        try:param(previous)
        except Exception as exc:out['restore_error']=str(exc)
    try:call(LogRequestEnd,base+'log_request_end',LogRequestEnd.Request())
    except Exception as exc:out['end_error']=str(exc)
    print(json.dumps(out,indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
