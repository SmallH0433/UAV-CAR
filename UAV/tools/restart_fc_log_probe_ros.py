"""Explicit ground maintenance: reboot a disarmed FC to test a fresh log. No erase."""
import argparse
import json
import time
import rclpy
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandLong
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter
from rclpy.qos import qos_profile_sensor_data

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reboot',action='store_true',required=True)
parser.add_argument('--sd-slowdown',type=int,choices=range(1,33))
args=parser.parse_args()
rclpy.init();n=rclpy.create_node('ground_log_recovery')
state=None;received_at=0;initial=None
def event(**row):print(json.dumps({'time':time.time(),**row}),flush=True)
def on_state(m):
    global state,received_at
    state,received_at=m,time.monotonic()
subs=[n.create_subscription(State,'/mavros/state',on_state,qos_profile_sensor_data),
      n.create_subscription(StatusText,'/mavros/statustext/recv',lambda m:event(fc_text=m.text),qos_profile_sensor_data)]
def safe():
    if state is None or not state.connected or state.armed or time.monotonic()-received_at>3:
        raise RuntimeError('Not a fresh connected disarmed FC')
def spin(seconds,guard=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
        if guard:safe()
def call(typ,name,req,guard=True):
    c=n.create_client(typ,name)
    try:
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Missing '+name)
        safe()
        f=c.call_async(req);end=time.monotonic()+15
        while not f.done() and time.monotonic()<end:spin(.1,guard)
        if not f.done():raise RuntimeError('Timeout '+name)
        return f.result()
    finally:n.destroy_client(c)
def read(names):
    req=GetParameters.Request();req.names=names
    res=call(GetParameters,'/mavros/param/get_parameters',req)
    return {key:v.integer_value if v.type==2 else None for key,v in zip(names,res.values)}
def setparam(name,value):
    for attempt in range(3):
        p=Parameter();p.name=name;p.value.type=2;p.value.integer_value=value
        req=SetParameters.Request();req.parameters=[p]
        res=call(SetParameters,'/mavros/param/set_parameters',req)
        spin(2)
        actual=read([name])[name]
        event(parameter=name,requested=value,readback=actual,ack=[r.successful for r in res.results])
        if actual==value and all(r.successful for r in res.results):return
        spin(3)
    raise RuntimeError('Could not verify '+name)
try:
    spin(5,False);safe()
    initial=read(['LOG_DISARMED','BRD_SD_SLOWDOWN','STAT_BOOTCNT'])
    event(original=initial,armed=state.armed,mode=state.mode)
    if initial['LOG_DISARMED'] not in (0,1):raise RuntimeError('Unexpected logging configuration')
    if args.sd_slowdown is not None:setparam('BRD_SD_SLOWDOWN',args.sd_slowdown)
    setparam('LOG_DISARMED',1)
    safe()
    req=CommandLong.Request();req.command=246;req.param1=1.0
    result=call(CommandLong,'/mavros/cmd/command',req,False)
    event(reboot_ack=result.success,result=result.result)
    spin(15,False)
    deadline=time.monotonic()+60
    while time.monotonic()<deadline:
        spin(.5,False)
        if state and state.connected and not state.armed and time.monotonic()-received_at<3:break
    safe();event(reconnected=True,mode=state.mode)
    spin(25)
finally:
    if initial and initial['LOG_DISARMED'] in (0,1):
        try:setparam('LOG_DISARMED',initial['LOG_DISARMED'])
        except Exception as exc:event(restore_error=str(exc))
    n.destroy_node();rclpy.shutdown()
