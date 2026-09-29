"""One-shot landed event -> sequential 50 mm jog. Persistent latch, explicit rearm."""
import json
import os
from pathlib import Path
import time
from std_srvs.srv import Trigger
from car_interfaces.srv import LeadscrewControl

class LandingReceiver:
    def __init__(self,node):
        self.node=node
        node.declare_parameter('landing_latch_file',str(Path.home()/'.local/state/car_landing/latch.json'))
        node.declare_parameter('landing_rpm',100.0)
        self.path=Path(node.get_parameter('landing_latch_file').value)
        self.charge_path=self.path.with_name('charge_'+self.path.name)
        self.pending=False
        self.client=node.create_client(LeadscrewControl,'/leadscrew/control')
        node.create_service(Trigger,'/uav_bridge/rearm_landing',self.rearm)
    def rearm(self,req,res):
        if self.pending:
            res.success=False;res.message='Landing request still pending';return res
        try:
            self.path.unlink(missing_ok=True)
            self.charge_path.unlink(missing_ok=True)
            res.success=True;res.message='Armed for the next landing; no motion commanded'
        except OSError as exc:res.success=False;res.message=str(exc)
        return res
    def handle(self,data,addr,sock,allowed_ip):
        try:
            text=data.decode('utf-8').strip()
            try:msg=json.loads(text)
            except ValueError:msg=text
        except UnicodeError:return False
        recognized=msg=='已成功降落' if isinstance(msg,str) else isinstance(msg,dict) and (msg.get('event')=='landed' or msg.get('message')=='已成功降落')
        charge=msg=='充电完成' if isinstance(msg,str) else isinstance(msg,dict) and (msg.get('event')=='charge_complete' or msg.get('message')=='充电完成')
        if not recognized and not charge:return False
        latch=self.charge_path if charge else self.path
        def reply(status,detail=''):
            try:sock.sendto(json.dumps(dict(event='charge_ack' if charge else 'landing_ack',status=status,detail=detail),ensure_ascii=False).encode(),addr)
            except OSError:pass
        if addr[0]!=allowed_ip:
            reply('rejected','Unexpected source IP');return True
        if self.pending or latch.exists():
            reply('duplicate','Already latched; explicit rearm required before next landing');return True
        if not self.client.service_is_ready():
            reply('unavailable','Screw driver offline; no command sent');return True
        request=LeadscrewControl.Request()
        request.group=0;request.action='charge_sequence' if charge else 'landing_sequence';request.direction=1 if charge else 0
        request.rpm=float(self.node.get_parameter('landing_rpm').value)
        request.distance_mm=50.;request.ramp_seconds=.1;request.deadline_unix=time.time()+2.5
        try:
            latch.parent.mkdir(parents=True,exist_ok=True)
            with latch.open('x') as f:
                json.dump(dict(received_at=time.time(),source=addr[0],message=msg),f)
                f.flush();os.fsync(f.fileno())
        except OSError as exc:
            reply('rejected',str(exc));return True
        self.pending=True
        try:future=self.client.call_async(request)
        except Exception as exc:
            self.pending=False;reply('unknown',str(exc));return True
        def done(f):
            self.pending=False
            try:
                result=f.result()
                status='accepted' if result.accepted else 'rejected'
                reply(status,result.message)
                self.node.get_logger().info(('Charge' if charge else 'Landing')+' 50mm sequential jog: '+status+' '+result.message)
            except Exception as exc:reply('unknown',str(exc))
        future.add_done_callback(done)
        reply('pending','Latched; awaiting driver acceptance; not physical completion')
        return True
