import tempfile
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import Future
from car_nodes.leadscrew_motion import Motion
from car_nodes.landing_receiver import LandingReceiver
for direction in (0,1):
 e=Motion(simulate=True);events=[];dirs=[];e.delay=lambda _:None
 def write(i,c,v):
  if c==0 and v:events.append(i)
  if c==1:dirs.append((i,v))
 e.write=write;e.landing_sequence(direction);e.worker.join(10)
 assert events==[0]*40000+[1]*40000 and dirs==[(0,direction),(1,direction)]
 assert e.pos==[40000*(1 if direction==0 else -1)]*2;e.close()
with tempfile.TemporaryDirectory() as d:
 calls=[]
 class Client:
  def service_is_ready(self):return True
  def call_async(self,req):
   calls.append(req);f=Future();f.set_result(SimpleNamespace(accepted=True,message='mock'));return f
 class Node:
  def declare_parameter(self,*a):pass
  def get_parameter(self,k):return SimpleNamespace(value=str(Path(d)/'latch.json') if k=='landing_latch_file' else 100.)
  def create_client(self,*a):return Client()
  def create_service(self,*a):pass
  def get_logger(self):return SimpleNamespace(info=lambda _:None)
 class Sock:
  def sendto(self,*a):pass
 h=LandingReceiver(Node())
 for text in ('已成功降落','已成功降落','充电完成','充电完成'):
  assert h.handle(text.encode(),('192.168.50.3',9999),Sock(),'192.168.50.3')
 assert [x.action for x in calls]==['landing_sequence','charge_sequence']
 assert all(x.rpm==100 and x.ramp_seconds==.1 and x.distance_mm==50 for x in calls)
print('PASS both message routes, independent duplicate latches, both sequential directions and exact counts')
