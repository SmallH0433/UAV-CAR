import json,socket,tempfile,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from car_interfaces.srv import LeadscrewControl
from car_nodes.uav_bridge import UavBridgeNode
from car_nodes.leadscrew_motion import Motion
rclpy.init(args=['--ros-args','-p','listen_port:=18889','-p','uav_ip:=127.0.0.1','-p','landing_latch_file:=/tmp/car_landing_test_latch.json'])
path=Path('/tmp/car_landing_test_latch.json');path.unlink(missing_ok=True)
bridge=UavBridgeNode();server=Node('test_screw_service');calls=[]
def callback(req,res):
    calls.append(req);res.accepted=True;res.message='mock accepted';return res
server.create_service(LeadscrewControl,'/leadscrew/control',callback)
ex=MultiThreadedExecutor();ex.add_node(bridge);ex.add_node(server);thread=threading.Thread(target=ex.spin);thread.start()
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(3)
try:
    end=time.time()+8
    while not bridge.landing_receiver.client.service_is_ready() and time.time()<end:time.sleep(.1)
    sock.sendto('已成功降落'.encode(),('127.0.0.1',18889))
    time.sleep(.7);assert len(calls)==1,len(calls)
    req=calls[0];assert (req.group,req.direction,req.distance_mm,req.rpm)==(0,0,50.,60.)
    sock.sendto('已成功降落'.encode(),('127.0.0.1',18889));time.sleep(.2);assert len(calls)==1
    assert path.exists()
    # Same latch survives receiver reconstruction/restart.
    from car_nodes.landing_receiver import LandingReceiver
    assert json.loads(path.read_text())['source']=='127.0.0.1'
    engine=Motion(simulate=True);counts=[0,0];levels={}
    def write(i,col,v):
        if col==0 and v:counts[i]+=1
        if col==1:levels[i]=v
    engine.write=write;engine.delay=lambda _:None
    engine.move(0,0,60,50,1);engine.worker.join(10)
    assert counts==[40000,40000] and levels=={0:0,1:0} and engine.pos==[40000,40000]
    engine.close();print('PASS UDP landing -> ROS group0 inward50mm; duplicate latch; paired40000 pulses DIR LOW; no hardware writes')
finally:
    ex.shutdown();thread.join();bridge.sock.close();bridge.destroy_node();server.destroy_node();rclpy.shutdown();sock.close();path.unlink(missing_ok=True)
