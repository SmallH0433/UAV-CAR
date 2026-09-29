import importlib.util
p='/home/ubuntu/CAR_ws/src/car_nodes/car_nodes/leadscrew_motion.py'
s=importlib.util.spec_from_file_location('m',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
e=m.Motion(simulate=True);events=[];e.delay=lambda _:None
e.write=lambda i,c,v:events.append(i) if c==0 and v==1 else None
e.landing_sequence();e.worker.join(10)
assert events==[0]*40000+[1]*40000 and e.pos==[40000,40000]
e.close()
e=m.Motion(simulate=True);calls=[]
def pulses(*args): calls.append(args);e.cancel.set()
e.pulses=pulses;e.landing_sequence();e.worker.join(2)
assert len(calls)==1 and calls[0]==((0,),0,40000,100*1600/60,.1)
e.close();print('PASS sequential counts, parameters, stop prevents second motor')
