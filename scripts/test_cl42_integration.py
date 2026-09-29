import importlib.util, tempfile, time, os
spec=importlib.util.spec_from_file_location('motion','/home/ubuntu/CAR_ws/src/car_nodes/car_nodes/leadscrew_motion.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.LOCK_FILE=tempfile.mktemp(prefix='cl42_test_')
events=[]
class Fake:
    def __init__(self,pin,value): self.pin=pin;events.append((pin,value))
    def write(self,value): events.append((self.pin,value))
    def close(self): pass
engine=m.Motion(output_factory=Fake)
try:
    assert events==[(17,0),(27,0),(22,0),(23,0),(24,0),(5,0)]
    try: other=m.Motion(output_factory=Fake);raise AssertionError('ownership not enforced')
    except BlockingIOError: pass
    engine.delay=lambda seconds: None
    engine.move(0,0,60,.01,.05);engine.worker.join(2)
    assert engine.pos==[8,8]
    assert (27,0) in events and (24,0) in events
    assert sum(x==(17,1) for x in events)==8 and sum(x==(23,1) for x in events)==8
    events.clear();engine.reset(2);engine.worker.join(2)
    assert (5,1) in events and (5,0) in events and (17,1) not in events and (23,1) not in events
    for args in [(1,0,float('nan'),1,1),(1,0,6,63,1),(1,0,6,1,0),(3,0,6,1,1)]:
        try: engine.move(*args);raise AssertionError('invalid accepted')
        except ValueError: pass
    engine.delay=lambda seconds: time.sleep(.001)
    engine.move(0,1,60,1,.05);time.sleep(.25);engine.stop();length=len(events);time.sleep(.05)
    assert len(events)==length and not engine.snapshot()['busy']
    assert engine.pos[0]==engine.pos[1] and engine.pos[0]<8
finally:
    engine.close();os.unlink(m.LOCK_FILE)
print('PASS: wiring, direction mapping, exact pulse count, EN reset, ownership, validation, stop')
