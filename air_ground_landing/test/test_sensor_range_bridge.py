import math
import struct
import unittest
from sensor_range_bridge import RangeGate

class Tests(unittest.TestCase):
    def sample(self, gate, *, boot=100, distance=50, ident=0, orient=25, **kw):
        args=dict(sysid=200,compid=88,msgid=132,framing=1,age=.01,
                  payload=struct.pack('<IHHHBBBB',boot,2,1200,distance,0,ident,orient,0))
        args.update(kw)
        return gate.decode(**args)
    def test_valid(self):
        self.assertEqual(self.sample(RangeGate()),(.5,.02,12.))
    def test_whitelist(self):
        for kw in ({'sysid':1},{'compid':1},{'ident':1},{'orient':0},{'msgid':173}):
            self.assertIsNone(self.sample(RangeGate(),**kw))
    def test_stale_and_corrupt(self):
        for kw in ({'age':.31},{'age':-1},{'framing':2},{'payload':b''}):
            self.assertTrue(math.isnan(self.sample(RangeGate(),**kw)[0]))
    def test_boundaries_not_touchdown_evidence(self):
        for d in (0,2,1200,65535):
            self.assertTrue(math.isnan(self.sample(RangeGate(),distance=d)[0]))
    def test_duplicates_and_reboot(self):
        gate=RangeGate()
        self.sample(gate)
        self.assertIsNone(self.sample(gate))
        self.assertIsNone(self.sample(gate,boot=99))
        self.assertEqual(self.sample(gate,boot=101)[0],.5)
    def test_v2_truncation(self):
        payload=struct.pack('<IHHHBBBB',100,2,1200,50,0,0,25,0)[:-1]
        self.assertEqual(self.sample(RangeGate(),payload=payload)[0],.5)
    def test_bad_quality(self):
        payload=struct.pack('<IHHHBBBB',100,2,1200,50,0,0,25,0).ljust(38,b'\0')+b'\x01'
        self.assertTrue(math.isnan(self.sample(RangeGate(),payload=payload)[0]))

    def new_epoch(self, gate, allowed=True):
        result=None
        for i in range(7):
            result=self.sample(gate,boot=100+i*100,now=1+i*.1,recovery_allowed=allowed)
        return result
    def test_disarmed_recovery(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        self.assertEqual(self.new_epoch(gate)[0],.5)
        self.assertEqual(gate.rebase_count,1)
    def test_no_recovery_armed_or_unknown(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        self.assertIsNone(self.new_epoch(gate,False))
        self.assertEqual(gate.last_boot,10000)
    def test_not_single_sample(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        self.assertTrue(math.isnan(self.sample(gate,boot=100,now=1,recovery_allowed=True)[0]))
        self.assertEqual(gate.last_boot,10000)
    def test_repeated_old_samples_cannot_rebase(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        for i in range(10):
            self.sample(gate,boot=100,now=1+i*.1,recovery_allowed=True)
        self.assertEqual(gate.rebase_count,0)
    def test_authorization_loss_cancels_dwell(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        for i in range(4):self.sample(gate,boot=100+i*100,now=1+i*.1,recovery_allowed=True)
        self.sample(gate,boot=500,now=1.4,recovery_allowed=False)
        self.sample(gate,boot=600,now=1.5,recovery_allowed=True)
        self.assertEqual(gate.rebase_count,0)
    def test_bad_distance_cancels_dwell(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        for i in range(4):self.sample(gate,boot=100+i*100,now=1+i*.1,recovery_allowed=True)
        self.sample(gate,boot=500,distance=2,now=1.4,recovery_allowed=True)
        self.sample(gate,boot=600,now=1.5,recovery_allowed=True)
        self.assertEqual(gate.rebase_count,0)
    def test_long_gap_restarts_confirmation(self):
        gate=RangeGate(); self.sample(gate,boot=10000)
        self.sample(gate,boot=100,now=1,recovery_allowed=True)
        self.sample(gate,boot=200,now=2,recovery_allowed=True)
        self.assertEqual(gate.rebase_count,0)

if __name__=='__main__':
    unittest.main()
