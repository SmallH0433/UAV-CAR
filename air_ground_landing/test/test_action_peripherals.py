import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import math

from air_ground_landing.action_execution import ActionKind
from air_ground_landing.executor_peripherals import StartupGuidedRecovery, TargetEchoMonitor
from air_ground_landing.follow_tone_policy import FollowTonePolicy, FollowToneEvent, TUNES
from air_ground_landing.legacy_mavlink_tune import encode_legacy_play_tune, MAVLINK_V2_MAGIC, PLAY_TUNE_MSG_ID


class RecoveryTests(unittest.TestCase):
    def test_startup_guided_waits_then_retries_until_heartbeat(self):
        r=StartupGuidedRecovery()
        update=lambda t,mode='GUIDED': r.update(t,fresh=True,connected=True,armed=True,mode=mode,owned=False)
        self.assertFalse(update(0))
        self.assertTrue(r.pending)
        self.assertFalse(update(.99))
        self.assertTrue(update(1))
        self.assertTrue(update(2))
        self.assertFalse(update(3,'LOITER'))
        self.assertFalse(r.pending)
        self.assertFalse(update(4))  # A later pilot GUIDED choice must win.

    def test_no_rollback_for_landed_native_land_or_owned_action(self):
        for mode,armed,owned in [('LAND',True,False),('GUIDED',False,False),('GUIDED',True,True)]:
            r=StartupGuidedRecovery()
            for now in (0,2):
                self.assertFalse(r.update(now,fresh=True,connected=True,armed=armed,mode=mode,owned=owned))

    def test_stale_state_resets_grace_and_manual_mode_cancels_pending(self):
        r=StartupGuidedRecovery()
        args=dict(connected=True,armed=True,owned=False,mode='GUIDED')
        r.update(0,fresh=True,**args)
        self.assertFalse(r.update(2,fresh=False,**args))
        self.assertFalse(r.update(3,fresh=True,**args))
        self.assertFalse(r.update(3.5,fresh=True,**dict(args,mode='STABILIZE')))
        self.assertFalse(r.update(5,fresh=True,**args))


class EchoTests(unittest.TestCase):
    def test_freshness_streak_and_comparison(self):
        m=TargetEchoMonitor(.5)
        m.receive(0,(.1,.2,0),1,0)
        m.receive(.2,(.2,.3,0),1,0)
        s=m.status(.3,((.1,.2,0),1,0),.2)
        self.assertTrue(s['target_echo_continuous'])
        self.assertAlmostEqual(s['target_echo_velocity_difference_mps']['x'],.1)
        self.assertFalse(m.status(.8)['target_echo_fresh'])
        m.receive(.9,(0,0,0),1,0)
        self.assertEqual(m.streak,1)

    def test_no_difference_for_masked_different_frame_stale_send_or_nan(self):
        m=TargetEchoMonitor()
        m.receive(1,(0,0,0),1,0)
        for sent,t in [(((0,0,0),8,0),1),(((0,0,0),1,8),1),(((0,0,0),1,0),0)]:
            self.assertIsNone(m.status(1.1,sent,t)['target_echo_velocity_difference_mps'])
        m.receive(2,(math.nan,0,0),1,0)
        self.assertEqual(m.count,1)


def load_mixin():
    root=Path(__file__).resolve().parents[1]
    ws=root/('ros2_ws_v2' if (root/'ros2_ws_v2').exists() else 'ros2_ws')
    path=ws/'src/air_ground_landing_ros2/air_ground_landing_ros2/action_peripherals.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    # Run real methods without importing ROS on Windows; no replacement logic.
    tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    class Mavlink(NS):
        FRAMING_OK=1
        def __init__(self): super().__init__(header=NS(stamp=None))
    ns=dict(globals(),Mavlink=Mavlink,PositionTarget=object,
            CommandLong=NS(Request=NS),qos_profile_sensor_data=object())
    exec(compile(tree,str(path),'exec'),ns)
    return ns['ActionPeripherals'],ns['PERIPHERAL_DEFAULTS']


Mixin,Defaults=load_mixin()


class FakeNode(Mixin):
    def __init__(self,approved=True,**params):
        self.params=dict(Defaults,mavros_command_service='/mavros/cmd/command',
                         tone_output_enabled=True,allow_target_echo_request=True)
        self.params.update(params)
        self.now=0
        self.sent=[]; self.requests=[]; self.futures=[]; self.modes=[]; self.invalidations=[]
        self.guided_mode='GUIDED'; self.land_mode='LAND'; self.output_enabled=approved
        self.state_maximum_age_s=2.; self.state_received_s=0
        self.vehicle_state=NS(connected=True,armed=True,mode='GUIDED')
        self.lifecycle=NS(active=False,retaining_control=False,fallback_mode='LOITER')
        self.pilot_session=NS(invalidate=lambda reason,t:self.invalidations.append(reason))
        self._init_peripherals(approved)
    def get_parameter(self,name): return NS(value=self.params[name])
    def create_publisher(self,*a): return NS(publish=self.sent.append)
    def create_subscription(self,*a): pass
    def create_client(self,*a): return NS(service_is_ready=lambda:True,call_async=self.call)
    def call(self,request):
        future=Future(); self.requests.append(request); self.futures.append(future); return future
    def get_clock(self): return NS(now=lambda:NS(to_msg=lambda:None))
    def _now_s(self): return self.now
    @staticmethod
    def _fresh(t,age,now): return t is not None and 0<=now-t<=age
    def _request_mode(self,mode,now): self.modes.append(mode)


class PeripheralWiringTests(unittest.TestCase):
    def test_interval_request_uses_511_85_5hz_and_ack_is_not_echo(self):
        n=FakeNode()
        n._ensure_target_echo_interval(0)
        self.assertEqual((n.requests[0].command,n.requests[0].param1,n.requests[0].param2),(511,85,200000))
        n.futures[0].set_result(NS(success=True,result=0))
        self.assertTrue(n.target_echo_interval_confirmed)
        self.assertFalse(n.echo_monitor.fresh(0))
        n._ensure_target_echo_interval(3)
        self.assertEqual(len(n.requests),1)

    def test_rejection_timeout_and_late_ack(self):
        n=FakeNode(); n._ensure_target_echo_interval(0)
        n.futures[0].set_result(NS(success=False,result=1))
        n._ensure_target_echo_interval(1)
        self.assertEqual(len(n.requests),1)
        n._ensure_target_echo_interval(2)
        n._ensure_target_echo_interval(4)  # Lost reply: bounded retry.
        n.futures[1].set_result(NS(success=True,result=0))
        self.assertFalse(n.target_echo_interval_confirmed)
        n.futures[2].set_result(NS(success=True,result=0))
        self.assertTrue(n.target_echo_interval_confirmed)

    def test_disconnect_invalidates_old_ack_and_requests_again(self):
        n=FakeNode(); n._peripheral_pre_tick(0)
        old=n.futures[0]
        n.vehicle_state.connected=False; n._peripheral_pre_tick(.2)
        old.set_result(NS(success=True,result=0))
        self.assertFalse(n.target_echo_interval_confirmed)
        n.vehicle_state.connected=True; n._peripheral_pre_tick(.3)
        self.assertEqual(len(n.requests),2)

    def test_preview_never_emits_tone_interval_or_recovery_command(self):
        n=FakeNode(approved=False)
        n._peripheral_pre_tick(0); n._peripheral_pre_tick(1.1)
        n._emit_tones((FollowToneEvent.FOLLOW_ACTIVE,))
        self.assertEqual((n.sent,n.requests,n.modes),([],[],[]))

    def test_startup_recovery_invokes_mode_request_without_rc_session(self):
        n=FakeNode(); self.assertTrue(n._peripheral_pre_tick(0))
        n._peripheral_pre_tick(1.1)
        self.assertEqual(n.modes,['LOITER'])
        self.assertEqual(n.invalidations,['ORPHANED_GUIDED_ROLLBACK'])

    def test_follow_and_hybrid_descent_tones_need_echo_and_real_output(self):
        n=FakeNode(); n.lifecycle.active=True
        snapshot=NS(authorized=True,candidate_fresh=True)
        status=NS(action='FOLLOW',detail='FOLLOW_ACTIVE')
        n._update_tones(snapshot,status,0)
        self.assertEqual(n.sent,[])
        n.last_sent_s=0; n.last_sent_target=((.1,0,0),1,0)
        n.echo_monitor.receive(0,(.1,0,0),1,0)
        n._update_tones(snapshot,status,0)
        self.assertEqual(n.tone_events,(FollowToneEvent.FOLLOW_ACTIVE,))
        self.assertEqual(n.sent[0].msgid,258)
        status.action='LAND'; status.detail='GUIDED_TRACK_DESCENT'
        n.last_sent_target=((0,0,-.1),1,0)
        n._update_tones(snapshot,status,.1)
        self.assertEqual(n.tone_events,(FollowToneEvent.LANDING_ACTIVE,))
        n._update_tones(snapshot,status,.2)
        self.assertEqual(n.tone_events,())
        status.detail='TAG_LOST_GUIDED_HOLD'; n.last_sent_target=((0,0,0),1,0)
        n._update_tones(snapshot,status,.3)
        self.assertEqual(n.tone_events,())
        n.vehicle_state.mode='LOITER'; n.lifecycle.active=False
        n._update_tones(snapshot,status,.4)
        self.assertEqual(n.tone_events,(FollowToneEvent.EXIT_CONFIRMED,))


if __name__=='__main__': unittest.main()
