"""CH8 level requests and actual executor callback integration, no vehicle I/O."""
import math
import json
import time
import unittest
from copy import deepcopy
from unittest.mock import patch
from types import SimpleNamespace as NS

from air_ground_landing.guided_execution import (
    RcLandingRequestGate, LandingSwitchConfig, LandingSwitchResult,
    LandingSwitchState,
    RcAuthorizationGate, RcGateConfig,
)
import test_pilot_session as session_fixtures

scope = session_fixtures.scope


class LandingLevelTests(unittest.TestCase):
    def setUp(self):
        self.g=RcLandingRequestGate(LandingSwitchConfig())

    def sample(self,t,pwm=1900,**kw):
        args=dict(received_time_s=t,now_s=t,follow_active=True)
        args.update(kw)
        return self.g.evaluate([1500]*7+[pwm],**args)

    def test_high_requires_follow_but_no_low_edge(self):
        self.assertFalse(self.sample(1,follow_active=False).requested)
        self.assertTrue(self.sample(1.1).requested)
        self.assertTrue(self.sample(1.2).requested)

    def test_low_and_neutral_cancel_high_restores(self):
        for pwm in (1100,1500):
            self.setUp()
            self.assertTrue(self.sample(1).requested)
            self.assertFalse(self.sample(1.1,pwm).requested)
            self.assertTrue(self.sample(1.2).requested)

    def test_stale_and_missing_rc_block(self):
        self.assertTrue(self.sample(1).requested)
        self.assertFalse(self.sample(1.6,received_time_s=1).requested)
        self.assertFalse(self.g.evaluate(None,received_time_s=None,now_s=2,follow_active=True).requested)
        self.assertTrue(self.sample(2.1).requested)

    def test_invalid_times_block(self):
        for t,received in [(1.1,math.nan),(1.1,math.inf),(math.nan,1.1),(1.1,2),(.9,.9)]:
            self.setUp()
            self.sample(1)
            self.assertFalse(self.sample(t,received_time_s=received).requested)

    def test_reset_requires_follow_again_not_new_edge(self):
        self.sample(1)
        self.g.reset()
        self.assertFalse(self.sample(1.1,follow_active=False).requested)
        self.assertTrue(self.sample(1.2).requested)


class AdapterLandingLevelTests(unittest.TestCase):
    def setUp(self):
        self.f = session_fixtures.AdapterSessionTests()
        self.f.setUp()
        self.f.active()
        self.n = self.f.n
        gate = RcAuthorizationGate(RcGateConfig(channel=6))
        self.n._rc_result = lambda t: gate.evaluate(self.n.rc_channels,
            received_time_s=self.n.rc_received_s,now_s=t)
        self.n._session_rc = lambda t: scope['_session_rc'](self.n,t)

    def sample(self,t,ch8,ch6=2000):
        self.f.t=t
        scope['_rc'](self.n,NS(channels=[1500]*5+[ch6,1500,ch8]))
        return self.n.landing_switch.requested

    def test_actual_callbacks_level_request_and_cancel_between_ticks(self):
        self.assertTrue(self.sample(1.8,1900))
        self.assertFalse(self.sample(1.9,1100))
        self.assertTrue(self.sample(2.,1900))
        self.assertFalse(self.sample(2.01,1500))
        self.assertTrue(self.sample(2.02,1900))
        self.assertTrue(self.n.pilot_session.enabled)

    def test_guided_heartbeat_and_active_transaction_required_before_request(self):
        self.f.setUp()
        self.n=self.f.n
        self.f.authorize()
        self.n._session_rc=lambda t: scope['_session_rc'](self.n,t)
        self.n._rc_result=lambda t: RcAuthorizationGate(RcGateConfig(channel=6)).evaluate(
            self.n.rc_channels,received_time_s=self.n.rc_received_s,now_s=t)
        self.assertFalse(self.sample(1.65,1100))
        self.n.manager.update(now_s=1.7,current_mode='LOITER',desired_mode='GUIDED')
        self.f.t=1.8
        self.f.state('GUIDED')
        self.assertFalse(self.sample(1.81,1100))
        self.n.manager.update(now_s=1.82,current_mode='GUIDED',desired_mode='GUIDED')
        self.assertTrue(self.sample(1.83,1900))
        self.sample(1.84,1100)
        self.assertTrue(self.sample(1.85,1900))

    def test_pilot_override_requires_rc6_rearm_then_ch8_high_requests(self):
        self.sample(1.8,1100)
        self.assertTrue(self.sample(1.9,1900))
        self.f.t=2.
        self.f.state('LOITER')
        self.assertFalse(self.sample(2.01,1900))
        self.sample(2.1,1900,1000)
        self.sample(2.55,1900,1000)
        self.sample(2.6,1900)
        self.assertTrue(self.n.pilot_session.enabled)
        self.n.manager.update(now_s=2.61,current_mode='LOITER',desired_mode='GUIDED')
        self.f.t=2.62
        self.f.state('GUIDED')
        self.n.manager.update(now_s=2.63,current_mode='GUIDED',desired_mode='GUIDED')
        self.assertTrue(self.sample(2.64,1900))
        self.sample(2.65,1100)
        self.assertTrue(self.sample(2.66,1900))

    def test_native_land_handover_keeps_confirmed_request_only(self):
        self.sample(1.8,1100)
        self.assertTrue(self.sample(1.9,1900))
        self.n.manager.update(now_s=2.,current_mode='GUIDED',desired_mode='LAND')
        self.assertTrue(self.sample(2.01,1900))
        self.f.t=2.02
        self.f.state('LAND')
        self.n.manager.update(now_s=2.03,current_mode='LAND',desired_mode='LAND')
        self.assertTrue(self.sample(2.04,1900))
        self.assertFalse(self.sample(2.05,1100))
        self.assertFalse(self.sample(2.06,1900))

    def test_ch8_low_overrides_terminal_land_latch(self):
        self.n.manager.update(now_s=2.0, current_mode="GUIDED", desired_mode="LAND")
        self.n.manager.update(now_s=2.1, current_mode="LAND", desired_mode="LAND")
        self.n.vehicle_state.mode = "LAND"
        low = LandingSwitchResult(
            LandingSwitchState.FOLLOW_INACTIVE,
            1100,
            0.0,
            explicit_low=True,
        )
        resume = scope["_desired_mode"](
            self.n,
            now_s=2.2,
            rc=NS(authorized=True),
            landing=low,
            guided_candidate=None,
            continuity=NS(keep_guided=True),
            terminal_land_latched=True,
        )
        self.assertEqual(resume, ("GUIDED", "CH8_EXPLICIT_LOW_RESUME_GUIDED"))

        rollback = scope["_desired_mode"](
            self.n,
            now_s=2.3,
            rc=NS(authorized=True),
            landing=low,
            guided_candidate=None,
            continuity=NS(keep_guided=False),
            terminal_land_latched=True,
        )
        self.assertEqual(rollback, (None, "CH8_EXPLICIT_LOW_EXIT_LAND"))

    def test_actual_guided_descent_output_uses_ch8_level(self):
        n=self.n
        candidate=NS(velocity=NS(x=.01,y=0.))
        n.owner='IBVS_GUIDED'
        n.candidates={'IBVS_GUIDED':(candidate,1.7)}
        n._authorized_candidate=lambda t,rc:(candidate,'test observation')
        n.descent_stream_next_s=math.inf
        n.descent_policy=session_fixtures.GuidedDescent()
        n.landing_disarm=session_fixtures.LandingDisarm()
        n.disarm_status='INACTIVE'
        n.disarm_sent_s=n.disarm_future=None
        n.rangefinder_filter=NS(status=lambda t:NS(healthy=True,median_m=.5))
        outputs,statuses=[],[]
        n.actual_publisher=NS(publish=outputs.append)
        n.preview_publisher=n.landing_request_publisher=NS(publish=lambda m:None)
        n.status_publisher=NS(publish=lambda m:statuses.append(json.loads(m.data)))
        n.command_client=NS(service_is_ready=lambda:False)
        n._dispatch_mode_request=lambda action:self.fail('Unexpected mode request')
        n.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:None))
        class Target:
            FRAME_LOCAL_NED=1
            IGNORE_PX,IGNORE_PY,IGNORE_PZ=1,2,4
            IGNORE_AFX,IGNORE_AFY,IGNORE_AFZ=64,128,256
            IGNORE_YAW,IGNORE_YAW_RATE=1024,2048
            def __init__(self):
                self.velocity=NS(x=0.,y=0.,z=0.)
                self.header=NS(stamp=None)
        def tick(t,pwm):
            self.f.t=t
            self.f.state('GUIDED')
            n.descent_telemetry.update({
                'pose':(NS(pose=NS(orientation=NS(x=0.,y=0.,z=0.,w=1.))),t),
                'velocity':(NS(twist=NS(linear=NS(x=0.,y=0.,z=0.))),t),
                'extended':(NS(landed_state=2),t),
                'vision':(NS(data='{"healthy":true,"aligned":true}'),t)})
            n.candidates['IBVS_GUIDED']=(candidate,t)
            self.sample(t,pwm)
            scope['_tick_companion_descent'](n)
        with patch.dict(scope,PositionTarget=Target,deepcopy=deepcopy,time=time,
                        ExtendedState=NS(LANDED_STATE_IN_AIR=2,LANDED_STATE_ON_GROUND=1)):
            for i in range(8):
                tick(1.8+i*.1,1900)
            self.assertTrue(outputs)
            self.assertEqual(outputs[-1].velocity.z,-.1)
            self.assertEqual(statuses[-1]['landing_switch_state'],'REQUESTED')
            tick(2.6,1100)
            self.assertEqual(statuses[-1]['landing_switch_state'],'READY')
            self.assertEqual(outputs[-1].velocity.z,0.)
            for i in range(7):
                tick(2.7+i*.1,1900)
            self.assertEqual(statuses[-1]['landing_switch_state'],'REQUESTED')
            self.assertEqual(outputs[-1].velocity.z,-.1)


if __name__=='__main__':
    unittest.main()
