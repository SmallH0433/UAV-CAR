"""Session policy and real adapter method tests, without ROS or vehicle access."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from air_ground_landing.guided_descent import GuidedDescent, DescentInput
from air_ground_landing.landing_disarm import LandingDisarm, LandingEvidence

from air_ground_landing.guided_execution import (
    PilotSessionGate, RcGateResult, RcGateState, ModeTransitionManager,
    ModeTransitionConfig, ModeTransitionPhase, RcAuthorizationGate, RcGateConfig,
    FollowContinuityGuard, FollowContinuityConfig, TerminalLandLatch,
    TerminalLandConfig, RcLandingRequestGate, LandingSwitchConfig,
    HorizontalVelocityLimiter, HorizontalVelocityLimitConfig,
    follow_mode_allowed,
)


def rc(high=True):
    return RcGateResult(RcGateState.AUTHORIZED if high else RcGateState.ABORT,
                        2000 if high else 1000, 0.)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.g = PilotSessionGate()
        self.g.observe_mode('LOITER', expected_mode=None, now_s=0)

    def update(self, t, high=True, **kw):
        args = dict(now_s=t, healthy=True, rc=rc(high), received_s=t, entry_allowed=True)
        args.update(kw)
        return self.g.update(**args)

    def authorize(self):
        self.update(1, False)
        self.update(1.5, False)
        self.assertTrue(self.update(1.6))

    def test_startup_high_never_authorizes(self):
        for t in range(31):
            self.assertFalse(self.update(t))

    def test_one_low_sample_cannot_complete_dwell(self):
        self.update(1, False)
        self.update(1.45, False, received_s=1)
        self.assertFalse(self.update(1.46))

    def test_low_gap_and_neutral_reset(self):
        self.update(1, False)
        self.update(2, False)
        self.assertFalse(self.update(2.1))
        self.update(3, False)
        self.update(3.4, False)
        self.update(3.5, rc=RcGateResult(RcGateState.STANDBY, 1500, 0))
        self.assertFalse(self.update(3.6))

    def test_external_hold_override_requires_new_low_cycle(self):
        self.authorize()
        self.g.observe_mode('GUIDED', expected_mode='GUIDED', now_s=2)
        self.assertTrue(self.g.observe_mode('ALT_HOLD', expected_mode=None, now_s=3))
        for t in range(4, 35):
            self.assertFalse(self.update(t))
        self.update(35, False)
        self.update(35.5, False)
        self.assertTrue(self.update(35.6))
        self.assertEqual(self.g.session_sequence, 2)

    def test_own_land_and_rollback_are_not_override(self):
        self.authorize()
        for t, mode in enumerate(('GUIDED', 'LAND', 'LOITER'), 2):
            self.assertFalse(self.g.observe_mode(mode, expected_mode=mode, now_s=t))
        self.assertTrue(self.g.enabled)

    def test_disconnect_and_stale_rc_require_rearm(self):
        self.authorize()
        self.update(2, healthy=False)
        self.assertFalse(self.update(3))
        self.update(4, False)
        self.update(4.5, False)
        self.update(4.6, rc=RcGateResult(RcGateState.STALE, 1000, 1))
        self.assertFalse(self.update(5))

    def test_early_high_not_queued_for_later_entry(self):
        self.update(1, False)
        self.update(1.5, False)
        self.assertFalse(self.update(1.6, entry_allowed=False))
        self.assertFalse(self.update(1.7))

    def test_high_requires_a_new_sample_after_low_confirmation(self):
        self.update(1, False)
        self.update(1.5, False)
        self.assertFalse(self.update(1.5, received_s=1.5))

    def test_long_gap_before_high_requires_new_low_confirmation(self):
        self.update(1, False)
        self.update(1.5, False)
        self.assertFalse(self.update(2.1))

    def test_invalid_sample_time_never_authorizes(self):
        for received in (None, math.nan, math.inf, 2.0, .1):
            with self.subTest(received=received):
                self.setUp()
                self.update(1, False)
                self.update(1.5, False)
                self.assertFalse(self.update(1.6, received_s=received))

    def test_mode_change_during_rearm_discards_earlier_low_samples(self):
        self.update(1, False)
        self.update(1.5, False)
        self.assertTrue(self.g.observe_mode('ALT_HOLD', expected_mode=None, now_s=1.55))
        self.assertFalse(self.update(1.6))


# Execute the actual callback/session/mode-gate bodies without importing ROS.
# This tests their integration with the real mode manager, not rewritten logic.
ADAPTER = (Path(__file__).resolve().parents[1] / 'ros2_ws/src/'
           'air_ground_landing_ros2/air_ground_landing_ros2/guided_executor.py')
tree = ast.parse(ADAPTER.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GuidedExecutor')
methods = {'_vehicle_state', '_rc', '_landing_rc', '_session_rc', '_desired_mode',
           '_tick_companion_descent', '_dispatch_mode_request'}
isolated = ast.Module(body=[ast.ImportFrom(module='__future__',
    names=[ast.alias(name='annotations')], level=0)] +
    [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods],
    type_ignores=[])
scope = dict(ModeTransitionPhase=ModeTransitionPhase, RcGateState=RcGateState,
             RcGateResult=RcGateResult, json=json, math=math, Bool=NS, String=NS,
             LandingEvidence=LandingEvidence, DescentInput=DescentInput,
             follow_mode_allowed=follow_mode_allowed)
scope['SetMode'] = NS(Request=NS)
exec(compile(ast.fix_missing_locations(isolated), str(ADAPTER), 'exec'), scope)


class AdapterSessionTests(unittest.TestCase):
    def setUp(self):
        self.t = 1.
        self.n = NS(pilot_session=PilotSessionGate(),
            manager=ModeTransitionManager(ModeTransitionConfig()),
            vehicle_state=NS(connected=False, armed=False, mode='UNKNOWN'),
            execution_enabled=True, descent_telemetry={}, rc_received_s=self.t,
            landing_switch=RcLandingRequestGate(LandingSwitchConfig()),
            terminal_land_latch=TerminalLandLatch(TerminalLandConfig()),
            follow_continuity=FollowContinuityGuard(FollowContinuityConfig()),
            horizontal_limiter=HorizontalVelocityLimiter(HorizontalVelocityLimitConfig()))
        self.n._now_s = lambda: self.t
        self.n.guided_mode = 'GUIDED'
        self.n.land_mode = 'LAND'
        self.n.rc_channels = [1500] * 8
        self.n._landing_rc = lambda *args: scope['_landing_rc'](self.n, *args)
        self.n._descent_observe = lambda k, m: self.n.descent_telemetry.update({k:(m,self.t)})
        self.n._rc_result = lambda t: rc(self.high)
        self.high = True

    def state(self, mode, armed=True, connected=True):
        scope['_vehicle_state'](self.n, NS(mode=mode, armed=armed, connected=connected))

    def tick(self, t, high=True):
        self.t = t
        self.high = high
        self.n.rc_received_s = t
        return scope['_session_rc'](self.n, t)

    def authorize(self):
        self.state('LOITER')
        self.tick(1, False)
        self.tick(1.5, False)
        self.assertTrue(self.tick(1.6).authorized)

    def active(self):
        self.authorize()
        self.request = self.n.manager.update(now_s=self.t, current_mode='LOITER', desired_mode='GUIDED')
        self.t = 1.7
        self.state('GUIDED')
        self.n.manager.update(now_s=self.t, current_mode='GUIDED', desired_mode='GUIDED')

    def test_each_pilot_mode_cancels_session_and_pending_callback(self):
        for mode in ('ALT_HOLD', 'LOITER', 'STABILIZE', 'RTL', 'LAND'):
            with self.subTest(mode=mode):
                self.setUp()
                self.active()
                self.state(mode)
                self.assertFalse(self.n.pilot_session.enabled)
                self.assertEqual(self.n.manager.phase, ModeTransitionPhase.IDLE)
                self.assertIsNone(self.n.manager.on_service_result(
                    sequence=self.request.sequence, mode_sent=False, now_s=2))
                for t in range(2, 33):
                    self.t = t
                    self.state(mode)
                    self.assertFalse(self.tick(t).authorized)

    def test_override_while_guided_request_pending(self):
        self.authorize()
        req = self.n.manager.update(now_s=self.t, current_mode='LOITER', desired_mode='GUIDED')
        self.state('ALT_HOLD')
        self.assertFalse(self.n.pilot_session.enabled)
        self.assertIsNone(self.n.manager.on_service_result(sequence=req.sequence, mode_sent=True, now_s=2))
        # A request already sent cannot be recalled; a late GUIDED heartbeat
        # must still not resume setpoints or create a new transaction.
        self.state('GUIDED')
        self.assertFalse(self.tick(2).authorized)
        self.assertEqual(self.n.manager.phase, ModeTransitionPhase.IDLE)

    def test_callbacks_catch_mode_change_and_return_between_ticks(self):
        self.active()
        self.state('STABILIZE')
        self.state('LOITER')
        self.assertFalse(self.tick(2).authorized)

    def test_normal_guided_then_land_confirmation_keeps_session(self):
        self.active()
        self.assertTrue(self.n.pilot_session.enabled)
        self.n.manager.update(now_s=2, current_mode='GUIDED', desired_mode='LAND')
        self.state('LAND')
        self.assertTrue(self.tick(2).authorized)

    def test_disarm_disconnect_and_state_timeout_cancel(self):
        for fault in ('disarm', 'disconnect', 'stale'):
            with self.subTest(fault=fault):
                self.setUp()
                self.active()
                if fault == 'stale':
                    self.tick(10)
                else:
                    self.state('GUIDED', armed=fault != 'disarm', connected=fault != 'disconnect')
                self.assertFalse(self.n.pilot_session.enabled)
                self.assertEqual(self.n.manager.phase, ModeTransitionPhase.IDLE)

    def test_rc_low_preserves_owned_rollback(self):
        self.active()
        self.assertFalse(self.tick(2, False).authorized)
        req = self.n.manager.update(now_s=2, current_mode='GUIDED', desired_mode=None)
        self.assertTrue(req.rollback)
        self.assertEqual(req.mode, 'LOITER')

    def test_pilot_override_during_rc_low_rollback_cancels_late_nack(self):
        self.active()
        self.tick(2, False)
        req = self.n.manager.update(now_s=2, current_mode='GUIDED', desired_mode=None)
        self.state('ALT_HOLD')
        self.assertEqual(self.n.manager.phase, ModeTransitionPhase.IDLE)
        self.assertIsNone(self.n.manager.on_service_result(
            sequence=req.sequence, mode_sent=False, now_s=3))
        self.assertFalse(self.tick(3).authorized)

    def test_legacy_orphan_rollback_and_latch_cannot_bypass_lockout(self):
        result = scope['_desired_mode'](self.n, now_s=1, rc=rc(), landing=None,
                    guided_candidate=object(), continuity=None, terminal_land_latched=True)
        self.assertEqual(result, (None, 'STARTUP_REARM_REQUIRED'))

    def test_actual_guided_tick_emits_no_modes_setpoints_or_disarm_after_override(self):
        self.active()
        n = self.n
        n._session_rc = lambda t: scope['_session_rc'](n, t)
        n.descent_stream_next_s = math.inf
        n.guided_mode = 'GUIDED'
        n.owner = 'IBVS_GUIDED'
        n.candidates = {'IBVS_GUIDED': (NS(velocity=NS(x=.1, y=0)), self.t)}
        n._authorized_candidate = lambda t, gate: (n.candidates['IBVS_GUIDED'][0], '')
        n.rc_channels = (1500,1500,1500,1500,1500,2000,1500,2000)
        n.descent_policy = GuidedDescent()
        n.landing_disarm = LandingDisarm()
        n.disarm_status = 'INACTIVE'
        n.disarm_sent_s = None
        n.disarm_future = None
        n.rangefinder_filter = NS(status=lambda t: NS(healthy=True, median_m=.08))
        modes, outputs, commands, statuses = [], [], [], []
        n._dispatch_mode_request = modes.append
        n.actual_publisher = NS(publish=outputs.append)
        n.preview_publisher = NS(publish=lambda m: None)
        n.landing_request_publisher = NS(publish=lambda m: None)
        n.status_publisher = NS(publish=statuses.append)
        n.command_client = NS(service_is_ready=lambda: False, call_async=commands.append)
        self.state('ALT_HOLD')
        # Even fresh candidates and CH8 high cannot revive the revoked session.
        for t in range(2, 34):
            self.t = t
            self.state('ALT_HOLD')
            n.rc_received_s = t
            n.candidates['IBVS_GUIDED'] = (n.candidates['IBVS_GUIDED'][0], t)
            scope['_tick_companion_descent'](n)
        self.assertEqual((modes, outputs, commands), ([], [], []))
        self.assertFalse(json.loads(statuses[-1].data)['pilot_session_authorized'])

    def test_rc_callback_observes_between_tick_neutral_and_high_samples(self):
        for pulse in (1500, 2000):
            with self.subTest(pulse=pulse):
                self.setUp()
                n = self.n
                self.state('LOITER')
                gate = RcAuthorizationGate(RcGateConfig(channel=6))
                n._rc_result = lambda t: gate.evaluate(n.rc_channels,
                    received_time_s=n.rc_received_s, now_s=t)
                n._session_rc = lambda t: scope['_session_rc'](n, t)
                for t, pwm in ((1.,1000),(1.2,pulse),(1.45,1000),(1.5,2000)):
                    self.t = t
                    scope['_rc'](n, NS(channels=[1500]*5+[pwm]))
                self.assertFalse(n.pilot_session.enabled)
                self.assertFalse(scope['_session_rc'](n, 1.5).authorized)
                # Only a complete new low sequence may authorize.
                for t, pwm in ((1.6,1000),(1.85,1000),(2.1,1000),(2.2,2000)):
                    self.t = t
                    scope['_rc'](n, NS(channels=[1500]*5+[pwm]))
                self.assertTrue(n.pilot_session.enabled)

    def test_mode_dispatch_drops_cancelled_request_and_accepts_owned_rollback(self):
        self.active()
        n = self.n
        sent = []
        n.mode_client = NS(service_is_ready=lambda: True,
            call_async=lambda request: (sent.append(request) or
                NS(add_done_callback=lambda callback: None)))
        self.state('ALT_HOLD')
        scope['_dispatch_mode_request'](n, self.request)
        self.assertEqual(sent, [])
        self.setUp()
        self.active()
        self.n.mode_client = n.mode_client
        self.tick(2, False)
        rollback = self.n.manager.update(now_s=2, current_mode='GUIDED', desired_mode=None)
        scope['_dispatch_mode_request'](self.n, rollback)
        self.assertEqual([r.custom_mode for r in sent], ['LOITER'])

    def test_mode_dispatch_rechecks_revoked_session_before_target_send(self):
        self.authorize()
        n = self.n
        request = n.manager.update(now_s=self.t, current_mode='LOITER', desired_mode='GUIDED')
        self.tick(1.7, False)
        sent = []
        n.mode_client = NS(service_is_ready=lambda: True,
            call_async=lambda msg: (sent.append(msg) or NS(add_done_callback=lambda cb: None)))
        scope['_dispatch_mode_request'](n, request)
        self.assertEqual(sent, [])


if __name__ == '__main__':
    unittest.main()
