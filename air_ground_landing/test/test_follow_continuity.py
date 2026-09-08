import unittest

from air_ground_landing.guided_execution import (
    FollowContinuityConfig,
    FollowContinuityGuard,
    FollowContinuityState,
    LandingSwitchConfig,
    LandingSwitchState,
    RcLandingRequestGate,
)


class FollowContinuityTests(unittest.TestCase):
    def setUp(self):
        self.guard = FollowContinuityGuard(
            FollowContinuityConfig(dropout_grace_s=1.0, reacquire_dwell_s=0.0)
        )

    def update(self, now_s, *, fresh, authorized=True, low=False, connected=True):
        return self.guard.update(
            now_s=now_s,
            connected=connected,
            rc_authorized=authorized,
            rc_explicit_low=low,
            fresh_control_signal=fresh,
        )

    def test_short_dropout_keeps_guided_with_zero_velocity_hold(self):
        self.assertEqual(self.update(0.0, fresh=True).state, FollowContinuityState.ACTIVE)
        grace = self.update(0.6, fresh=False)
        self.assertEqual(grace.state, FollowContinuityState.GRACE_ZERO_HOLD)
        self.assertTrue(grace.keep_guided)
        self.assertTrue(grace.zero_velocity_hold)
        self.assertFalse(grace.reacquire_pending)
        self.assertEqual(self.update(0.8, fresh=True).state, FollowContinuityState.ACTIVE)

    def test_long_dropout_reacquires_on_first_fresh_candidate(self):
        self.update(0.0, fresh=True)
        timed_out = self.update(1.01, fresh=False)
        self.assertEqual(timed_out.state, FollowContinuityState.REACQUIRE_WAIT)
        self.assertFalse(timed_out.keep_guided)
        self.assertEqual(
            self.update(1.1, fresh=True).state,
            FollowContinuityState.ACTIVE,
        )
        self.assertEqual(
            self.update(1.3, fresh=False).state,
            FollowContinuityState.GRACE_ZERO_HOLD,
        )
        self.assertEqual(
            self.update(1.4, fresh=True).state,
            FollowContinuityState.ACTIVE,
        )

    def test_grace_does_not_clear_swd_low_edge(self):
        self.update(0.0, fresh=True)
        landing = RcLandingRequestGate(LandingSwitchConfig(channel=8))
        channels = [1500] * 7 + [1100]
        self.assertEqual(
            landing.evaluate(
                channels,
                received_time_s=0.0,
                now_s=0.0,
                follow_active=True,
            ).state,
            LandingSwitchState.READY,
        )
        # Real RC continues while only the visual candidate is in grace.
        landing.evaluate(channels, received_time_s=0.3, now_s=0.3, follow_active=True)
        grace = self.update(0.6, fresh=False)
        channels[-1] = 1900
        self.assertEqual(
            landing.evaluate(
                channels,
                received_time_s=0.6,
                now_s=0.6,
                follow_active=grace.session_available,
            ).state,
            LandingSwitchState.REQUESTED,
        )

    def test_swd_high_requests_after_follow_becomes_active(self):
        landing = RcLandingRequestGate(LandingSwitchConfig(channel=8))
        channels = [1500] * 7 + [1900]
        self.assertEqual(
            landing.evaluate(
                channels,
                received_time_s=0.0,
                now_s=0.0,
                follow_active=False,
            ).state,
            LandingSwitchState.FOLLOW_INACTIVE,
        )
        requested = landing.evaluate(
            channels,
            received_time_s=0.1,
            now_s=0.1,
            follow_active=True,
        )
        self.assertEqual(requested.state, LandingSwitchState.REQUESTED)
        self.assertTrue(requested.requested)

    def test_disconnect_reacquires_on_first_fresh_candidate(self):
        self.update(0.0, fresh=True)
        self.assertEqual(
            self.update(0.1, fresh=False, connected=False).state,
            FollowContinuityState.REACQUIRE_WAIT,
        )
        self.assertEqual(
            self.update(0.2, fresh=True).state,
            FollowContinuityState.ACTIVE,
        )


if __name__ == "__main__":
    unittest.main()
