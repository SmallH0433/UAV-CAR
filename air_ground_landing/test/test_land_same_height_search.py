"""Behavior and simulated motion checks for landing recovery, without outputs."""
import math
import unittest
from air_ground_landing.action_execution import ActionExecutor, ActionKind, ActionRequest
from test_guided_land_action import flight


class SameHeightSearchTests(unittest.TestCase):
    def start(self, **changes):
        settings = dict(land_yaw_alignment_dwell_s=0, land_recovery_timeout_s=60,
                        land_reacquire_dwell_s=.5, land_search_radius_m=.5,
                        land_search_speed_mps=.05)
        settings.update(changes)
        e = ActionExecutor(**settings)
        e.start(ActionRequest('search', ActionKind.LAND, 180, {'rc_managed': True}), flight())
        e.tick(flight(.2))
        return e

    def lost(self, t, **changes):
        return flight(t, candidate_fresh=False, **changes)

    def test_heading_drift_stops_descent_and_restarts_yaw_alignment(self):
        e = self.start()
        for t in (.3, .5, .8):
            _, c = e.tick(flight(t, landing_heading_error_rad=1.0))
            self.assertEqual(c.velocity_enu[2], 0)
            self.assertGreater(c.yaw_rate_rad_s, 0)

    def test_loss_brakes_before_horizontal_search(self):
        e = self.start()
        for t in (.3, .5, .7):
            _, c = e.tick(self.lost(t))
            self.assertEqual(c.velocity_enu, (0, 0, 0))
        s, c = e.tick(self.lost(.9))
        self.assertGreater(c.velocity_enu[0], 0)
        self.assertEqual(c.velocity_enu[2], 0)
        self.assertEqual(c.yaw_rate_rad_s, 0)
        self.assertEqual(s.detail, 'TAG_LOST_SAME_HEIGHT_SEARCH')

    def test_simulated_search_stays_bounded_and_visits_four_directions(self):
        e = self.start(land_recovery_timeout_s=90)
        x, y, z = 2.0, -3.0, 1.0
        e.tick(self.lost(.3, position_enu=(x, y, z)))
        points = []
        for step in range(1, 1800):
            t = .3 + step * .05
            s, c = e.tick(self.lost(t, position_enu=(x, y, z)))
            self.assertEqual(c.desired_mode, 'GUIDED')
            self.assertEqual(c.velocity_enu[2], 0)
            self.assertEqual(c.yaw_rate_rad_s, 0)
            self.assertLessEqual(math.hypot(*c.velocity_enu[:2]), .0500001)
            self.assertFalse(c.request_disarm)
            x += c.velocity_enu[0] * .05
            y += c.velocity_enu[1] * .05
            points.append((x - 2, y + 3))
            self.assertLess(math.hypot(x - 2, y + 3), .5)
            if s.detail == 'SEARCH_COMPLETE_GUIDED_HOLD':
                break
        else:
            self.fail('Bounded search did not finish')
        self.assertGreater(max(p[0] for p in points), .35)
        self.assertLess(min(p[0] for p in points), -.35)
        self.assertGreater(max(p[1] for p in points), .35)
        self.assertLess(min(p[1] for p in points), -.35)
        self.assertTrue(e.active)

    def test_boundary_latches_hold_even_if_target_returns(self):
        e = self.start(); e.tick(self.lost(.3))
        for t in (.9, 1.0):
            s, c = e.tick(self.lost(t, position_enu=(.51, 0, 1)))
            self.assertEqual(c.velocity_enu, (0, 0, 0))
            self.assertEqual(s.detail, 'SEARCH_BOUNDARY_GUIDED_HOLD')
        _, c = e.tick(flight(1.2))
        self.assertEqual(c.velocity_enu, (0, 0, 0))

    def test_timeout_keeps_land_session_and_never_requests_climb_or_loiter(self):
        e = self.start(land_recovery_timeout_s=1); e.tick(self.lost(.3))
        for t in (1.3, 3.0, 8.0):
            s, c = e.tick(flight(t))
            self.assertEqual(c.desired_mode, 'GUIDED')
            self.assertEqual(c.velocity_enu, (0, 0, 0))
            self.assertEqual(s.detail, 'TAG_REACQUIRE_TIMEOUT_GUIDED_HOLD')
            self.assertTrue(e.active)

    def test_reacquisition_waits_for_stability_then_resumes_slow_descent(self):
        e = self.start(); e.tick(self.lost(.3)); e.tick(self.lost(.9))
        for t in (1.0, 1.2, 1.4):
            _, c = e.tick(flight(t))
            self.assertEqual(c.velocity_enu, (0, 0, 0))
        _, c = e.tick(flight(1.6, landing_heading_error_rad=.02))
        self.assertLess(c.velocity_enu[2], 0)
        self.assertEqual(c.yaw_rate_rad_s, 0)

    def test_brief_poll_gap_does_not_restart_reacquisition_dwell(self):
        e = self.start(); e.tick(self.lost(.3))
        e.tick(flight(1.0)); e.tick(flight(1.2))
        _, c = e.tick(self.lost(1.3))
        self.assertEqual(c.velocity_enu, (0, 0, 0))
        e.tick(flight(1.4)); _, c = e.tick(flight(1.6))
        self.assertLess(c.velocity_enu[2], 0)

    def test_long_gap_requires_new_stable_evidence(self):
        e = self.start(); e.tick(self.lost(.3))
        e.tick(flight(1.0)); e.tick(self.lost(1.4))
        for t in (1.5, 1.7, 1.9):
            _, c = e.tick(flight(t))
            self.assertEqual(c.velocity_enu, (0, 0, 0))
        _, c = e.tick(flight(2.1))
        self.assertLess(c.velocity_enu[2], 0)

    def test_invalid_range_or_unstable_motion_stops_horizontal_search(self):
        for change in ({'range_fresh': False}, {'range_m': -1}, {'range_m': 9},
                       {'tilt_deg': 12}, {'velocity_enu': (.2, 0, 0)}):
            with self.subTest(change=change):
                e = self.start(); e.tick(self.lost(.3))
                _, c = e.tick(self.lost(.9, **change))
                self.assertEqual(c.velocity_enu, (0, 0, 0))

    def test_rc_release_and_authorization_always_override_search(self):
        for change in ({'landing_explicit_low': True, 'landing_requested': False},
                       {'authorized': False}):
            e = self.start(); e.tick(self.lost(.3)); e.tick(self.lost(.9))
            _, c = e.tick(self.lost(1.0, **change))
            self.assertEqual(c.desired_mode, 'LOITER')
            self.assertEqual(c.velocity_enu, (0, 0, 0))

    def test_ekf_failure_brakes_search_before_releasing_control(self):
        e = self.start(); e.tick(self.lost(.3)); e.tick(self.lost(.9))
        _, c = e.tick(self.lost(1.0, ekf_healthy=False))
        self.assertEqual(c.velocity_enu, (0, 0, 0))
        self.assertFalse(e.active)
        self.assertFalse(e.retaining_control)

    def test_stale_telemetry_brakes_search_during_grace(self):
        e = self.start(); e.tick(self.lost(.3)); e.tick(self.lost(.9))
        _, c = e.tick(self.lost(1.0, telemetry_fresh=False))
        self.assertEqual(c.velocity_enu, (0, 0, 0))
        self.assertTrue(e.active)
        _, c = e.tick(self.lost(2.1, telemetry_fresh=False))
        self.assertEqual(c.velocity_enu, (0, 0, 0))
        self.assertFalse(e.active)

    def test_native_land_commit_wins_over_search_and_stays_latched(self):
        e = self.start(); e.tick(self.lost(.3))
        _, c = e.tick(self.lost(.9, range_m=.10))
        self.assertEqual(c.desired_mode, 'LAND')
        _, c = e.tick(flight(1.0, mode='LAND', range_m=.15))
        self.assertEqual(c.desired_mode, 'LAND')
        self.assertIsNone(c.velocity_enu)

    def test_new_task_uses_new_loss_position(self):
        e = self.start(); e.tick(self.lost(.3)); e.cancel('search', .5)
        e.start(ActionRequest('new', ActionKind.LAND, 180, {'rc_managed': True}), flight(1))
        e.tick(self.lost(1.2, position_enu=(4, 5, 1)))
        self.assertEqual(e.land_search_center_xy, (4, 5))


if __name__ == '__main__':
    unittest.main()
