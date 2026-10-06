"""Real receiver/controller output checks for continuity after yaw alignment.

The receiver harness executes callback and snapshot methods extracted from the
production adapter. Tests never replace its health, caching or loss policy.
"""
from dataclasses import replace
import json
import math
from types import SimpleNamespace
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor, ActionKind, ActionRequest, ActionState,
)
from test_land_yaw_continuity import (
    DUPLICATE, TIMEOUT, ReceiverHarness, observed, start,
)


STALE = "BRIDGE_REJECTED:STALE_FRAME"
REAL_LOSS = "BRIDGE_REJECTED:TARGET_NOT_FOUND"
TRANSIENT_REASONS = (DUPLICATE, TIMEOUT, STALE, "OBSERVATION_EXPIRED")


def descending(kind="LAND", **settings):
    """Reach descent using actual fresh yaw evidence and the normal dwell."""
    settings.setdefault("land_recovery_timeout_s", 60)
    executor = start(kind, **settings)
    command = None
    for t in (.1, .2, .3, .4, .5, .61, .81):
        _, command = executor.tick(observed(t))
    if command is None or command.velocity_enu is None:
        raise AssertionError("Landing did not produce a descent command")
    if not math.isclose(command.velocity_enu[2], -.05, abs_tol=1e-9):
        raise AssertionError("Landing did not reach the configured 0.05 m/s descent")
    return executor


def descending_receiver(kind="LAND", **settings):
    node = ReceiverHarness(kind)
    # These are receiver constructor fields, absent from the older harness.
    node.target_status_reason = ""
    node.target_loss_sequence = 0
    node.accept(10, heading_deg=2)
    settings.setdefault("land_recovery_timeout_s", 60)
    settings.setdefault("land_reacquire_dwell_s", .5)
    node.lifecycle = ActionExecutor(**settings)
    node.lifecycle.start(ActionRequest("descent-receiver", ActionKind(kind), 180,
        {"rc_managed": True}), node.snapshot())
    node.tick()
    for t in (10.1, 10.2, 10.3, 10.4, 10.5, 10.61, 10.81):
        node.accept(t, heading_deg=2)
        _, command = node.tick()
    if command is None or command.velocity_enu is None:
        raise AssertionError("Receiver/controller pipeline did not command descent")
    if not math.isclose(command.velocity_enu[2], -.05, abs_tol=1e-9):
        raise AssertionError("Receiver/controller pipeline did not settle at 0.05 m/s")
    return node


def unavailable(now, reason, loss_sequence=0, **changes):
    return observed(now, candidate_fresh=False, landing_alignment_fresh=False,
        landing_observation_failure_reason=reason,
        landing_observation_loss_sequence=loss_sequence, **changes)


class OutputAssertions(unittest.TestCase):
    def assert_hold(self, command):
        self.assertIsNotNone(command)
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assertFalse(command.request_disarm)

    def assert_descent(self, command, *, settled=False):
        self.assertIsNotNone(command)
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertIsNotNone(command.velocity_enu)
        self.assertLess(command.velocity_enu[2], 0)
        self.assertGreaterEqual(command.velocity_enu[2], -.050000001)
        if settled:
            self.assertAlmostEqual(command.velocity_enu[2], -.05)
        self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assertFalse(command.request_disarm)

    def assert_no_descent(self, command):
        if command is not None and command.velocity_enu is not None:
            self.assertGreaterEqual(command.velocity_enu[2], 0)
            self.assertIn(command.yaw_rate_rad_s, (None, 0))


class DescentReceiverContinuityTests(OutputAssertions):
    def test_duplicate_and_timeout_continue_descent_on_original_fresh_evidence(self):
        for reason in (DUPLICATE, TIMEOUT):
            with self.subTest(reason=reason):
                node = descending_receiver()
                before = node.snapshot()
                alignment = node.landing_alignment
                candidate_time = node.candidate_received_s
                status_time = node.target_status_received_s
                for t in (10.9, 11.0, 11.07):
                    node.reject(reason, t)
                    snap = node.snapshot()
                    self.assertTrue(snap.candidate_fresh)
                    self.assertTrue(snap.landing_alignment_fresh)
                    self.assertEqual(snap.landing_observation_s,
                                     before.landing_observation_s)
                    self.assertEqual(snap.landing_observation_loss_sequence,
                                     before.landing_observation_loss_sequence)
                    self.assertIs(node.landing_alignment, alignment)
                    self.assertEqual(node.candidate_received_s, candidate_time)
                    self.assertEqual(node.target_status_received_s, status_time)
                    _, command = node.lifecycle.tick(snap)
                    self.assert_descent(command, settled=True)
                # Source time is 10.79, so 11.10 has already expired.
                node.reject(reason, 11.10)
                snap = node.snapshot()
                self.assertFalse(snap.landing_alignment_fresh)
                _, command = node.lifecycle.tick(snap)
                self.assert_hold(command)

    def test_source_age_expires_cached_descent_before_receipt_age_limit(self):
        for reason in (DUPLICATE, TIMEOUT):
            node = descending_receiver()
            node.accept(10.9, heading_deg=2, source_age=.25)
            _, command = node.tick()
            self.assert_descent(command, settled=True)
            node.reject(reason, 10.94)
            _, command = node.tick()
            self.assert_descent(command, settled=True)
            node.reject(reason, 10.96)
            self.assert_hold(node.tick()[1])

    def test_short_age_expiry_then_new_frame_resumes_descent_immediately(self):
        node = descending_receiver()
        node.now = 11.10  # No report arrives; snapshot must notice source expiry.
        snap = node.snapshot()
        self.assertEqual(snap.landing_observation_failure_reason,
                         "OBSERVATION_EXPIRED")
        self.assert_hold(node.lifecycle.tick(snap)[1])
        node.accept(11.20, heading_deg=-2)
        self.assert_descent(node.tick()[1])
        node.accept(11.40, heading_deg=2)
        self.assert_descent(node.tick()[1], settled=True)

    def test_expired_duplicate_and_timeout_then_new_frame_resume_immediately(self):
        for reason in (DUPLICATE, TIMEOUT, STALE):
            with self.subTest(reason=reason):
                node = descending_receiver()
                node.reject(reason, 11.10)
                self.assert_hold(node.tick()[1])
                node.accept(11.20, heading_deg=2)
                self.assert_descent(node.tick()[1])

    def test_real_loss_remains_visible_through_following_transient_reports(self):
        for reason in (DUPLICATE, TIMEOUT, STALE):
            with self.subTest(reason=reason):
                node = descending_receiver()
                original_sequence = node.snapshot().landing_observation_loss_sequence
                node.reject(REAL_LOSS, 10.90)
                self.assert_hold(node.tick()[1])
                self.assertGreater(node.snapshot().landing_observation_loss_sequence,
                                   original_sequence)
                node.reject(reason, 10.95)
                self.assert_hold(node.tick()[1])
                for t in (11.0, 11.2, 11.4):
                    node.accept(t, heading_deg=2)
                    self.assert_hold(node.tick()[1])
                node.accept(11.51, heading_deg=2)
                self.assert_descent(node.tick()[1])

    def test_bad_and_good_status_in_same_control_tick_cannot_hide_actual_loss(self):
        for bad in (REAL_LOSS, "LOW_QUALITY", "APRILTAG_CORNERS_OUTSIDE_IMAGE",
                    "VISION_STATUS_UNAVAILABLE:ConnectionRefusedError"):
            with self.subTest(bad=bad):
                node = descending_receiver()
                node.reject(bad, 10.90)
                node.accept(10.90, heading_deg=2)
                self.assertTrue(node.snapshot().landing_alignment_fresh)
                self.assert_hold(node.tick()[1])
                for t in (11.0, 11.2, 11.4):
                    node.accept(t, heading_deg=2)
                    self.assert_hold(node.tick()[1])
                node.accept(11.51, heading_deg=2)
                self.assert_descent(node.tick()[1])

    def test_malformed_and_invalid_healthy_pose_cannot_be_hidden_by_good_status(self):
        invalid_reports = ("{", "null", "[]", json.dumps({
            "healthy": True, "aligned": True,
            "landing_alignment": {"frame": "BODY_FLU", "velocity_flu": [.1, 0],
                                  "heading_error_rad": None}}))
        for bad in invalid_reports:
            with self.subTest(report=bad):
                node = descending_receiver()
                old_sequence = node.snapshot().landing_observation_loss_sequence
                node.now = 10.90
                node._target_status(SimpleNamespace(data=bad))
                node.accept(10.90, heading_deg=2)
                snap = node.snapshot()
                self.assertTrue(snap.landing_alignment_fresh)
                self.assertGreater(snap.landing_observation_loss_sequence,
                                   old_sequence)
                self.assert_hold(node.lifecycle.tick(snap)[1])

    def test_non_string_failure_reason_clears_evidence_and_records_real_loss(self):
        for reason in ([], {}):
            with self.subTest(reason=reason):
                node = descending_receiver()
                old_sequence = node.snapshot().landing_observation_loss_sequence
                node.now = 10.90
                node._target_status(SimpleNamespace(data=json.dumps(
                    {"healthy": False, "reason": reason})))
                snap = node.snapshot()
                self.assertFalse(snap.candidate_fresh)
                self.assertFalse(snap.landing_alignment_fresh)
                self.assertGreater(snap.landing_observation_loss_sequence,
                                   old_sequence)
                # A good callback before control executes still needs a hold.
                node.accept(10.90, heading_deg=2)
                self.assert_hold(node.tick()[1])

    def test_new_real_loss_during_verification_restarts_stability_dwell(self):
        node = descending_receiver()
        node.reject(REAL_LOSS, 10.90)
        self.assert_hold(node.tick()[1])
        for t in (11.0, 11.2):
            node.accept(t, heading_deg=2)
            self.assert_hold(node.tick()[1])
        # Both callbacks happen before the next controller tick.
        node.reject("LOW_QUALITY", 11.3)
        node.accept(11.3, heading_deg=2)
        self.assert_hold(node.tick()[1])
        for t in (11.4, 11.6, 11.79):
            node.accept(t, heading_deg=2)
            self.assert_hold(node.tick()[1])
        node.accept(11.81, heading_deg=2)
        self.assert_descent(node.tick()[1])

    def test_transient_reports_never_increment_actual_loss_sequence(self):
        node = descending_receiver()
        original = node.snapshot().landing_observation_loss_sequence
        for t, reason in ((10.9, DUPLICATE), (11.0, TIMEOUT), (11.1, STALE),
                          (11.2, DUPLICATE), (11.3, TIMEOUT)):
            node.reject(reason, t)
            self.assertEqual(node.snapshot().landing_observation_loss_sequence,
                             original)

    def test_descent_cache_is_not_used_for_follow_native_land_or_exit(self):
        for scenario in ("FOLLOW", "NATIVE_LAND", "EXIT_GUIDED", "INACTIVE"):
            for reason in (DUPLICATE, TIMEOUT):
                with self.subTest(scenario=scenario, reason=reason):
                    node = descending_receiver()
                    if scenario == "FOLLOW":
                        node.lifecycle = start("FOLLOW")
                        node.lifecycle.land_phase = "GUIDED_TRACK_DESCENT"
                        node.lifecycle.land_yaw_alignment_complete = True
                    elif scenario == "NATIVE_LAND":
                        node.lifecycle.land_phase = "HYBRID_LAND_COMMITTED"
                    elif scenario == "EXIT_GUIDED":
                        node.lifecycle.land_phase = "EXIT_GUIDED"
                    else:
                        node.lifecycle.cancel(None, 10.82)
                    node.reject(reason, 10.90)
                    self.assertFalse(node.snapshot().candidate_fresh)
                    self.assertFalse(node.snapshot().landing_alignment_fresh)


class DescentCoreContinuityTests(OutputAssertions):
    def test_short_transient_hold_resumes_only_on_distinct_fresh_observation(self):
        for reason in TRANSIENT_REASONS:
            with self.subTest(reason=reason):
                executor = descending()
                self.assert_hold(executor.tick(unavailable(.9, reason))[1])
                # The prior source is still mathematically fresh here, but it
                # cannot prove that observation resumed after the hold.
                self.assert_hold(executor.tick(observed(1.0, observation=.81))[1])
                _, command = executor.tick(observed(1.1, observation=1.09,
                    landing_heading_error_rad=math.radians(2)))
                self.assert_descent(command)

    def test_long_transient_outage_requires_dwell_even_without_search_tick(self):
        for reason in TRANSIENT_REASONS:
            with self.subTest(reason=reason):
                executor = descending()
                self.assert_hold(executor.tick(unavailable(.9, reason))[1])
                # The next tick is already past the 0.5 s search threshold.
                for t in (1.41, 1.61, 1.81):
                    self.assert_hold(executor.tick(observed(t))[1])
                self.assert_descent(executor.tick(observed(1.92))[1])

    def test_exact_search_wait_boundary_also_requires_reacquisition_dwell(self):
        executor = descending()
        self.assert_hold(executor.tick(unavailable(1.0, TIMEOUT))[1])
        self.assert_hold(executor.tick(observed(1.5))[1])
        for t in (1.7, 1.9):
            self.assert_hold(executor.tick(observed(t))[1])
        self.assert_descent(executor.tick(observed(2.01))[1])

    def test_true_loss_brakes_then_searches_at_same_height_and_zero_yaw(self):
        executor = descending()
        for t in (.9, 1.1, 1.3):
            self.assert_hold(executor.tick(unavailable(t, REAL_LOSS, 1))[1])
        status, command = executor.tick(unavailable(1.5, REAL_LOSS, 1))
        self.assertEqual(status.detail, "TAG_LOST_SAME_HEIGHT_SEARCH")
        self.assertGreater(math.hypot(*command.velocity_enu[:2]), 0)
        self.assertLessEqual(math.hypot(*command.velocity_enu[:2]), .050000001)
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assertFalse(command.request_disarm)
        for t in (1.6, 1.8, 2.0):
            self.assert_hold(executor.tick(observed(t,
                landing_observation_loss_sequence=1))[1])
        self.assert_descent(executor.tick(observed(2.11,
            landing_observation_loss_sequence=1))[1])

    def test_search_boundary_stop_cannot_be_revived_by_new_fresh_frame(self):
        executor = descending()
        executor.tick(unavailable(.9, TIMEOUT))
        status, command = executor.tick(unavailable(1.5, TIMEOUT,
            position_enu=(.51, 0, 1)))
        self.assertEqual(status.detail, "SEARCH_BOUNDARY_GUIDED_HOLD")
        self.assert_hold(command)
        for t in (1.6, 1.8, 2.0, 2.2):
            status, command = executor.tick(observed(t))
            self.assertEqual(status.detail, "SEARCH_BOUNDARY_GUIDED_HOLD")
            self.assert_hold(command)

    def test_recovery_deadline_wins_over_fresh_return_and_stays_stopped(self):
        executor = descending(land_recovery_timeout_s=1)
        executor.tick(unavailable(1.0, TIMEOUT))
        for t in (2.0, 2.2, 2.4, 2.6):
            status, command = executor.tick(observed(t))
            self.assertEqual(status.detail, "TAG_REACQUIRE_TIMEOUT_GUIDED_HOLD")
            self.assert_hold(command)
            self.assertTrue(executor.active)

    def test_action_timeout_wins_over_fast_transient_return(self):
        executor = descending()
        executor.request = replace(executor.request, timeout_s=1)
        self.assert_hold(executor.tick(unavailable(.9, TIMEOUT))[1])
        status, command = executor.tick(observed(1.0))
        self.assertEqual(status.state, ActionState.TIMEOUT)
        self.assert_hold(command)

    def test_fast_recovery_still_obeys_range_mode_telemetry_and_ekf_guards(self):
        for changes in ({"range_fresh": False}, {"range_m": -1},
                        {"range_m": math.nan}, {"mode": "LOITER"},
                        {"telemetry_fresh": False}, {"ekf_healthy": False}):
            with self.subTest(changes=changes):
                executor = descending()
                self.assert_hold(executor.tick(unavailable(.9, TIMEOUT))[1])
                status, command = executor.tick(observed(1.0, **changes))
                self.assert_no_descent(command)
                if changes.get("ekf_healthy") is False:
                    self.assertFalse(executor.active)
                    self.assertEqual(status.reason, "EKF_BECAME_UNHEALTHY")
                elif changes.get("mode") != "LOITER":
                    self.assert_hold(command)

    def test_near_ground_handoff_wins_over_fast_recovery_and_stopped_hold(self):
        for stopped in (False, True):
            with self.subTest(stopped=stopped):
                executor = descending(land_recovery_timeout_s=1)
                executor.tick(unavailable(1.0, TIMEOUT))
                now = 1.1
                if stopped:
                    self.assert_hold(executor.tick(observed(2.0))[1])
                    now = 2.1
                _, command = executor.tick(observed(now, range_m=.10,
                    telemetry_fresh=False, ekf_healthy=False))
                self.assertEqual(command.desired_mode, "LAND")
                self.assertIsNone(command.velocity_enu)
                self.assertFalse(command.request_disarm)
                _, command = executor.tick(observed(now + .2,
                    mode="LAND", range_m=.8))
                self.assertEqual(command.desired_mode, "LAND")
                self.assertIsNone(command.velocity_enu)

    def test_rc_release_and_authorization_win_over_fast_return(self):
        for changes, expected_mode in (({"landing_requested": False,
                    "landing_explicit_low": True}, "GUIDED"),
                    ({"authorized": False}, "LOITER")):
            with self.subTest(changes=changes):
                executor = descending()
                executor.tick(unavailable(.9, TIMEOUT))
                _, command = executor.tick(observed(1.0, **changes))
                self.assertEqual(command.desired_mode, expected_mode)
                self.assertEqual(command.velocity_enu, (0, 0, 0))
                self.assertFalse(command.request_disarm)

    def test_new_session_baselines_old_loss_sequence_and_repeats_yaw_gate(self):
        executor = descending()
        executor.tick(unavailable(.9, REAL_LOSS, 4,
            position_enu=(4, 5, 1)))
        executor.tick(observed(1.0, landing_observation_loss_sequence=4))
        executor.tick(observed(1.2, landing_observation_loss_sequence=4))
        executor.cancel("continuity", 1.3)
        new_snapshot = observed(2.0, landing_observation_loss_sequence=4,
            landing_heading_error_rad=math.radians(40))
        status = executor.start(ActionRequest("new-descent", ActionKind.LAND,
            180, {"rc_managed": True}), new_snapshot)
        self.assertEqual(status.state, ActionState.RUNNING)
        _, command = executor.tick(observed(2.1,
            landing_observation_loss_sequence=4,
            landing_heading_error_rad=math.radians(-40)))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertLess(command.yaw_rate_rad_s, 0)
        for t in (2.2, 2.4, 2.6):
            _, command = executor.tick(observed(t,
                landing_observation_loss_sequence=4))
            self.assertEqual(command.velocity_enu[2], 0)
        self.assert_descent(executor.tick(observed(2.71,
            landing_observation_loss_sequence=4))[1])
        # Its own subsequent loss must still be noticed immediately.
        self.assert_hold(executor.tick(observed(2.8,
            landing_observation_loss_sequence=5))[1])


if __name__ == "__main__":
    unittest.main()
