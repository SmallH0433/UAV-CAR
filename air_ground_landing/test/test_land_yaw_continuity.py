"""Real receiver and controller checks for yaw-first observation continuity.

The ROS methods are extracted from the deployed adapter source, then executed
without ROS or aircraft outputs. No copied receiver implementation is used.
"""
import ast
import copy
import json
import math
from types import MethodType, SimpleNamespace
from typing import Optional
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor, ActionKind, ActionRequest, VehicleSnapshot,
)
from air_ground_landing.landing_alignment import LandingAlignment
from test_guided_land_action import executor_source, flight


DUPLICATE = "BRIDGE_REJECTED:DUPLICATE_FRAME"
TIMEOUT = "VISION_STATUS_UNAVAILABLE:TimeoutError"


def start(kind="LAND", **settings):
    # Match the active hardware recovery policy, whose dwell is 0.5 s.
    settings.setdefault("land_reacquire_dwell_s", .5)
    executor = ActionExecutor(**settings)
    executor.start(ActionRequest("continuity", ActionKind(kind), 180,
                                 {"rc_managed": True}), flight())
    return executor


def observed(now, observation=None, **changes):
    changes.setdefault("landing_heading_error_rad", math.radians(2))
    return flight(now, landing_observation_s=(now if observation is None
                                               else observation), **changes)


def receiver_methods():
    tree = ast.parse(executor_source().read_text(encoding="utf-8"))
    names = {"_target_status", "_snapshot", "_candidate", "_fresh"}
    methods = [copy.deepcopy(method)
               for cls in tree.body if isinstance(cls, ast.ClassDef)
               for method in cls.body
               if isinstance(method, ast.FunctionDef) and method.name in names]
    for method in methods:
        method.decorator_list = []
    scope = dict(json=json, math=math, String=object, PositionTarget=object,
                 LandingAlignment=LandingAlignment, ActionKind=ActionKind,
                 Optional=Optional, VehicleSnapshot=VehicleSnapshot,
                 ExtendedState=SimpleNamespace(LANDED_STATE_ON_GROUND=1))
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods,
             type_ignores=[])), "production_receiver", "exec"), scope)
    return {name: scope[name] for name in names}


class ReceiverHarness:
    """Provide message fields to the actual ROS callback/snapshot methods."""
    def __init__(self, kind="LAND"):
        self.now = 10.0
        self.lifecycle = start(kind)
        self.lifecycle.land_phase = "GUIDED_ALIGN_YAW"
        self._now_s = lambda: self.now
        methods = receiver_methods()
        self._fresh = methods["_fresh"]
        for name in ("_candidate", "_target_status", "_snapshot"):
            setattr(self, name, MethodType(methods[name], self))
        self.candidate_maximum_age_s = .3
        self.landing_alignment_maximum_age_s = .3
        self.state_maximum_age_s = .3
        self.pose_maximum_age_s = .3
        self.velocity_maximum_age_s = .3
        self.ekf_maximum_age_s = 5
        self.landing_target_maximum_age_s = .5
        self.range_maximum_age_s = .5
        self.candidate = None
        self.candidate_received_s = None
        self.target_status_received_s = None
        self.target_healthy = False
        self.target_aligned = False
        self.landing_alignment = None
        self._landing_switch = lambda now: (True, False)
        self._rc_authorized = lambda now: True
        self.home_set = True
        self.estimator_healthy = True
        self.landing_target_stream_healthy = True
        self.landing_target_output_enabled = True
        self.range_m = .8
        self.vehicle_state = SimpleNamespace(connected=True, armed=True,
                                              mode="GUIDED")
        self.extended = SimpleNamespace(landed_state=2)
        self.pose = SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0., z=1.),
            orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        self.velocity = SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0., y=0., z=0.),
            angular=SimpleNamespace(z=0.)))
        self.refresh_telemetry()

    def refresh_telemetry(self):
        for name in ("state_received_s", "pose_received_s", "velocity_received_s",
                     "estimator_received_s", "extended_received_s",
                     "landing_target_status_received_s", "range_received_s"):
            setattr(self, name, self.now)

    def accept(self, now=None, heading_deg=20, source_age=.02):
        if now is not None:
            self.now = now
        self.refresh_telemetry()
        self._candidate(SimpleNamespace(velocity=SimpleNamespace(x=.1, y=0)))
        self._target_status(SimpleNamespace(data=json.dumps({
            "healthy": True, "aligned": True,
            "landing_alignment": {"frame": "BODY_FLU", "velocity_flu": [.1, 0],
                "heading_error_rad": math.radians(heading_deg),
                "center_error_px": 10, "source_age_s": source_age}})))

    def reject(self, reason, now=None):
        if now is not None:
            self.now = now
        self.refresh_telemetry()
        self._target_status(SimpleNamespace(data=json.dumps(
            {"healthy": False, "reason": reason})))

    def snapshot(self):
        self.refresh_telemetry()
        return self._snapshot(self.now)

    def tick(self):
        return self.lifecycle.tick(self.snapshot())


class YawEvidenceContinuityTests(unittest.TestCase):
    def test_fresh_alignment_still_requires_half_second_before_descent(self):
        for kind in ("LAND", "PRECISION_LAND"):
            with self.subTest(kind=kind):
                executor = start(kind)
                for t in (.1, .2, .3, .4, .5):
                    _, command = executor.tick(observed(t))
                    self.assertEqual(command.velocity_enu[2], 0)
                    self.assertEqual(command.yaw_rate_rad_s, 0)
                _, command = executor.tick(observed(.61))
                self.assertLess(command.velocity_enu[2], 0)
                self.assertTrue(executor.land_yaw_alignment_complete)

    def test_same_observation_never_advances_stable_window(self):
        executor = start()
        for t in (.1, .2, .3, .39):
            _, command = executor.tick(observed(t, observation=.1))
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertFalse(executor.land_yaw_alignment_complete)
        # Even a tick far beyond dwell cannot turn a repeated frame into proof.
        _, command = executor.tick(observed(.7, observation=.1,
                                            landing_alignment_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertFalse(executor.land_yaw_alignment_complete)

    def test_cached_ticks_do_not_hide_gap_between_new_observations(self):
        executor = start()
        for t in (.1, .2, .3, .39):
            executor.tick(observed(t, observation=.1))
        _, command = executor.tick(observed(.62))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertFalse(executor.land_yaw_alignment_complete)
        for t in (.72, .82, .92, 1.02):
            _, command = executor.tick(observed(t))
            self.assertEqual(command.velocity_enu[2], 0)
        _, command = executor.tick(observed(1.13))
        self.assertLess(command.velocity_enu[2], 0)

    def test_measured_rotation_resets_window_on_cached_frame(self):
        executor = start()
        for t in (.1, .3, .5):
            executor.tick(observed(t))
        _, command = executor.tick(observed(.55, observation=.5,
                                     yaw_rate_rad_s=math.radians(12)))
        self.assertEqual(command.velocity_enu[2], 0)
        # The same cached frame cannot restart proof after the gyro settles.
        _, command = executor.tick(observed(.6, observation=.5))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertIsNone(executor.land_yaw_aligned_since_s)
        for t in (.7, .9, 1.1):
            _, command = executor.tick(observed(t))
            self.assertEqual(command.velocity_enu[2], 0)
        _, command = executor.tick(observed(1.21))
        self.assertLess(command.velocity_enu[2], 0)

    def test_out_of_order_observation_cannot_complete_alignment(self):
        executor = start()
        for t in (.1, .3, .5):
            executor.tick(observed(t))
        _, command = executor.tick(observed(.62, observation=.4))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertFalse(executor.land_yaw_alignment_complete)
        _, command = executor.tick(observed(.65))
        self.assertLess(command.velocity_enu[2], 0)

    def test_source_age_changes_cannot_shorten_measured_stability_dwell(self):
        executor = start()
        # All frames are fresh and distinct, but capture ages fall rapidly.
        # Their span is 0.50 s; the live gyro was observed stable for only .22 s.
        for receipt, capture in ((.30, .01), (.31, .25), (.42, .40),
                                 (.52, .51), (.62, .61), (.72, .71)):
            _, command = executor.tick(observed(receipt, observation=capture))
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertFalse(executor.land_yaw_alignment_complete)
        _, command = executor.tick(observed(.81, observation=.80))
        self.assertLess(command.velocity_enu[2], 0)
        self.assertTrue(executor.land_yaw_alignment_complete)

    def test_rotation_invalidates_stability_even_during_target_or_range_hold(self):
        for kind in ("LAND", "PRECISION_LAND"):
            for unavailable in ({"candidate_fresh": False}, {"range_fresh": False}):
                for rate in (math.radians(12), math.nan):
                    with self.subTest(kind=kind, unavailable=unavailable, rate=rate):
                        executor = start(kind)
                        for t in (.1, .3, .5):
                            executor.tick(observed(t))
                        _, command = executor.tick(observed(.55,
                            yaw_rate_rad_s=rate, **unavailable))
                        self.assertEqual(command.velocity_enu, (0, 0, 0))
                        self.assertIsNone(executor.land_yaw_aligned_since_s)
                        # Returning valid target/range cannot reuse the old dwell.
                        for t in (.6, .8, 1.0):
                            _, command = executor.tick(observed(t))
                            self.assertEqual(command.velocity_enu[2], 0)
                            self.assertFalse(executor.land_yaw_alignment_complete)
                        _, command = executor.tick(observed(1.11))
                        self.assertLess(command.velocity_enu[2], 0)
                        self.assertTrue(executor.land_yaw_alignment_complete)

    def test_large_error_cached_observations_keep_turning_and_hold_height(self):
        for kind in ("LAND", "PRECISION_LAND"):
            executor = start(kind)
            for t in (.1, .2, .3):
                _, command = executor.tick(observed(t, observation=.1,
                    landing_heading_error_rad=math.radians(-65)))
                self.assertEqual(command.velocity_enu[2], 0)
                self.assertAlmostEqual(command.yaw_rate_rad_s, -math.radians(15))

    def test_pre_yaw_recovery_returns_to_turning_without_extra_dwell(self):
        executor = start()
        executor.tick(observed(.1, landing_heading_error_rad=1))
        _, command = executor.tick(observed(.2, candidate_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        _, command = executor.tick(observed(.3, landing_heading_error_rad=-1))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertLess(command.yaw_rate_rad_s, 0)
        self.assertEqual(executor.land_phase, "GUIDED_ALIGN_YAW")

    def test_pre_yaw_recovery_brakes_search_before_resuming_heading(self):
        executor = start()
        executor.tick(observed(.1, landing_heading_error_rad=1))
        executor.tick(observed(.2, candidate_fresh=False))
        _, command = executor.tick(observed(.8, candidate_fresh=False))
        self.assertGreater(command.velocity_enu[0], 0)
        _, command = executor.tick(observed(.9, landing_heading_error_rad=-1,
                                           landing_velocity_flu=(0, 0)))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertLess(command.yaw_rate_rad_s, 0)
        self.assertIsNone(executor.land_search_center_xy)

    def test_after_yaw_loss_requires_heading_realignment(self):
        executor = start()
        for t in (.1, .2, .3, .4, .5, .61):
            executor.tick(observed(t))
        self.assertTrue(executor.land_yaw_alignment_complete)
        executor.tick(observed(.7, candidate_fresh=False))
        for t in (.8, 1, 1.2):
            _, command = executor.tick(observed(t, landing_heading_error_rad=1))
            self.assertEqual(command.velocity_enu, (0, 0, 0))
            self.assertEqual(command.yaw_rate_rad_s, 0)
        _, command = executor.tick(observed(1.31, landing_heading_error_rad=1))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertGreater(command.yaw_rate_rad_s, 0)
        _, command = executor.tick(observed(1.51, landing_heading_error_rad=1))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertGreater(command.yaw_rate_rad_s, 0)


class YawReceiverContinuityTests(unittest.TestCase):
    def test_whitelist_reuses_original_evidence_without_refreshing_timestamps(self):
        for kind in ("LAND", "PRECISION_LAND"):
            for reason in (DUPLICATE, TIMEOUT):
                with self.subTest(kind=kind, reason=reason):
                    node = ReceiverHarness(kind)
                    node.accept()
                    alignment = node.landing_alignment
                    candidate_time = node.candidate_received_s
                    status_time = node.target_status_received_s
                    node.reject(reason, 10.1)
                    self.assertIs(node.landing_alignment, alignment)
                    self.assertEqual(node.candidate_received_s, candidate_time)
                    self.assertEqual(node.target_status_received_s, status_time)
                    self.assertTrue(node.target_healthy)
                    snap = node.snapshot()
                    self.assertTrue(snap.candidate_fresh)
                    self.assertTrue(snap.landing_alignment_fresh)
                    self.assertAlmostEqual(snap.landing_observation_s, 9.98)

    def test_original_source_age_limits_reuse(self):
        node = ReceiverHarness()
        node.accept(source_age=.25)
        alignment = node.landing_alignment
        node.reject(DUPLICATE, 10.04)
        self.assertIs(node.landing_alignment, alignment)
        node.reject(DUPLICATE, 10.06)
        self.assertFalse(node.target_healthy)
        self.assertIsNone(node.landing_alignment)

    def test_repeated_timeout_cannot_keep_old_candidate_alive(self):
        node = ReceiverHarness()
        node.accept()
        for t in (10.1, 10.2, 10.27):
            node.reject(TIMEOUT, t)
            self.assertTrue(node.snapshot().candidate_fresh)
        node.reject(TIMEOUT, 10.31)
        self.assertFalse(node.snapshot().candidate_fresh)
        self.assertFalse(node.snapshot().landing_alignment_fresh)

    def test_actual_loss_quality_failure_and_other_errors_clear_immediately(self):
        for reason in ("BRIDGE_REJECTED:TARGET_NOT_FOUND",
                       "APRILTAG_CORNERS_OUTSIDE_IMAGE", "LOW_QUALITY",
                       "VISION_STATUS_UNAVAILABLE:ConnectionRefusedError",
                       "BRIDGE_REJECTED:DUPLICATE_FRAME_EXTRA"):
            with self.subTest(reason=reason):
                node = ReceiverHarness()
                node.accept()
                node.reject(reason, 10.01)
                self.assertFalse(node.target_healthy)
                self.assertIsNone(node.landing_alignment)
                node.reject(DUPLICATE, 10.02)
                self.assertFalse(node.target_healthy)
                self.assertIsNone(node.landing_alignment)

    def test_malformed_status_clears_and_cannot_be_resurrected(self):
        for data in ("{", "null", "[]"):
            node = ReceiverHarness()
            node.accept()
            node._target_status(SimpleNamespace(data=data))
            self.assertFalse(node.target_healthy)
            self.assertIsNone(node.landing_alignment)
            node.reject(TIMEOUT, 10.02)
            self.assertFalse(node.target_healthy)

    def test_whitelist_reason_with_malformed_health_does_not_reuse_evidence(self):
        for reason in (DUPLICATE, TIMEOUT):
            for extra in ({}, {"healthy": None}, {"healthy": "false"},
                          {"healthy": 0}):
                with self.subTest(reason=reason, extra=extra):
                    node = ReceiverHarness()
                    node.accept()
                    node.now = 10.01
                    node._target_status(SimpleNamespace(data=json.dumps(
                        {"reason": reason, **extra})))
                    self.assertFalse(node.target_healthy)
                    self.assertIsNone(node.landing_alignment)
                    node.reject(DUPLICATE, 10.02)
                    self.assertFalse(node.target_healthy)
                    self.assertIsNone(node.landing_alignment)

    def test_no_valid_prior_evidence_is_never_fabricated(self):
        for missing in ("candidate", "landing_alignment", "target_healthy",
                        "candidate_received_s", "target_status_received_s"):
            node = ReceiverHarness()
            node.accept()
            setattr(node, missing, False if missing == "target_healthy" else None)
            node.reject(DUPLICATE, 10.01)
            self.assertFalse(node.target_healthy)
            self.assertIsNone(node.landing_alignment)

    def test_follow_and_non_guided_landing_phases_do_not_relax_target_health(self):
        for scenario in ("FOLLOW", "PRECISION_COMPLETE", "INACTIVE", "NATIVE_LAND",
                         "LEGACY_NATIVE_RECOVERY", "LANDED_DISARM", "EXIT_GUIDED"):
            node = ReceiverHarness()
            node.accept()
            if scenario == "FOLLOW":
                node.lifecycle = start("FOLLOW")
                node.lifecycle.land_phase = "GUIDED_ALIGN_YAW"
            elif scenario == "PRECISION_COMPLETE":
                node.lifecycle = start("PRECISION_LAND")
                node.lifecycle.land_yaw_alignment_complete = True
            elif scenario == "INACTIVE":
                node.lifecycle.cancel(None, 10)
            elif scenario == "LEGACY_NATIVE_RECOVERY":
                node.lifecycle = ActionExecutor()
                node.lifecycle.start(ActionRequest("native", ActionKind.LAND,
                    180, {"guided_descent": False}), flight())
                node.lifecycle.land_phase = "GUIDED_REACQUIRE_HOLD"
            else:
                node.lifecycle.land_phase = ("HYBRID_LAND_COMMITTED"
                    if scenario == "NATIVE_LAND" else scenario)
            node.reject(DUPLICATE, 10.01)
            self.assertFalse(node.target_healthy)
            self.assertIsNone(node.landing_alignment)

    def test_yaw_recovery_phase_can_reuse_unexpired_observation(self):
        node = ReceiverHarness()
        node.accept()
        node.lifecycle.land_phase = "GUIDED_REACQUIRE_HOLD"
        alignment = node.landing_alignment
        node.reject(DUPLICATE, 10.1)
        self.assertIs(node.landing_alignment, alignment)
        self.assertTrue(node.target_healthy)

    def test_callback_snapshot_controller_keeps_turning_through_duplicate_poll(self):
        node = ReceiverHarness()
        node.accept(10, heading_deg=40)
        # Start the real session on the same clock as the ROS harness.
        node.lifecycle = ActionExecutor()
        node.lifecycle.start(ActionRequest("receiver", ActionKind.LAND, 180,
            {"rc_managed": True}), node.snapshot())
        _, command = node.tick()
        self.assertGreater(command.yaw_rate_rad_s, 0)
        for t, reason in ((10.1, DUPLICATE), (10.2, TIMEOUT)):
            node.reject(reason, t)
            _, command = node.tick()
            self.assertGreater(command.yaw_rate_rad_s, 0)
            self.assertEqual(command.velocity_enu[2], 0)
        node.reject(DUPLICATE, 10.31)
        _, command = node.tick()
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)

    def test_callback_snapshot_controller_requires_distinct_fresh_alignment(self):
        node = ReceiverHarness()
        node.accept(10, heading_deg=2)
        node.lifecycle = ActionExecutor()
        node.lifecycle.start(ActionRequest("receiver", ActionKind.LAND, 180,
            {"rc_managed": True}), node.snapshot())
        node.tick()
        for t in (10.1, 10.2, 10.27):
            node.reject(DUPLICATE, t)
            _, command = node.tick()
            self.assertEqual(command.velocity_enu[2], 0)
        node.accept(10.52, heading_deg=2)
        _, command = node.tick()
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertFalse(node.lifecycle.land_yaw_alignment_complete)
        for t in (10.62, 10.72, 10.82, 10.92):
            node.accept(t, heading_deg=2)
            _, command = node.tick()
            self.assertEqual(command.velocity_enu[2], 0)
        node.accept(11.03, heading_deg=2)
        _, command = node.tick()
        self.assertLess(command.velocity_enu[2], 0)
        self.assertGreaterEqual(command.velocity_enu[2], -.05)
        self.assertTrue(node.lifecycle.land_yaw_alignment_complete)
        node.accept(11.23, heading_deg=2)
        _, command = node.tick()
        self.assertAlmostEqual(command.velocity_enu[2], -.05)

    def test_callback_pipeline_real_loss_then_new_frame_resumes_yaw_immediately(self):
        node = ReceiverHarness()
        node.accept(10, heading_deg=40)
        node.lifecycle = ActionExecutor()
        node.lifecycle.start(ActionRequest("receiver", ActionKind.LAND, 180,
            {"rc_managed": True}), node.snapshot())
        node.tick()
        node.reject("BRIDGE_REJECTED:TARGET_NOT_FOUND", 10.1)
        _, command = node.tick()
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        node.accept(10.2, heading_deg=-40)
        _, command = node.tick()
        self.assertLess(command.yaw_rate_rad_s, 0)
        self.assertEqual(command.velocity_enu[2], 0)


if __name__ == "__main__":
    unittest.main()
