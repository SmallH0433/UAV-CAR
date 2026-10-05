"""Offline alignment before descent, landing direction and freshness."""
import ast
from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from air_ground_landing.action_execution import ActionExecutor, ActionRequest, ActionKind
from air_ground_landing.hybrid_guidance import IbvsConfig, IbvsFeatureController
from air_ground_landing.landing_alignment import LandingAlignment, alignment_payload
from test_guided_land_action import flight, executor_source


class LandingCorrectionTests(unittest.TestCase):
    def start(self, kind="LAND"):
        executor = ActionExecutor()
        self.assertEqual(executor.start(ActionRequest("landing", ActionKind(kind), 10, {}), flight()).state.value, "RUNNING")
        return executor

    def test_default_land_holds_height_while_turning_and_centering(self):
        for yaw, heading in ((0, .7), (math.pi / 2, -.7), (-math.pi / 2, math.pi)):
            executor = self.start()
            _, command = executor.tick(flight(.2, yaw_rad=yaw,
                landing_heading_error_rad=heading, landing_center_error_px=180,
                target_aligned=False, landing_velocity_flu=(.1, .05)))
            self.assertEqual(command.desired_mode, "GUIDED")
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertGreater(math.hypot(*command.velocity_enu[:2]), 0)
            self.assertGreater(abs(command.yaw_rate_rad_s), 0)
            self.assertLessEqual(abs(command.yaw_rate_rad_s), math.radians(15))
            if abs(heading) < math.pi:
                self.assertGreater(command.yaw_rate_rad_s * heading, 0)
            # Body FLU velocity is rotated exactly once into earth-local ENU.
            expected = (math.cos(yaw)*.1-math.sin(yaw)*.05, math.sin(yaw)*.1+math.cos(yaw)*.05)
            self.assertAlmostEqual(command.velocity_enu[0]*expected[1] - command.velocity_enu[1]*expected[0], 0)

    def test_later_misalignment_stops_descent_immediately(self):
        executor = self.start()
        executor.tick(flight(0))
        _, command = executor.tick(flight(.41))
        self.assertLess(command.velocity_enu[2], 0)
        _, command = executor.tick(flight(.42, target_aligned=False,
            landing_heading_error_rad=-1.0, landing_center_error_px=200))
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertLess(command.yaw_rate_rad_s, 0)

    def test_stale_or_malformed_pose_clears_descent_and_yaw_immediately(self):
        for changes in ({"landing_alignment_fresh": False}, {"landing_heading_error_rad": math.nan},
                        {"landing_velocity_flu": (math.nan, 0)}, {"landing_center_error_px": -1},
                        {"candidate_fresh": False}, {"range_fresh": False}):
            with self.subTest(changes=changes):
                executor = self.start()
                executor.tick(flight(.2, landing_heading_error_rad=.5))
                _, command = executor.tick(flight(.21, **changes))
                self.assertEqual(command.velocity_enu, (0, 0, 0))
                self.assertEqual(command.yaw_rate_rad_s, 0)
                self.assertFalse(command.request_disarm)

    def test_wrong_way_parallel_is_not_treated_as_aligned(self):
        executor = self.start()
        _, command = executor.tick(flight(.2, landing_heading_error_rad=math.pi))
        self.assertNotEqual(command.yaw_rate_rad_s, 0)
        self.assertEqual(command.velocity_enu[2], 0)

    def test_pose_loss_near_ground_hands_off_to_native_land_without_disarm(self):
        executor = self.start()
        _, command = executor.tick(flight(.2, range_m=.09, landing_alignment_fresh=False))
        self.assertEqual(command.desired_mode, "LAND")
        self.assertFalse(command.request_disarm)
        self.assertIsNone(command.velocity_enu)
        _, command = executor.tick(flight(1.21, range_m=.09, landing_alignment_fresh=False))
        self.assertEqual(command.desired_mode, "LAND")
        self.assertFalse(command.request_disarm)

    def test_small_yaw_error_has_deadband_without_stopping_centering(self):
        executor = self.start()
        _, command = executor.tick(flight(.2, landing_heading_error_rad=math.radians(2)))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assertGreater(command.velocity_enu[0], 0)
        self.assertEqual(command.velocity_enu[2], 0)

    def test_same_pose_center_and_heading_must_both_be_within_tolerance(self):
        for changes in ({"landing_center_error_px": 20.01},
                        {"landing_heading_error_rad": math.radians(4.01)},
                        {"landing_heading_error_rad": math.pi}):
            with self.subTest(changes=changes):
                executor = self.start()
                executor.tick(flight(0, target_aligned=True, **changes))
                status, command = executor.tick(flight(.6, target_aligned=True, **changes))
                self.assertEqual(status.detail, "GUIDED_ALIGN")
                self.assertEqual(command.velocity_enu[2], 0)

    def test_tolerance_boundaries_and_continuous_dwell(self):
        executor = self.start()
        at_limit = dict(landing_center_error_px=20,
                        landing_heading_error_rad=math.radians(4))
        for t in (0, .2, .399):
            status, command = executor.tick(flight(t, **at_limit))
            self.assertEqual(status.detail, "GUIDED_VERIFY_ALIGNMENT")
            self.assertEqual(command.velocity_enu[2], 0)
        status, command = executor.tick(flight(.4, **at_limit))
        self.assertEqual(status.detail, "GUIDED_TRACK_DESCENT")
        self.assertLess(command.velocity_enu[2], 0)

    def test_misalignment_resets_dwell_and_restarts_from_zero_vertical_velocity(self):
        executor = self.start()
        executor.tick(flight(0))
        executor.tick(flight(.3, landing_center_error_px=21))
        executor.tick(flight(.31))
        _, command = executor.tick(flight(.6))
        self.assertEqual(command.velocity_enu[2], 0)
        _, command = executor.tick(flight(.72))
        self.assertLess(command.velocity_enu[2], 0)
        executor.tick(flight(.73, landing_heading_error_rad=.5))
        executor.tick(flight(.74))
        _, command = executor.tick(flight(1.0))
        self.assertEqual(command.velocity_enu[2], 0)
        _, command = executor.tick(flight(1.15))
        self.assertLess(command.velocity_enu[2], 0)

    def test_range_loss_requires_new_alignment_dwell(self):
        executor = self.start()
        executor.tick(flight(0))
        executor.tick(flight(.41))
        _, command = executor.tick(flight(.42, range_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        executor.tick(flight(.5))
        _, command = executor.tick(flight(.7))
        self.assertEqual(command.velocity_enu[2], 0)
        _, command = executor.tick(flight(.91))
        self.assertLess(command.velocity_enu[2], 0)

    def test_alignment_parameters_are_validated(self):
        for name in ("land_center_tolerance_px", "land_alignment_dwell_s"):
            for value in (0, -1, math.nan, math.inf):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    ActionExecutor(**{name: value})

    def test_precision_land_requires_alignment_even_in_terminal_phase(self):
        executor = self.start("PRECISION_LAND")
        for t, height in ((.2, .8), (.4, .09), (.9, .09), (1.0, .08)):
            _, command = executor.tick(flight(t, range_m=height, target_aligned=False,
                landing_heading_error_rad=.5))
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertGreater(command.velocity_enu[0], 0)
            self.assertGreater(command.yaw_rate_rad_s, 0)
        _, command = executor.tick(flight(1.1, range_m=.08, landing_alignment_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        executor.tick(flight(1.2, range_m=.08))
        executor.tick(flight(1.61, range_m=.08))
        _, command = executor.tick(flight(2.02, range_m=.08))
        self.assertLess(command.velocity_enu[2], 0)
        self.assertIsNotNone(executor._descent.terminal_since)
        _, command = executor.tick(flight(2.03, range_m=.08, landing_heading_error_rad=.5))
        self.assertEqual(command.velocity_enu[2], 0)

    def test_yaw_rate_respects_general_executor_limit(self):
        executor = ActionExecutor(maximum_yaw_rate_rad_s=.1)
        executor.start(ActionRequest("limited", ActionKind.LAND, 10, {}), flight())
        _, command = executor.tick(flight(.2, landing_heading_error_rad=1))
        self.assertAlmostEqual(command.yaw_rate_rad_s, .1)

    def test_timeout_and_ch8_exit_stop_turn_and_descent(self):
        for changes in ({"now_s": 10}, {"landing_requested": False, "landing_explicit_low": True}):
            executor = ActionExecutor()
            executor.start(ActionRequest("limited", ActionKind.LAND, 10, {"rc_managed": True}), flight())
            executor.tick(flight(.2, landing_heading_error_rad=1))
            _, command = executor.tick(replace(flight(.3), **changes))
            self.assertEqual(command.velocity_enu, (0, 0, 0))
            self.assertEqual(command.yaw_rate_rad_s, 0)

    def test_tag_loss_timeout_keeps_zero_command_until_loiter_heartbeat(self):
        executor = self.start()
        executor.tick(flight(.2, landing_heading_error_rad=.5))
        status, command = executor.tick(flight(.3, candidate_fresh=False))
        self.assertEqual(status.detail, "TAG_LOST_GUIDED_HOLD")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        status, command = executor.tick(flight(1.31, candidate_fresh=False))
        self.assertEqual(command.desired_mode, "LOITER")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        status, command = executor.tick(flight(1.4, candidate_fresh=False))
        self.assertEqual(status.detail, "WAITING_LOITER_HEARTBEAT")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        status, command = executor.tick(flight(1.5, mode="LOITER", candidate_fresh=False))
        self.assertEqual(status.state.value, "DONE")
        self.assertEqual(status.reason, "LAND_EXIT_TAG_REACQUIRE_TIMEOUT")

    def test_recovery_reacquires_and_aligns_before_resuming_descent(self):
        executor = self.start()
        executor.tick(flight(.2, landing_heading_error_rad=.5))
        _, command = executor.tick(flight(.3, candidate_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        _, command = executor.tick(flight(.6, candidate_fresh=True,
            landing_heading_error_rad=-.5, landing_center_error_px=90))
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertEqual(command.velocity_enu[2], 0)
        self.assertGreater(command.velocity_enu[0], 0)
        self.assertLess(command.yaw_rate_rad_s, 0)

    def test_timeout_with_tag_exits_to_guided_without_descent(self):
        executor = self.start()
        executor.tick(flight(.2))
        executor.tick(flight(.3, candidate_fresh=False))
        status, command = executor.tick(flight(1.3, candidate_fresh=True))
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)
        status, command = executor.tick(flight(1.4))
        self.assertEqual(status.reason, "LAND_EXIT_TAG_REACQUIRE_TIMEOUT")
        self.assertEqual(status.state.value, "DONE")
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))


class AlignmentEvidenceTests(unittest.TestCase):
    def payload(self, angle=math.pi/2):
        observation = SimpleNamespace(orientation_body_frd_wxyz=(math.cos(angle/2), 0, 0, math.sin(angle/2)), capture_time_s=1)
        features = SimpleNamespace(valid=True, correction_body_frd_mps=(.1, .2, 0), centroid_error_px=50)
        return alignment_payload(features, observation, 1.1)

    def test_frd_right_heading_becomes_clockwise_ros_yaw(self):
        for angle in (-math.pi/2, 0, math.pi/2, math.pi):
            payload = self.payload(angle)
            self.assertAlmostEqual(payload["heading_error_rad"], -angle)
            self.assertEqual(payload["velocity_flu"], [.1, -.2])

    def test_camera_projection_is_not_reused_as_body_heading(self):
        observation = SimpleNamespace(orientation_body_frd_wxyz=(math.sqrt(.5), 0, 0, math.sqrt(.5)), capture_time_s=1)
        features = SimpleNamespace(valid=True, correction_body_frd_mps=(0, 0, 0), centroid_error_px=0)
        # Common pad quaternion is already corrected for the inner tag's 45 deg layout.
        result = alignment_payload(features, observation, 1.01)
        self.assertAlmostEqual(result["heading_error_rad"], -math.pi/2)
        observation.orientation_body_frd_wxyz = None
        self.assertIsNone(alignment_payload(features, observation, 1.01))

    def test_source_age_plus_receipt_age_expires(self):
        evidence = LandingAlignment.from_payload(self.payload(), 10)
        self.assertTrue(evidence.fresh(10.2, .5))
        self.assertFalse(evidence.fresh(10.41, .5))
        self.assertFalse(evidence.fresh(9.9, .5))
        for bad in (None, [], {"frame": "BODY_FRD"},
                    {**self.payload(), "source_age_s": -1},
                    {**self.payload(), "heading_error_rad": math.nan},
                    {**self.payload(), "velocity_flu": [True, 0]}):
            self.assertIsNone(LandingAlignment.from_payload(bad, 10))

    def test_image_geometric_center_not_optical_principal_point(self):
        config = IbvsConfig(1280, 800, 800, 800, 650, 430,
            ((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
        controller = IbvsFeatureController(replace(config, cx_px=640, cy_px=400))
        observation = SimpleNamespace(capture_time_s=1, quality=1, tag_id=1, distance_m=.5)
        for cx, cy in ((640, 400), (650, 430), (740, 400)):
            status = dict(analysis_size=[1280, 800], corners_px=[[cx-10, cy-10], [cx+10, cy-10], [cx+10, cy+10], [cx-10, cy+10]])
            features = controller.process_status(status, observation, now_s=1.01)
            self.assertTrue(features.valid)
            self.assertAlmostEqual(features.centroid_error_px, math.hypot(cx-640, cy-400))
            if cx == 640:
                self.assertEqual(features.correction_body_frd_mps, (0, 0, 0))
            else:
                self.assertGreater(features.correction_body_frd_mps[1], 0)

    def test_receiver_clears_previous_heading_on_bad_status(self):
        tree = ast.parse(executor_source().read_text(encoding="utf-8"))
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
                      if isinstance(n, ast.FunctionDef) and n.name == "_target_status")
        scope = dict(String=object, json=json, LandingAlignment=LandingAlignment)
        exec(compile(ast.Module(body=[method], type_ignores=[]), "receiver", "exec"), scope)
        node = SimpleNamespace(_now_s=lambda: 10)
        for bad in ('null', '[]', '{', '{"healthy":false}'):
            scope['_target_status'](node, SimpleNamespace(data=json.dumps({"healthy": True, "landing_alignment": self.payload()})))
            self.assertIsNotNone(node.landing_alignment)
            scope['_target_status'](node, SimpleNamespace(data=bad))
            self.assertIsNone(node.landing_alignment)
            self.assertFalse(node.target_healthy)

    def test_position_target_enables_velocity_and_yaw_rate_together(self):
        from air_ground_landing.action_execution import ActionCommand
        class Target:
            FRAME_LOCAL_NED = 1
            IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
            IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
            IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048
            def __init__(self):
                self.header = SimpleNamespace()
                self.velocity = SimpleNamespace()
        tree = ast.parse(executor_source().read_text(encoding="utf-8"))
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
                      if isinstance(n, ast.FunctionDef) and n.name == "_position_target")
        scope = dict(PositionTarget=Target, ActionCommand=ActionCommand)
        exec(compile(ast.Module(body=[method], type_ignores=[]), "transport", "exec"), scope)
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
        target = scope['_position_target'](node, ActionCommand(velocity_enu=(.1, -.05, -.1), yaw_rate_rad_s=-.2))
        self.assertEqual(target.coordinate_frame, Target.FRAME_LOCAL_NED)
        self.assertEqual(target.type_mask & (8 | 16 | 32 | Target.IGNORE_YAW_RATE), 0)
        self.assertTrue(target.type_mask & Target.IGNORE_YAW)
        self.assertEqual((target.velocity.x, target.velocity.y, target.velocity.z, target.yaw_rate), (.1, -.05, -.1, -.2))


if __name__ == '__main__':
    unittest.main()
