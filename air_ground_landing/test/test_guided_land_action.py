"""CH8 hybrid landing checks without ROS or hardware outputs."""
import ast
from pathlib import Path
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor, ActionKind, ActionRequest, ActionState, GUIDED_ACTIONS,
)
from test_action_execution import snapshot, request


def executor_source():
    root = Path(__file__).resolve().parents[1]
    workspace = root / "ros2_ws_v2"
    if not workspace.is_dir():
        workspace = root / "ros2_ws"
    return workspace / "src/air_ground_landing_ros2/air_ground_landing_ros2/action_executor.py"


def flight(now=0.0, **changes):
    values = dict(mode="GUIDED", candidate_fresh=True,
                  candidate_velocity_flu=(.1, .0), range_fresh=True, range_m=.8,
                  landing_alignment_fresh=True, landing_velocity_flu=(.1, .0),
                  landing_center_error_px=40.0, landing_heading_error_rad=0.0,
                  landing_target_fresh=True, landing_target_output_enabled=True)
    values.update(changes)
    return snapshot(now, **values)


class GuidedLandTests(unittest.TestCase):
    def setUp(self):
        self.executor = ActionExecutor()
        status = self.executor.start(request("LAND", guided_descent=True,
                                             rc_managed=True), flight())
        self.assertEqual(status.state, ActionState.RUNNING)

    def test_ch8_descends_in_guided_then_hands_off_at_threshold(self):
        status, command = self.executor.tick(flight(.2))
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertGreater(command.velocity_enu[0], 0)
        self.assertLess(command.velocity_enu[2], 0)
        status, command = self.executor.tick(flight(.4, range_m=.10))
        self.assertEqual(command.desired_mode, "LAND")
        self.assertIsNone(command.velocity_enu)

    def test_loss_immediately_stops_all_velocity_then_resumes_guided_descent(self):
        self.executor.tick(flight(.2))
        status, command = self.executor.tick(flight(.3, candidate_fresh=False))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        status, command = self.executor.tick(flight(.5))
        self.assertEqual(status.detail, "GUIDED_TRACK_DESCENT")
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertLess(command.velocity_enu[2], 0)

    def test_deadline_wins_over_late_reacquisition(self):
        for visible, expected in ((True, "GUIDED"), (False, "LOITER")):
            with self.subTest(visible=visible):
                self.setUp()
                self.executor.tick(flight(.25, candidate_fresh=False))
                status, command = self.executor.tick(flight(1.25, candidate_fresh=visible))
                self.assertEqual(command.desired_mode, expected)
                self.assertEqual(self.executor.land_exit_reason, "LAND_EXIT_TAG_REACQUIRE_TIMEOUT")
                self.assertTrue(self.executor.land_phase.startswith("EXIT_"))

    def test_ch8_release_exits_by_visibility_and_never_descends(self):
        for visible, expected in ((True, "GUIDED"), (False, "LOITER")):
            with self.subTest(visible=visible):
                self.setUp()
                self.executor.tick(flight(.2))
                status, command = self.executor.tick(flight(
                    .3, candidate_fresh=visible, landing_requested=False,
                    landing_explicit_low=True))
                self.assertEqual(command.desired_mode, expected)
                self.assertIn(command.velocity_enu, (None, (0, 0, 0)))

    def test_recovery_latch_holds_until_deadline_despite_ch8_release(self):
        self.executor.tick(flight(.25, candidate_fresh=False))
        status, command = self.executor.tick(flight(
            .5, candidate_fresh=False, landing_requested=False, landing_explicit_low=True))
        self.assertEqual(status.detail, "TAG_LOST_GUIDED_HOLD")
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        status, command = self.executor.tick(flight(1.25, candidate_fresh=False))
        self.assertEqual(command.desired_mode, "LOITER")

    def test_invalid_range_holds_without_descent(self):
        self.executor.tick(flight(.2))
        status, command = self.executor.tick(flight(.4, range_fresh=False))
        self.assertEqual(status.detail, "RANGE_INVALID_GUIDED_HOLD")
        self.assertEqual(command.velocity_enu, (0, 0, 0))

    def test_rc6_withdrawal_overrides_recovery(self):
        self.executor.tick(flight(.25, candidate_fresh=False))
        status, command = self.executor.tick(flight(.5, authorized=False))
        self.assertEqual(command.desired_mode, "LOITER")

    def test_below_threshold_loss_requests_disarm_until_heartbeat(self):
        status, command = self.executor.tick(flight(.2, range_m=.09, candidate_fresh=False))
        self.assertTrue(command.request_disarm)
        self.assertEqual(command.desired_mode, "LAND")
        self.assertEqual(status.state, ActionState.RUNNING)
        status, command = self.executor.tick(flight(
            .4, mode="LAND", candidate_fresh=True, range_fresh=False,
            landing_requested=False, landing_explicit_low=True))
        self.assertTrue(command.request_disarm)
        status, command = self.executor.tick(flight(.6, mode="LAND", armed=False))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "LAND_AND_DISARM_CONFIRMED")

    def test_tag_loss_after_native_land_handoff_still_requests_disarm(self):
        self.executor.tick(flight(.2, range_m=.10))
        status, command = self.executor.tick(flight(
            .4, mode="LAND", range_m=.09, candidate_fresh=False))
        self.assertTrue(command.request_disarm)

    def test_recovery_drifting_below_threshold_requests_disarm(self):
        self.executor.tick(flight(.2, candidate_fresh=False))
        status, command = self.executor.tick(flight(.4, range_m=.09, candidate_fresh=False))
        self.assertTrue(command.request_disarm)

    def test_exact_threshold_and_stale_range_do_not_request_disarm(self):
        for changes in (dict(range_m=.10), dict(range_m=.09, range_fresh=False)):
            with self.subTest(changes=changes):
                self.setUp()
                status, command = self.executor.tick(flight(.2, candidate_fresh=False, **changes))
                self.assertFalse(command.request_disarm)

    def test_landed_alone_does_not_complete_hybrid_action(self):
        status, command = self.executor.tick(flight(.2, mode="LAND", landed=True))
        self.assertEqual(status.state, ActionState.RUNNING)
        self.assertEqual(status.detail, "LANDED_WAIT_DISARM")

    def test_ros_float32_threshold_hands_off_without_false_disarm(self):
        import struct
        range_m = struct.unpack('f', struct.pack('f', .10))[0]
        status, command = self.executor.tick(flight(.2, range_m=range_m, candidate_fresh=False))
        self.assertEqual(command.desired_mode, "LAND")
        self.assertFalse(command.request_disarm)


class RcFlowTests(unittest.TestCase):
    def test_disarm_transport_is_normal_and_has_separate_output_gate(self):
        from types import SimpleNamespace
        path = executor_source()
        tree = ast.parse(path.read_text(encoding="utf-8"))
        methods = [n for c in tree.body if isinstance(c, ast.ClassDef)
                   for n in c.body if isinstance(n, ast.FunctionDef)
                   and n.name in {"_request_command", "_command_result"}]
        namespace = dict(CommandLong=SimpleNamespace(Request=SimpleNamespace),
                         MAV_CMD_COMPONENT_ARM_DISARM=400)
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
        sent = []
        lifecycle = ActionExecutor()
        lifecycle.start(request("LAND", guided_descent=True), flight())
        lifecycle.tick(flight(.2, range_m=.09, candidate_fresh=False))
        driver = SimpleNamespace(
            allow_disarm=False, allow_landing_disarm=False, lifecycle=lifecycle,
            command_future=None, command_request_s=0, command_retry_s=1,
            command_client=SimpleNamespace(service_is_ready=lambda: True,
                call_async=lambda r: sent.append(r) or SimpleNamespace(add_done_callback=lambda cb: None)),
        )
        namespace['_request_command'](driver, 'DISARM', 2)
        self.assertEqual(sent, [])
        driver.allow_landing_disarm = True
        namespace['_request_command'](driver, 'DISARM', 2)
        self.assertEqual(len(sent), 1)
        self.assertEqual((sent[0].command, sent[0].param1, sent[0].param2), (400, 0, 0))
        future = SimpleNamespace(result=lambda: SimpleNamespace(success=False))
        namespace['_command_result'](driver, future, lifecycle.request.action_id, 'DISARM')
        self.assertTrue(lifecycle.active)
        self.assertEqual(lifecycle.reason, 'DISARM_REJECTED_CONTINUING_LAND')

    def test_actual_rc_driver_selects_hybrid_and_requires_ch8_reset_after_exit(self):
        # Compile the real orchestration method in isolation, without ROS stubs.
        path = executor_source()
        tree = ast.parse(path.read_text(encoding="utf-8"))
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                      for n in c.body if isinstance(n, ast.FunctionDef)
                      and n.name == "_drive_rc_flight_flow")
        namespace = dict(ActionKind=ActionKind, ActionRequest=ActionRequest,
                         GUIDED_ACTIONS=GUIDED_ACTIONS, math=__import__('math'),
                         VehicleSnapshot=object)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        class Driver:
            rc_flight_flow_enabled = True
            guided_mode = "GUIDED"
            entry_modes = {"LOITER", "ALT_HOLD"}
            auto_land_inhibited = False
            auto_action_sequence = 0
            last_auto_follow_attempt_s = -float('inf')
            lifecycle = ActionExecutor()
            def _publish_status(self, *args, **kwargs):
                pass
        driver = Driver()
        drive = lambda s: namespace['_drive_rc_flight_flow'](driver, s, s.now_s)
        drive(flight(0, landing_requested=False))
        self.assertEqual(driver.lifecycle.request.kind, ActionKind.FOLLOW)
        drive(flight(.1))
        self.assertTrue(driver.lifecycle.request.params['guided_descent'])
        driver.lifecycle.tick(flight(.25, candidate_fresh=False))
        driver.lifecycle.tick(flight(1.25))  # Deadline, visible tag -> GUIDED exit.
        driver.lifecycle.tick(flight(1.3))
        drive(flight(1.4))
        self.assertEqual(driver.lifecycle.request.kind, ActionKind.FOLLOW)
        drive(flight(1.5))
        self.assertEqual(driver.lifecycle.request.kind, ActionKind.FOLLOW)
        drive(flight(1.6, landing_requested=False, landing_explicit_low=True))
        drive(flight(1.7))
        self.assertEqual(driver.lifecycle.request.kind, ActionKind.LAND)

    def test_ch6_failed_follow_retries_at_most_once_per_second(self):
        path = executor_source()
        tree = ast.parse(path.read_text(encoding="utf-8"))
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                      for n in c.body if isinstance(n, ast.FunctionDef)
                      and n.name == "_drive_rc_flight_flow")
        namespace = dict(ActionKind=ActionKind, ActionRequest=ActionRequest,
                         GUIDED_ACTIONS=GUIDED_ACTIONS, math=__import__('math'),
                         VehicleSnapshot=object)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        class Driver:
            rc_flight_flow_enabled = True
            guided_mode = "GUIDED"
            entry_modes = {"LOITER", "ALT_HOLD"}
            auto_land_inhibited = False
            auto_action_sequence = 0
            last_auto_follow_attempt_s = -float('inf')
            lifecycle = ActionExecutor()
            def _publish_status(self, *args, **kwargs):
                pass
        driver = Driver()
        drive = lambda t: namespace['_drive_rc_flight_flow'](
            driver, flight(t, mode="LOITER", landing_requested=False,
                           ekf_healthy=False), t)
        drive(0.0)
        self.assertEqual(driver.auto_action_sequence, 1)
        for t in (0.05, 0.2, 0.5, 0.99):
            drive(t)
        self.assertEqual(driver.auto_action_sequence, 1)
        drive(1.0)
        self.assertEqual(driver.auto_action_sequence, 2)


if __name__ == '__main__':
    unittest.main()
