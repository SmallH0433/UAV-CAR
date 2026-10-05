import math
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor,
    ActionKind,
    ActionRequest,
    ActionState,
    VehicleSnapshot,
)


def snapshot(now=0.0, **changes):
    values = dict(
        now_s=now,
        telemetry_fresh=True,
        connected=True,
        armed=True,
        mode="LOITER",
        authorized=True,
        ekf_healthy=True,
        home_set=True,
        landed=False,
        position_enu=(0.0, 0.0, 1.0),
        velocity_enu=(0.0, 0.0, 0.0),
        yaw_rad=0.0,
    )
    values.update(changes)
    return VehicleSnapshot(**values)


def request(kind, *, action_id="a1", timeout=5.0, **params):
    # These legacy LAND tests explicitly exercise the native FCU backend.
    # New precision LAND requests align before GUIDED descent.
    if kind == "LAND":
        params.setdefault("guided_descent", False)
    return ActionRequest(action_id, ActionKind(kind), timeout, params)


class ActionRequestTests(unittest.TestCase):
    def test_json_contract_is_strict(self):
        parsed = ActionRequest.from_mapping({
            "action_id": "mission-7/move-1",
            "action": "move",
            "timeout_s": 3,
            "params": {"direction": [1, 0, 0], "distance_m": 1, "speed_mps": .2},
        })
        self.assertEqual(parsed.kind, ActionKind.MOVE)
        for bad in (
            {},
            {"action_id": "x", "action": "unknown", "timeout_s": 1},
            {"action_id": "x", "action": "hover", "timeout_s": float("nan")},
        ):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    ActionRequest.from_mapping(bad)


class ActionExecutorTests(unittest.TestCase):
    def setUp(self):
        self.executor = ActionExecutor(completion_dwell_s=.2, follow_loss_grace_s=.3)

    def test_only_one_action_can_run(self):
        status = self.executor.start(request("HOVER", duration_s=1), snapshot())
        self.assertEqual(status.state, ActionState.RUNNING)
        rejected = self.executor.start(
            request(
                "MOVE",
                action_id="a2",
                direction=[1, 0, 0],
                distance_m=1,
                speed_mps=.2,
            ),
            snapshot(),
        )
        self.assertEqual(rejected.reason, "REJECTED_BUSY")
        self.assertEqual(rejected.action_id, "a2")
        self.assertEqual(self.executor.request.action_id, "a1")

    def test_action_id_replay_never_runs_twice(self):
        original = request("HOVER", duration_s=.1)
        self.executor.start(original, snapshot())
        self.executor.tick(snapshot(.1, mode="GUIDED"))
        replay = self.executor.start(original, snapshot(.2, mode="GUIDED"))
        self.assertEqual(replay.state, ActionState.FAILED)
        self.assertEqual(replay.reason, "REJECTED_ACTION_ID_REPLAY")

    def test_mode_ack_requires_heartbeat_before_setpoint(self):
        self.executor.start(request("HOVER", duration_s=1), snapshot())
        status, command = self.executor.tick(snapshot(.1, mode="LOITER"))
        self.assertEqual(status.detail, "WAITING_GUIDED_HEARTBEAT")
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertIsNone(command.velocity_enu)
        _, command = self.executor.tick(snapshot(.2, mode="GUIDED"))
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))

    def test_authorization_withdrawal_releases_control(self):
        self.executor.start(request("HOVER", duration_s=1), snapshot())
        status, command = self.executor.tick(snapshot(.1, mode="GUIDED", authorized=False))
        self.assertEqual(status.state, ActionState.CANCELLED)
        self.assertEqual(status.reason, "AUTHORIZATION_WITHDRAWN")
        self.assertEqual(command.desired_mode, "LOITER")
        self.assertTrue(status.control_retained)

    def test_nonfinite_telemetry_is_rejected(self):
        status = self.executor.start(
            request("HOVER", duration_s=1),
            snapshot(yaw_rad=float("nan")),
        )
        self.assertEqual(status.state, ActionState.FAILED)
        self.assertEqual(status.reason, "REJECTED_NONFINITE_FLIGHT_TELEMETRY")

    def test_action_cannot_exceed_executor_speed_envelope(self):
        status = self.executor.start(
            request("MOVE", direction=[1, 0, 0], distance_m=1, speed_mps=.6),
            snapshot(),
        )
        self.assertEqual(status.state, ActionState.FAILED)
        self.assertEqual(status.reason, "REJECTED_INVALID_PARAMETERS")

    def test_move_is_closed_loop_and_holds_after_done(self):
        self.executor.start(
            request(
                "MOVE",
                direction=[1, 0, 0],
                distance_m=1,
                speed_mps=.4,
            ),
            snapshot(),
        )
        _, command = self.executor.tick(snapshot(.1, mode="GUIDED"))
        self.assertAlmostEqual(command.velocity_enu[0], .05)
        self.executor.tick(snapshot(
            1.0,
            mode="GUIDED",
            position_enu=(1.0, 0.0, 1.0),
        ))
        status, command = self.executor.tick(snapshot(
            1.21,
            mode="GUIDED",
            position_enu=(1.0, 0.0, 1.0),
        ))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertTrue(status.control_retained)
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))

    def test_timeout_reports_timeout_and_holds_when_safe(self):
        self.executor.start(
            request(
                "MOVE",
                timeout=.5,
                direction=[1, 0, 0],
                distance_m=10,
                speed_mps=.2,
            ),
            snapshot(),
        )
        status, command = self.executor.tick(snapshot(.5, mode="GUIDED"))
        self.assertEqual(status.state, ActionState.TIMEOUT)
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))
        status, command = self.executor.tick(snapshot(.6, mode="ALT_HOLD"))
        self.assertIsNone(command)
        self.assertFalse(status.control_retained)

    def test_follow_loss_has_private_grace_then_fails(self):
        self.executor.start(request("FOLLOW"), snapshot())
        status, command = self.executor.tick(snapshot(.1, mode="GUIDED"))
        self.assertEqual(status.detail, "FOLLOW_TARGET_GRACE_HOLD")
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))
        status, command = self.executor.tick(snapshot(.41, mode="GUIDED"))
        self.assertEqual(status.state, ActionState.FAILED)
        self.assertEqual(status.reason, "FOLLOW_TARGET_LOST")
        self.assertTrue(status.control_retained)

    def test_follow_body_velocity_converts_to_local_enu(self):
        self.executor.start(request("FOLLOW", maximum_speed_mps=.5), snapshot())
        _, command = self.executor.tick(snapshot(
            .1,
            mode="GUIDED",
            yaw_rad=math.pi / 2,
            candidate_velocity_flu=(.2, 0),
            candidate_fresh=True,
        ))
        self.assertAlmostEqual(command.velocity_enu[0], 0.0, places=6)
        self.assertAlmostEqual(command.velocity_enu[1], .05, places=6)

    def test_follow_recovery_accelerates_from_last_zero_output(self):
        executor = ActionExecutor(maximum_horizontal_acceleration_mps2=.4)
        executor.start(request("FOLLOW"), snapshot(mode="GUIDED"))
        def tick(t, fresh):
            return executor.tick(snapshot(t, mode="GUIDED", candidate_fresh=fresh,
                candidate_velocity_flu=(.2, .2)))[1].velocity_enu
        self.assertGreater(math.hypot(*tick(.2, True)[:2]), .07)
        for t in (.21, .22, .4):
            self.assertEqual(tick(t, False), (0, 0, 0))
            self.assertEqual(executor.last_velocity_command, (0, 0, 0))
            self.assertEqual(executor.last_velocity_command_s, t)
        resumed = tick(.41, True)
        self.assertGreater(math.hypot(*resumed[:2]), 0)
        self.assertLessEqual(math.hypot(*resumed[:2]), .004 + 1e-12)
        next_velocity = tick(.42, True)
        self.assertLessEqual(math.hypot(next_velocity[0]-resumed[0],
                                       next_velocity[1]-resumed[1]), .004 + 1e-12)

    def test_follow_missing_velocity_also_resets_limiter(self):
        self.executor.start(request("FOLLOW"), snapshot(mode="GUIDED"))
        self.executor.tick(snapshot(.2, mode="GUIDED", candidate_fresh=True,
            candidate_velocity_flu=(.2, 0)))
        _, command = self.executor.tick(snapshot(.21, mode="GUIDED", candidate_fresh=True,
            candidate_velocity_flu=None))
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(self.executor.last_velocity_command, (0, 0, 0))

    def test_disarm_rejects_airborne_and_completes_on_heartbeat(self):
        status = self.executor.start(request("DISARM"), snapshot())
        self.assertEqual(status.reason, "REJECTED_NOT_LANDED")
        status = self.executor.start(
            request("DISARM", action_id="a2"),
            snapshot(1, landed=True),
        )
        self.assertEqual(status.state, ActionState.RUNNING)
        _, command = self.executor.tick(snapshot(1.1, landed=True))
        self.assertTrue(command.request_disarm)
        status, _ = self.executor.tick(snapshot(1.2, landed=True, armed=False))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "DISARM_CONFIRMED")

    def test_native_land_requires_confirmed_guided_entry(self):
        status = self.executor.start(
            request("LAND"),
            snapshot(
                mode="LOITER",
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        self.assertEqual(status.state, ActionState.FAILED)
        self.assertEqual(status.reason, "REJECTED_LAND_REQUIRES_GUIDED")

    def test_precision_land_requires_active_landing_target_output(self):
        status = self.executor.start(request("LAND"), snapshot(mode="GUIDED"))
        self.assertEqual(status.reason, "REJECTED_LANDING_TARGET_OUTPUT_DISABLED")

        status = self.executor.start(
            request("LAND", action_id="a2"),
            snapshot(mode="GUIDED", landing_target_output_enabled=True),
        )
        self.assertEqual(status.reason, "REJECTED_LANDING_TARGET_UNHEALTHY")

    def test_guided_to_land_waits_for_land_heartbeat(self):
        start = snapshot(
            mode="GUIDED",
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        )
        status = self.executor.start(request("LAND"), start)
        self.assertEqual(status.state, ActionState.RUNNING)
        status, command = self.executor.tick(snapshot(
            .1,
            mode="GUIDED",
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        ))
        self.assertEqual(status.detail, "WAITING_LAND_HEARTBEAT")
        self.assertEqual(command.desired_mode, "LAND")
        self.assertIsNone(command.velocity_enu)

        status, command = self.executor.tick(snapshot(
            .2,
            mode="LAND",
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        ))
        self.assertEqual(status.detail, "FCU_LAND_ACTIVE")
        self.assertEqual(command.desired_mode, "LAND")
        self.assertIsNone(command.velocity_enu)

    def test_land_completion_and_timeout_never_restore_guided(self):
        start = snapshot(
            mode="GUIDED",
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        )
        self.executor.start(request("LAND", timeout=.5), start)
        status, command = self.executor.tick(snapshot(.2, mode="LAND", landed=True))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "LANDING_DETECTED")
        self.assertIsNone(command.desired_mode)
        self.assertFalse(status.control_retained)

        other = ActionExecutor()
        other.start(request("LAND", action_id="a2", timeout=.5), start)
        status, command = other.tick(snapshot(.5, mode="LAND"))
        self.assertEqual(status.state, ActionState.TIMEOUT)
        self.assertEqual(status.detail, "FCU_LAND_CONTINUES")
        self.assertIsNone(command)
        self.assertFalse(status.control_retained)

    def test_nonprecision_land_can_start_without_target_stream(self):
        status = self.executor.start(
            request("LAND", precision=False),
            snapshot(mode="GUIDED"),
        )
        self.assertEqual(status.state, ActionState.RUNNING)

    def test_land_authorization_withdrawal_does_not_request_rollback(self):
        self.executor.start(
            request("LAND"),
            snapshot(
                mode="GUIDED",
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        status, command = self.executor.tick(
            snapshot(.1, mode="LAND", authorized=False)
        )
        self.assertEqual(status.state, ActionState.RUNNING)
        self.assertEqual(status.detail, "WAITING_LOITER_HEARTBEAT")
        self.assertEqual(command.desired_mode, "LOITER")

    def test_land_tag_loss_above_threshold_holds_then_reacquires(self):
        self.executor.start(
            request("LAND", rc_managed=True, loss_recovery=True),
            snapshot(
                mode="GUIDED",
                candidate_fresh=True,
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        self.executor.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=True,
            range_m=.8,
            range_fresh=True,
        ))
        status, command = self.executor.tick(snapshot(
            .2,
            mode="LAND",
            candidate_fresh=False,
            range_m=.7,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "TAG_LOST_REQUEST_GUIDED_HOLD")
        self.assertEqual(command.desired_mode, "GUIDED")
        status, command = self.executor.tick(snapshot(
            .3,
            mode="GUIDED",
            candidate_fresh=False,
            range_m=.7,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "TAG_LOST_GUIDED_HOLD")
        self.assertEqual(command.velocity_enu, (0.0, 0.0, 0.0))
        status, command = self.executor.tick(snapshot(
            .4,
            mode="GUIDED",
            candidate_fresh=True,
            range_m=.7,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "TAG_REACQUIRED_REQUEST_LAND")
        self.assertEqual(command.desired_mode, "LAND")
        status, command = self.executor.tick(snapshot(
            .5,
            mode="LAND",
            candidate_fresh=True,
            range_m=.6,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "FCU_LAND_ACTIVE")
        self.assertEqual(command.desired_mode, "LAND")

    def test_land_tag_loss_timeout_exits_to_loiter(self):
        self.executor = ActionExecutor(
            completion_dwell_s=.2,
            follow_loss_grace_s=.3,
            land_recovery_timeout_s=.5,
        )
        self.executor.start(
            request("LAND", rc_managed=True, loss_recovery=True),
            snapshot(
                mode="GUIDED",
                candidate_fresh=True,
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        self.executor.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=False,
            range_m=.8,
            range_fresh=True,
        ))
        self.executor.tick(snapshot(
            .2,
            mode="GUIDED",
            candidate_fresh=False,
            range_m=.8,
            range_fresh=True,
        ))
        status, command = self.executor.tick(snapshot(
            .71,
            mode="GUIDED",
            candidate_fresh=False,
            range_m=.8,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "WAITING_LOITER_HEARTBEAT")
        self.assertEqual(command.desired_mode, "LOITER")
        status, command = self.executor.tick(snapshot(
            .8,
            mode="LOITER",
            candidate_fresh=False,
            range_m=.8,
            range_fresh=True,
        ))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "LAND_EXIT_TAG_REACQUIRE_TIMEOUT")
        self.assertIsNone(command.desired_mode)

    def test_land_tag_loss_below_threshold_commits_land(self):
        self.executor.start(
            request("LAND", rc_managed=True, loss_recovery=True),
            snapshot(
                mode="GUIDED",
                candidate_fresh=True,
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        status, command = self.executor.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=False,
            range_m=.09,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "LAND_COMMITTED")
        self.assertEqual(command.desired_mode, "LAND")

    def test_pending_guided_heartbeat_is_consumed_before_low_height_land_commit(self):
        self.executor.start(
            request("LAND", rc_managed=True, loss_recovery=True),
            snapshot(
                mode="GUIDED",
                candidate_fresh=True,
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        self.executor.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=False,
            range_m=.8,
            range_fresh=True,
        ))
        status, command = self.executor.tick(snapshot(
            .2,
            mode="LAND",
            candidate_fresh=False,
            range_m=.09,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "TAG_LOST_REQUEST_GUIDED_HOLD")
        self.assertEqual(command.desired_mode, "GUIDED")

        status, command = self.executor.tick(snapshot(
            .3,
            mode="GUIDED",
            candidate_fresh=False,
            range_m=.09,
            range_fresh=True,
        ))
        self.assertEqual(status.detail, "RECOVERY_ABORTED_BELOW_COMMIT_HEIGHT")
        self.assertEqual(command.desired_mode, "LAND")

    def test_ch8_low_exits_land_by_tag_visibility(self):
        start = snapshot(
            mode="GUIDED",
            candidate_fresh=True,
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        )
        self.executor.start(request("LAND", rc_managed=True), start)
        status, command = self.executor.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=True,
            landing_requested=False,
            landing_explicit_low=True,
        ))
        self.assertEqual(command.desired_mode, "GUIDED")
        status, command = self.executor.tick(snapshot(
            .2,
            mode="GUIDED",
            candidate_fresh=True,
            landing_requested=False,
            landing_explicit_low=True,
        ))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "LAND_EXIT_CH8_RELEASED")
        self.assertTrue(status.control_retained)

        other = ActionExecutor()
        other.start(request("LAND", action_id="a2", rc_managed=True), start)
        _, command = other.tick(snapshot(
            .1,
            mode="LAND",
            candidate_fresh=False,
            landing_requested=False,
            landing_explicit_low=True,
        ))
        self.assertEqual(command.desired_mode, "LOITER")

    def test_complete_hold_guided_land_lifecycle(self):
        self.executor.start(
            request("HOVER", timeout=1.0, duration_s=.1),
            snapshot(mode="ALT_HOLD"),
        )
        status, command = self.executor.tick(snapshot(.05, mode="ALT_HOLD"))
        self.assertEqual(status.detail, "WAITING_GUIDED_HEARTBEAT")
        self.assertEqual(command.desired_mode, "GUIDED")
        status, _ = self.executor.tick(snapshot(.11, mode="GUIDED"))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertTrue(status.control_retained)

        status = self.executor.start(
            request("LAND", action_id="a2", timeout=5.0),
            snapshot(
                .2,
                mode="GUIDED",
                landing_target_fresh=True,
                landing_target_output_enabled=True,
            ),
        )
        self.assertEqual(status.state, ActionState.RUNNING)
        _, command = self.executor.tick(snapshot(
            .3,
            mode="GUIDED",
            landing_target_fresh=True,
            landing_target_output_enabled=True,
        ))
        self.assertEqual(command.desired_mode, "LAND")
        status, command = self.executor.tick(snapshot(.4, mode="LAND", landed=True))
        self.assertEqual(status.state, ActionState.DONE)
        self.assertEqual(status.reason, "LANDING_DETECTED")
        self.assertIsNone(command.desired_mode)

    def test_cancel_needs_matching_action_id(self):
        self.executor.start(request("HOVER", duration_s=1), snapshot())
        status = self.executor.cancel("other", .1)
        self.assertEqual(status.reason, "REJECTED_ACTION_ID_MISMATCH")
        self.assertTrue(self.executor.active)
        status = self.executor.cancel("a1", .2)
        self.assertEqual(status.state, ActionState.CANCELLED)

    def test_emergency_stop_preempts_and_cannot_be_cancelled(self):
        self.executor.start(request("FOLLOW"), snapshot())
        status = self.executor.start(
            request("EMERGENCY_STOP", action_id="estop", timeout=1),
            snapshot(0.1, telemetry_fresh=False, authorized=False),
        )
        self.assertEqual(status.state, ActionState.RUNNING)
        status, command = self.executor.tick(
            snapshot(.2, telemetry_fresh=False, authorized=False)
        )
        self.assertTrue(command.request_emergency_stop)
        rejected = self.executor.cancel("estop", .3)
        self.assertEqual(rejected.reason, "REJECTED_EMERGENCY_STOP_NOT_CANCELLABLE")
        self.assertTrue(self.executor.active)


if __name__ == "__main__":
    unittest.main()
