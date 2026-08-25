#!/usr/bin/env python3

from __future__ import annotations

import unittest

from follow_controller import HorizontalFollowController
from follow_replay import simulate
from follow_state_machine import FollowInputs, FollowSafetyStateMachine, FollowState
from mavlink_guided_velocity import (
    GuidedVelocitySetpoint,
    VELOCITY_AND_YAW_RATE_TYPE_MASK,
    make_message,
    pack_message,
)
from target_tracker import AlphaBetaTargetTracker, TargetMeasurement


class TrackerTests(unittest.TestCase):
    def test_acquires_after_five_consistent_measurements(self):
        tracker = AlphaBetaTargetTracker(acquire_count=5)
        track = None
        for index in range(5):
            track = tracker.update(
                TargetMeasurement(index * 0.1, (index * 0.01, 0.0, 0.8))
            )
        self.assertIsNotNone(track)
        self.assertTrue(track.acquired)
        self.assertTrue(track.accepted)

    def test_rejects_large_jump(self):
        tracker = AlphaBetaTargetTracker(max_residual_m=0.25)
        tracker.update(TargetMeasurement(0.0, (0.0, 0.0, 0.8)))
        track = tracker.update(TargetMeasurement(0.1, (1.0, 0.0, 0.8)))
        self.assertFalse(track.accepted)
        self.assertEqual(track.rejection_reason, "RESIDUAL_LIMIT")


class ControllerTests(unittest.TestCase):
    def test_direction_and_speed_limit(self):
        controller = HorizontalFollowController(max_speed_mps=0.2)
        command = controller.update(
            timestamp_s=0.0,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(1.0, -1.0),
        )
        self.assertGreater(command.velocity_ned_mps[0], 0.0)
        self.assertLess(command.velocity_ned_mps[1], 0.0)
        speed = (command.velocity_ned_mps[0] ** 2 + command.velocity_ned_mps[1] ** 2) ** 0.5
        self.assertLessEqual(speed, 0.2 + 1e-9)

    def test_acceleration_limit(self):
        controller = HorizontalFollowController(max_speed_mps=1.0, max_accel_mps2=0.2)
        controller.update(
            timestamp_s=0.0,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(0.0, 0.0),
        )
        command = controller.update(
            timestamp_s=0.1,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(1.0, 0.0),
        )
        self.assertAlmostEqual(command.velocity_ned_mps[0], 0.02, places=6)

    def test_one_mps_candidate_limit(self):
        controller = HorizontalFollowController(
            kp_xy=1.0, max_speed_mps=1.0, max_accel_mps2=5.0
        )
        command = controller.update(
            timestamp_s=0.0,
            vehicle_position_ned_m=(0.0, 0.0),
            target_position_ned_m=(10.0, 10.0),
        )
        speed = (command.velocity_ned_mps[0] ** 2 + command.velocity_ned_mps[1] ** 2) ** 0.5
        self.assertAlmostEqual(speed, 1.0, places=9)


class StateMachineTests(unittest.TestCase):
    def base_inputs(self, **changes):
        values = dict(
            timestamp_s=1.0,
            armed=True,
            mode="GUIDED",
            rc_enable=True,
            ekf_position_ok=True,
            battery_ok=True,
            altitude_ok=True,
            target_acquired=True,
            target_age_s=0.0,
        )
        values.update(changes)
        return FollowInputs(**values)

    def test_fresh_target_allows_follow(self):
        machine = FollowSafetyStateMachine()
        decision = machine.update(self.base_inputs())
        self.assertEqual(decision.state, FollowState.FOLLOW_XY)
        self.assertTrue(decision.may_send_velocity)
        self.assertEqual(decision.velocity_scale, 1.0)

    def test_target_loss_latches_hold_until_rc_cycle(self):
        machine = FollowSafetyStateMachine()
        decision = machine.update(self.base_inputs(target_age_s=0.8))
        self.assertEqual(decision.state, FollowState.HOLD)
        decision = machine.update(self.base_inputs(target_age_s=0.0))
        self.assertEqual(decision.state, FollowState.HOLD)
        machine.update(self.base_inputs(rc_enable=False))
        decision = machine.update(self.base_inputs(target_age_s=0.0))
        self.assertEqual(decision.state, FollowState.FOLLOW_XY)

    def test_mode_change_gives_pilot_override(self):
        machine = FollowSafetyStateMachine()
        decision = machine.update(self.base_inputs(mode="LOITER"))
        self.assertEqual(decision.state, FollowState.PILOT_OVERRIDE)
        self.assertFalse(decision.may_send_velocity)


class MavlinkEncodingTests(unittest.TestCase):
    def test_velocity_packet(self):
        setpoint = GuidedVelocitySetpoint(1000, 0.1, -0.1, 0.0, 0.0)
        message = make_message(setpoint, max_speed_mps=0.2)
        self.assertEqual(message.type_mask, VELOCITY_AND_YAW_RATE_TYPE_MASK)
        self.assertAlmostEqual(message.vx, 0.1)
        self.assertAlmostEqual(message.vy, -0.1)
        self.assertGreater(len(pack_message(setpoint, max_speed_mps=0.2)), 20)

    def test_rejects_overspeed(self):
        with self.assertRaises(ValueError):
            make_message(GuidedVelocitySetpoint(1000, 0.3, 0.0), max_speed_mps=0.2)

    def test_one_mps_candidate_encoder_boundary(self):
        accepted = make_message(
            GuidedVelocitySetpoint(1000, 0.8, 0.6), max_speed_mps=1.0
        )
        self.assertAlmostEqual(accepted.vx, 0.8)
        self.assertAlmostEqual(accepted.vy, 0.6)
        with self.assertRaises(ValueError):
            make_message(
                GuidedVelocitySetpoint(1000, 1.001, 0.0), max_speed_mps=1.0
            )


class ReplayTests(unittest.TestCase):
    def test_dynamic_replay_stays_bounded_and_holds_on_loss(self):
        records, summary = simulate(10.0, 10.0)
        self.assertEqual(len(records), 101)
        self.assertLessEqual(summary["max_command_speed_mps"], 0.2 + 1e-9)
        self.assertTrue(summary["hold_observed"])
        self.assertTrue(summary["rc_reenable_observed"])
        self.assertFalse(summary["real_mavlink_sent"])


if __name__ == "__main__":
    unittest.main()
