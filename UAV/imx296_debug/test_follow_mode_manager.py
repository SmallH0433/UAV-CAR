#!/usr/bin/env python3

from __future__ import annotations

import unittest

from follow_mode_manager import (
    FollowModeManager,
    ModeManagerInputs,
    ModeManagerState,
    PilotStickOverrideDetector,
)


class FollowModeManagerTests(unittest.TestCase):
    def inputs(self, timestamp_s: float, **changes) -> ModeManagerInputs:
        values = dict(
            timestamp_s=timestamp_s,
            armed=True,
            current_mode="ALT_HOLD",
            rc_enable=True,
            prerequisites_ok=True,
            pilot_stick_override=False,
        )
        values.update(changes)
        return ModeManagerInputs(**values)

    def enter_guided(self, manager: FollowModeManager) -> None:
        first = manager.update(self.inputs(0.0))
        self.assertEqual(first.request_mode, "GUIDED")
        for index in range(3):
            decision = manager.update(
                self.inputs(0.1 + index * 0.1, current_mode="GUIDED")
            )
        self.assertEqual(decision.state, ModeManagerState.ACTIVE)
        self.assertTrue(decision.allow_follow_velocity)

    def test_ch7_requests_and_confirms_guided(self):
        manager = FollowModeManager()
        self.enter_guided(manager)

    def test_ch7_low_stops_and_restores_entry_mode(self):
        manager = FollowModeManager()
        self.enter_guided(manager)
        decision = manager.update(
            self.inputs(0.5, current_mode="GUIDED", rc_enable=False)
        )
        self.assertEqual(decision.state, ModeManagerState.RESTORE_MODE)
        self.assertEqual(decision.request_mode, "ALT_HOLD")
        self.assertTrue(decision.send_zero_velocity)
        self.assertFalse(decision.allow_follow_velocity)

    def test_pilot_mode_switch_wins_and_latches(self):
        manager = FollowModeManager()
        self.enter_guided(manager)
        decision = manager.update(self.inputs(0.5, current_mode="LOITER"))
        self.assertEqual(decision.state, ModeManagerState.PILOT_OVERRIDE_LOCKOUT)
        self.assertIsNone(decision.request_mode)
        decision = manager.update(self.inputs(2.0, current_mode="LOITER"))
        self.assertTrue(decision.lockout)
        self.assertIsNone(decision.request_mode)
        manager.update(self.inputs(2.1, current_mode="LOITER", rc_enable=False))
        decision = manager.update(self.inputs(2.2, current_mode="LOITER"))
        self.assertEqual(decision.request_mode, "GUIDED")

    def test_stick_override_stops_restores_and_latches(self):
        manager = FollowModeManager()
        self.enter_guided(manager)
        decision = manager.update(
            self.inputs(0.5, current_mode="GUIDED", pilot_stick_override=True)
        )
        self.assertEqual(decision.state, ModeManagerState.PILOT_OVERRIDE_LOCKOUT)
        self.assertEqual(decision.request_mode, "ALT_HOLD")
        self.assertTrue(decision.send_zero_velocity)
        self.assertTrue(decision.lockout)

    def test_failed_guided_entry_never_allows_velocity(self):
        manager = FollowModeManager(mode_request_timeout_s=1.0)
        manager.update(self.inputs(0.0))
        decision = manager.update(self.inputs(1.01))
        self.assertEqual(decision.state, ModeManagerState.FAULT_LOCKOUT)
        self.assertFalse(decision.allow_follow_velocity)

    def test_disarmed_or_bad_prerequisites_never_changes_mode(self):
        manager = FollowModeManager()
        decision = manager.update(self.inputs(0.0, armed=False))
        self.assertIsNone(decision.request_mode)
        decision = manager.update(self.inputs(0.1, prerequisites_ok=False))
        self.assertIsNone(decision.request_mode)

    def test_unapproved_entry_mode_fails_closed(self):
        manager = FollowModeManager()
        decision = manager.update(self.inputs(0.0, current_mode="STABILIZE"))
        self.assertEqual(decision.state, ModeManagerState.FAULT_LOCKOUT)
        self.assertIsNone(decision.request_mode)

    def test_props_off_manager_rejects_preexisting_unowned_guided(self):
        manager = FollowModeManager(allow_preexisting_guided=False)
        decision = manager.update(self.inputs(0.0, current_mode="GUIDED"))
        self.assertEqual(decision.state, ModeManagerState.FAULT_LOCKOUT)
        self.assertEqual(decision.reason, "PREEXISTING_GUIDED_NOT_OWNED")
        self.assertFalse(decision.allow_follow_velocity)


class PilotStickOverrideDetectorTests(unittest.TestCase):
    def test_requires_sustained_deflection(self):
        detector = PilotStickOverrideDetector(threshold_pwm=150, debounce_s=0.2)
        self.assertFalse(detector.update({1: 1700, 2: 1500, 4: 1500}, 1.0))
        self.assertFalse(detector.update({1: 1700, 2: 1500, 4: 1500}, 1.19))
        self.assertTrue(detector.update({1: 1700, 2: 1500, 4: 1500}, 1.20))

    def test_centred_sticks_clear_pending_override(self):
        detector = PilotStickOverrideDetector()
        detector.update({1: 1700, 2: 1500, 4: 1500}, 1.0)
        self.assertFalse(detector.update({1: 1500, 2: 1500, 4: 1500}, 1.1))
        self.assertFalse(detector.update({1: 1700, 2: 1500, 4: 1500}, 1.2))


if __name__ == "__main__":
    unittest.main()
