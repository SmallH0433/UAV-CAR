import unittest

from air_ground_landing.guided_execution import (
    follow_mode_allowed, ModeTransitionConfig, ModeTransitionManager,
    ModeTransitionPhase,
)


class PilotModeGateTests(unittest.TestCase):
    def test_only_hold_modes_allow_entry(self):
        for mode in ('ALT_HOLD', 'LOITER', 'GUIDED', 'LAND'):
            self.assertTrue(follow_mode_allowed(mode))
        for mode in ('STABILIZE', 'ACRO', 'RTL', 'POSHOLD', 'UNKNOWN', ''):
            self.assertFalse(follow_mode_allowed(mode))

    def test_pilot_override_cancels_pending_request_without_rollback(self):
        manager = ModeTransitionManager(ModeTransitionConfig())
        request = manager.update(now_s=0, current_mode='LOITER', desired_mode='GUIDED')
        self.assertIsNotNone(request)
        manager.release_to_pilot('STABILIZE')
        self.assertIsNone(manager.on_service_result(sequence=request.sequence, mode_sent=True, now_s=1))
        self.assertEqual(manager.phase, ModeTransitionPhase.IDLE)
        self.assertIsNone(manager.target_mode)
        self.assertIsNone(manager.update(now_s=2, current_mode='STABILIZE', desired_mode=None))

    def test_mode_transport_accepts_request_after_separate_session_authorization(self):
        # Operator rearm is enforced by PilotSessionGate in the adapter;
        # this low-level transport has no RC input.
        manager = ModeTransitionManager(ModeTransitionConfig())
        manager.release_to_pilot('STABILIZE')
        request = manager.update(now_s=1, current_mode='ALT_HOLD', desired_mode='GUIDED')
        self.assertEqual(request.mode, 'GUIDED')


if __name__ == '__main__':
    unittest.main()
