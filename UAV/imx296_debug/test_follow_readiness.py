#!/usr/bin/env python3

from __future__ import annotations

import unittest

from follow_readiness import ReadinessInputs, evaluate_readiness


class ReadinessTests(unittest.TestCase):
    def healthy(self, **changes) -> ReadinessInputs:
        values = dict(
            heartbeat_age_s=0.1, armed=True, mode="LOITER", rc7_pwm=2000, rc_age_s=0.1,
            ekf_flags=367, ekf_age_s=0.1, battery_voltage_v=23.0,
            battery_remaining_pct=90, battery_age_s=0.1, range_m=0.72,
            range_age_s=0.1, flow_quality=105, flow_age_s=0.1,
            origin_valid=True, target_acquired=True, target_age_s=0.05, camera_ok=True,
        )
        values.update(changes)
        return ReadinessInputs(**values)

    def test_all_conditions_ready(self):
        self.assertTrue(evaluate_readiness(self.healthy()).ready_for_follow_request)

    def test_origin_is_mandatory(self):
        result = evaluate_readiness(self.healthy(origin_valid=False))
        self.assertIn("EKF_GLOBAL_ORIGIN_MISSING", result.blockers)

    def test_tag_loss_is_fail_closed(self):
        result = evaluate_readiness(self.healthy(target_acquired=False, target_age_s=1.0))
        self.assertFalse(result.ready_for_follow_request)
        self.assertIn("APRILTAG_NOT_ACQUIRED", result.blockers)

    def test_height_gate(self):
        for height in (0.2, 1.0):
            with self.subTest(height=height):
                self.assertIn("HEIGHT_OUTSIDE_FOLLOW_GATE",
                              evaluate_readiness(self.healthy(range_m=height)).blockers)

    def test_stale_telemetry_and_low_flow_fail(self):
        result = evaluate_readiness(self.healthy(heartbeat_age_s=2.0, flow_quality=20))
        self.assertIn("FLIGHT_CONTROLLER_HEARTBEAT_STALE", result.blockers)
        self.assertIn("OPTICAL_FLOW_QUALITY_LOW", result.blockers)

    def test_stabilize_is_not_an_approved_entry_mode(self):
        result = evaluate_readiness(self.healthy(mode="STABILIZE"))
        self.assertIn("ENTRY_MODE_NOT_APPROVED", result.blockers)

    def test_props_off_entry_modes_can_be_explicitly_approved(self):
        result = evaluate_readiness(
            self.healthy(mode="ALT_HOLD"),
            allowed_modes=("ALT_HOLD", "LOITER", "POSHOLD", "GUIDED"),
        )
        self.assertTrue(result.ready_for_follow_request)

    def test_battery_gate_can_be_disabled_for_props_off_test(self):
        result = evaluate_readiness(
            self.healthy(
                battery_voltage_v=None,
                battery_remaining_pct=0,
                battery_age_s=None,
            ),
            battery_telemetry_required=False,
        )
        self.assertTrue(result.ready_for_follow_request)
        self.assertFalse(any(item.startswith("BATTERY_") for item in result.blockers))
        self.assertIn("BATTERY_CHECK_DISABLED", result.warnings)


if __name__ == "__main__":
    unittest.main()
