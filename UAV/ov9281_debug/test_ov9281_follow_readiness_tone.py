#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path

from ov9281_follow_readiness_tone import (
    ReadyToneLatch,
    TelemetryState,
    VisionAcquisition,
    evaluate_operational_readiness,
    validate_config,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "ov9281_follow_readiness_tone_20260817.json"


def valid_status(tag_id: int = 0) -> dict:
    if tag_id == 0:
        size, role = 0.100, "outer"
    else:
        size, role = 0.020, "inner"
    return {
        "mode": "apriltag",
        "found": True,
        "analysis_sequence": 1,
        "frame_age_ms": 20.0,
        "tag_id": tag_id,
        "tag_size_m": size,
        "role": role,
        "area_px2": 200.0,
        "decision_margin": 55.0,
        "hamming": 0,
        "reprojection_error_px": 0.8,
        "distance_m": 0.65,
        "x_m": 0.01,
        "y_m": -0.02,
        "z_m": 0.65,
    }


class ReadinessToneTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    def test_dual_tag_readiness_and_one_shot_tone_have_no_control_permission(self) -> None:
        validate_config(self.config)
        safety = self.config["safety"]
        self.assertTrue(safety["play_tune"])
        for key in (
            "parameter_write",
            "mode_change",
            "movement_setpoint",
            "arm_command",
            "takeoff_command",
            "land_command",
            "motor_command",
        ):
            self.assertFalse(safety[key])

        for tag_id in (0, 1):
            gate = VisionAcquisition(self.config["vision"])
            snapshot = None
            for sequence in range(1, 6):
                status = valid_status(tag_id)
                status["analysis_sequence"] = sequence
                status["hamming"] = 2
                snapshot = gate.update(status, 10.0 + sequence * 0.1)
            self.assertIsNotNone(snapshot)
            self.assertTrue(snapshot.acquired)
            self.assertEqual(snapshot.tag_id, tag_id)

        now_s = 20.0
        state = TelemetryState(
            armed=True,
            mode="ALT_HOLD",
            heartbeat_at_s=now_s,
            rc_pwm=1000,
            rc_at_s=now_s,
            ekf_flags=267,
            ekf_at_s=now_s,
            range_m=0.65,
            range_at_s=now_s,
            flow_quality=120,
            flow_at_s=now_s,
            attitude_at_s=now_s,
            origin_valid=True,
        )
        ready, blockers, _ = evaluate_operational_readiness(
            self.config,
            state,
            snapshot,
            now_s=now_s,
            rc7_low_cycle_seen=True,
        )
        self.assertTrue(ready, blockers)

        latch = ReadyToneLatch(rearm_after_not_ready_s=2.0)
        self.assertTrue(latch.update(True, 20.0))
        self.assertFalse(latch.update(True, 20.1))

    def test_bad_quality_or_control_permission_fails_closed(self) -> None:
        gate = VisionAcquisition(self.config["vision"])
        status = valid_status(1)
        status["reprojection_error_px"] = 3.0
        snapshot = gate.update(status, 1.0)
        self.assertFalse(snapshot.acquired)
        self.assertIn("APRILTAG_REPROJECTION_ERROR_HIGH", snapshot.blockers)

        bounded_inner = VisionAcquisition(self.config["vision"])
        status = valid_status(1)
        status["hamming"] = 3
        snapshot = bounded_inner.update(status, 2.0)
        self.assertFalse(snapshot.acquired)
        self.assertIn("APRILTAG_HAMMING_HIGH", snapshot.blockers)

        unsafe = copy.deepcopy(self.config)
        unsafe["safety"]["mode_change"] = True
        with self.assertRaises(RuntimeError):
            validate_config(unsafe)


if __name__ == "__main__":
    unittest.main()
