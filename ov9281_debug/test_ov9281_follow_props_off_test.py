#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
import unittest
from pathlib import Path

from pymavlink import mavutil

from ov9281_follow_props_off_test import (
    TelemetryState,
    install_pymavlink_instance_guard,
    observation_ready_without_ch7,
    target_echo_matches,
    validate_props_off_config,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "ov9281_follow_props_off_control_20260814.json"
class PropsOffRuntimeGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(CONFIG.read_text(encoding="utf-8"))

    def test_dedicated_config_enables_only_bounded_props_off_control(self):
        self.assertEqual(validate_props_off_config(self.config, 120.0), 120.0)
        self.assertFalse(self.config["operator_start_policy"]["manual_start_only"])
        self.assertTrue(
            self.config["operator_start_policy"]["props_removed_autostart_acknowledged"]
        )
        self.assertEqual(
            self.config["operator_start_policy"]["autostart_scope"],
            "props_removed_only",
        )
        self.assertTrue(self.config["rc_authorization"]["low_at_start_required"])
        self.assertTrue(self.config["safety"]["control_enabled"])
        self.assertTrue(self.config["safety"]["mavlink_transmit"])
        self.assertFalse(self.config["flight_use_approved"])
        self.assertFalse(self.config["safety"]["arm_command"])
        self.assertLessEqual(self.config["controller"]["max_speed_mps"], 0.10)

    def test_config_refuses_flight_approval_but_has_no_cli_token(self):
        approved = copy.deepcopy(self.config)
        approved["flight_use_approved"] = True
        with self.assertRaises(RuntimeError):
            validate_props_off_config(approved, 120.0)
        self.assertNotIn("required_cli_token", self.config["operator_start_policy"])

    def test_autostart_requires_props_removed_ack_and_rc7_low_cycle(self):
        missing_ack = copy.deepcopy(self.config)
        missing_ack["operator_start_policy"]["props_removed_autostart_acknowledged"] = False
        with self.assertRaises(RuntimeError):
            validate_props_off_config(missing_ack, 120.0)

        no_low_cycle = copy.deepcopy(self.config)
        no_low_cycle["rc_authorization"]["low_at_start_required"] = False
        with self.assertRaises(RuntimeError):
            validate_props_off_config(no_low_cycle, 120.0)

    def test_echo_must_arrive_after_and_match_the_sent_target(self):
        state = TelemetryState(
            target_echo_at_s=2.0,
            target_echo_velocity_ned_mps=(0.08, -0.02, 0.0),
        )
        self.assertTrue(
            target_echo_matches(
                state,
                sent_at_s=1.9,
                sent_velocity_ned_mps=(0.08, -0.02, 0.0),
                tolerance_mps=0.01,
            )
        )
        self.assertFalse(
            target_echo_matches(
                state,
                sent_at_s=2.1,
                sent_velocity_ned_mps=(0.08, -0.02, 0.0),
                tolerance_mps=0.01,
            )
        )

    def test_observation_ready_requires_everything_except_ch7(self):
        values = dict(
            armed=True,
            current_mode="ALT_HOLD",
            allowed_entry_modes=("ALT_HOLD", "LOITER", "POSHOLD"),
            non_ch7_prerequisites_ok=True,
        )
        self.assertTrue(observation_ready_without_ch7(**values))
        for field, value in (
            ("armed", False),
            ("current_mode", "STABILIZE"),
            ("non_ch7_prerequisites_ok", False),
        ):
            with self.subTest(field=field):
                changed = dict(values)
                changed[field] = value
                self.assertFalse(observation_ready_without_ch7(**changed))

    def test_pymavlink_instance_guard_repairs_none_instance_cache(self):
        class FakeMessage:
            _instance_field = "sensor_id"
            _instances = None
            sensor_id = 2

        original = mavutil.add_message
        try:
            install_pymavlink_instance_guard()
            messages = {"FAKE": FakeMessage()}
            mavutil.add_message(messages, "FAKE", FakeMessage())
            self.assertIn(2, messages["FAKE"]._instances)
        finally:
            mavutil.add_message = original


if __name__ == "__main__":
    unittest.main()
