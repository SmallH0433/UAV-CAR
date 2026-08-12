#!/usr/bin/env python3

from __future__ import annotations

import unittest
from types import SimpleNamespace

from rc_follow_gate import RcFollowGate


class RcFollowGateTests(unittest.TestCase):
    def test_measured_switch_positions(self):
        gate = RcFollowGate(channel=7)
        self.assertFalse(gate.update(1000, 1.0).enabled)
        self.assertTrue(gate.update(2000, 2.0).enabled)

    def test_ambiguous_pwm_fails_closed(self):
        gate = RcFollowGate(channel=7)
        status = gate.update(1500, 1.0)
        self.assertFalse(status.enabled)
        self.assertEqual(status.reason, "RC_PWM_AMBIGUOUS")

    def test_stale_sample_fails_closed(self):
        gate = RcFollowGate(channel=7, timeout_s=0.5)
        gate.update(2000, 1.0)
        status = gate.status(1.51)
        self.assertFalse(status.enabled)
        self.assertEqual(status.reason, "RC_SAMPLE_TIMEOUT")

    def test_extracts_channel_seven(self):
        gate = RcFollowGate(channel=7)
        message = SimpleNamespace(chan7_raw=2000)
        self.assertTrue(gate.update_from_rc_channels(message, 1.0).enabled)


if __name__ == "__main__":
    unittest.main()
