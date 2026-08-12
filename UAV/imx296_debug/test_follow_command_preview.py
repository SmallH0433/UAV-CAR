#!/usr/bin/env python3

from __future__ import annotations

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from follow_command_preview import rotate_body_to_ned, update_preview_velocity


class CoordinateTransformTests(unittest.TestCase):
    def test_yaw_zero_preserves_forward_and_right(self):
        north, east = rotate_body_to_ned(0.1, -0.2, 0.0)
        self.assertAlmostEqual(north, 0.1)
        self.assertAlmostEqual(east, -0.2)

    def test_yaw_ninety_maps_forward_to_east(self):
        north, east = rotate_body_to_ned(0.1, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(north, 0.0, places=7)
        self.assertAlmostEqual(east, 0.1)

    def test_rotation_preserves_speed(self):
        north, east = rotate_body_to_ned(0.12, -0.08, 1.1)
        self.assertAlmostEqual(math.hypot(north, east), math.hypot(0.12, -0.08))

    def test_hold_forces_immediate_zero_and_resets_controller(self):
        controller = Mock()
        track = SimpleNamespace(
            position_m=(0.2, -0.1, 0.7),
            velocity_mps=(0.1, 0.1, 0.0),
        )

        velocity = update_preview_velocity(
            controller, "PREVIEW_HOLD", 1.0, track, 0.0
        )

        self.assertEqual(velocity, (0.0, 0.0, 0.0))
        controller.reset.assert_called_once_with()
        controller.update.assert_not_called()

    def test_acquire_forces_immediate_zero_and_resets_controller(self):
        controller = Mock()
        track = SimpleNamespace(
            position_m=(0.2, -0.1, 0.7),
            velocity_mps=(0.1, 0.1, 0.0),
        )

        velocity = update_preview_velocity(controller, "ACQUIRE", 1.0, track, 0.0)

        self.assertEqual(velocity, (0.0, 0.0, 0.0))
        controller.reset.assert_called_once_with()
        controller.update.assert_not_called()

    def test_rc_disabled_forces_immediate_zero_and_resets_controller(self):
        controller = Mock()
        track = SimpleNamespace(
            position_m=(0.2, -0.1, 0.7),
            velocity_mps=(0.1, 0.1, 0.0),
        )

        velocity = update_preview_velocity(
            controller, "RC_DISABLED", 1.0, track, 0.0
        )

        self.assertEqual(velocity, (0.0, 0.0, 0.0))
        controller.reset.assert_called_once_with()
        controller.update.assert_not_called()


if __name__ == "__main__":
    unittest.main()
