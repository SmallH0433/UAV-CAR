"""Regression checks for the merged centre/heading descent gate."""

import math
import unittest

from air_ground_landing.action_execution import ActionExecutor, ActionKind, ActionRequest
from test_guided_land_action import flight


class LandingStabilityMergeTests(unittest.TestCase):
    def start(self) -> ActionExecutor:
        executor = ActionExecutor(
            land_yaw_tolerance_rad=math.radians(25.0),
            land_center_tolerance_px=20.0,
            land_yaw_alignment_dwell_s=0.5,
        )
        executor.start(
            ActionRequest("stable-land", ActionKind.LAND, 20.0, {"rc_managed": True}),
            flight(0.0),
        )
        return executor

    def test_off_centre_pose_holds_vertical_motion(self):
        executor = self.start()
        for now in (0.1, 0.7, 1.2):
            status, command = executor.tick(
                flight(now, landing_center_error_px=20.01)
            )
            self.assertEqual(status.detail, "GUIDED_ALIGN_CENTER")
            self.assertEqual(command.velocity_enu[2], 0.0)

    def test_centre_heading_and_rate_share_one_stable_window(self):
        executor = self.start()
        for now in (0.1, 0.3, 0.59):
            status, command = executor.tick(flight(now))
            self.assertEqual(status.detail, "GUIDED_ALIGNMENT_SETTLING")
            self.assertEqual(command.velocity_enu[2], 0.0)
        status, command = executor.tick(flight(0.61))
        self.assertEqual(status.detail, "GUIDED_TRACK_DESCENT")
        self.assertLess(command.velocity_enu[2], 0.0)

    def test_descent_pauses_and_realigns_after_lateral_drift(self):
        executor = self.start()
        for now in (0.1, 0.3, 0.5):
            executor.tick(flight(now))
        _, command = executor.tick(flight(0.61))
        self.assertLess(command.velocity_enu[2], 0.0)

        status, command = executor.tick(
            flight(0.62, landing_center_error_px=30.0)
        )
        self.assertEqual(status.detail, "GUIDED_ALIGN_CENTER")
        self.assertEqual(command.velocity_enu[2], 0.0)
        self.assertFalse(executor.land_yaw_alignment_complete)

        for now in (0.7, 0.9, 1.1):
            executor.tick(flight(now))
        _, command = executor.tick(flight(1.19))
        self.assertEqual(command.velocity_enu[2], 0.0)
        _, command = executor.tick(flight(1.21))
        self.assertLess(command.velocity_enu[2], 0.0)

    def test_descent_pauses_and_turns_after_heading_drift(self):
        executor = self.start()
        for now in (0.1, 0.3, 0.5, 0.61):
            executor.tick(flight(now))
        status, command = executor.tick(
            flight(0.62, landing_heading_error_rad=math.radians(25.1))
        )
        self.assertEqual(status.detail, "GUIDED_ALIGN_YAW")
        self.assertEqual(command.velocity_enu[2], 0.0)
        self.assertGreater(command.yaw_rate_rad_s, 0.0)

    def test_turning_inside_tolerance_cannot_open_gate(self):
        executor = self.start()
        for now in (0.1, 0.4, 0.8):
            _, command = executor.tick(
                flight(now, yaw_rate_rad_s=math.radians(3.1))
            )
            self.assertEqual(command.velocity_enu[2], 0.0)
        self.assertFalse(executor.land_yaw_alignment_complete)


if __name__ == "__main__":
    unittest.main()
