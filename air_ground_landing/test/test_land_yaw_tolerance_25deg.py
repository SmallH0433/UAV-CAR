"""Hardware-configured LAND yaw tolerance checks using the real controller."""
import math
import re
import unittest

from air_ground_landing.action_execution import (
    ActionExecutor, ActionKind, ActionRequest, ActionState,
)
from test_guided_land_action import executor_source
from test_land_yaw_continuity import observed


PARAMETER_NAMES = (
    "land_yaw_tolerance_deg", "land_yaw_alignment_dwell_s",
    "land_yaw_alignment_rate_tolerance_deg_s",
    "landing_alignment_maximum_age_s", "land_guided_descent_mps",
    "land_recovery_timeout_s", "land_reacquire_dwell_s",
)


def hardware_parameters():
    """Read unique numeric ROS YAML scalars without another test dependency."""
    path = executor_source().parents[1] / "config/action_executor.hardware.yaml"
    source = path.read_text(encoding="utf-8")
    parameters = {}
    for name in PARAMETER_NAMES:
        values = re.findall(r"(?m)^    " + re.escape(name) +
                            r":[ \t]*([^#\r\n]+)", source)
        if len(values) != 1:
            raise AssertionError(f"Expected one hardware parameter {name} in {path}")
        parameters[name] = float(values[0].strip())
    return parameters


class HardwareYawToleranceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = hardware_parameters()

    def start(self):
        p = self.parameters
        executor = ActionExecutor(
            land_yaw_tolerance_rad=math.radians(p["land_yaw_tolerance_deg"]),
            land_yaw_alignment_dwell_s=p["land_yaw_alignment_dwell_s"],
            land_yaw_alignment_rate_tolerance_rad_s=math.radians(
                p["land_yaw_alignment_rate_tolerance_deg_s"]),
            land_yaw_alignment_maximum_gap_s=p["landing_alignment_maximum_age_s"],
            land_guided_descent_mps=p["land_guided_descent_mps"],
            land_recovery_timeout_s=p["land_recovery_timeout_s"],
            land_reacquire_dwell_s=p["land_reacquire_dwell_s"],
        )
        status = executor.start(ActionRequest("yaw-25-hardware", ActionKind.LAND,
            180, {"rc_managed": True}), observed(0))
        self.assertEqual(status.state, ActionState.RUNNING)
        return executor

    def tick(self, executor, now, heading_deg=20, **changes):
        return executor.tick(observed(now,
            landing_heading_error_rad=math.radians(heading_deg), **changes))

    def assert_descent(self, command, *, settled=False):
        self.assertIsNotNone(command)
        self.assertEqual(command.desired_mode, "GUIDED")
        self.assertLess(command.velocity_enu[2], 0)
        self.assertGreaterEqual(command.velocity_enu[2],
                                -self.parameters["land_guided_descent_mps"] - 1e-9)
        if settled:
            self.assertAlmostEqual(command.velocity_enu[2],
                                   -self.parameters["land_guided_descent_mps"])
        self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assertFalse(command.request_disarm)

    def test_hardware_profile_uses_25_degrees_and_preserves_stability_guards(self):
        self.assertEqual(self.parameters["land_yaw_tolerance_deg"], 25.0)
        self.assertEqual(self.parameters["land_yaw_alignment_dwell_s"], .5)
        self.assertEqual(self.parameters["land_yaw_alignment_rate_tolerance_deg_s"], 3)
        self.assertEqual(self.parameters["landing_alignment_maximum_age_s"], .3)
        self.assertEqual(self.parameters["land_guided_descent_mps"], .05)

    def test_both_25_degree_boundaries_allow_descent_after_stability_dwell(self):
        tolerance = self.parameters["land_yaw_tolerance_deg"]
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                executor = self.start()
                for t in (.1, .2, .3, .4, .5, .59):
                    _, command = self.tick(executor, t, sign * tolerance)
                    self.assertEqual(command.velocity_enu[2], 0)
                    self.assertEqual(command.yaw_rate_rad_s, 0)
                self.assert_descent(self.tick(executor, .61, sign * tolerance)[1])
                self.assert_descent(self.tick(executor, .81, sign * tolerance)[1],
                                    settled=True)

    def test_both_25_point_1_degree_errors_keep_height_and_turn(self):
        error = self.parameters["land_yaw_tolerance_deg"] + .1
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                executor = self.start()
                for t in (.1, .3, .5, .7, .9, 1.1):
                    _, command = self.tick(executor, t, sign * error)
                    self.assertEqual(command.velocity_enu[2], 0)
                    self.assertGreater(sign * command.yaw_rate_rad_s, 0)

    def test_20_degree_error_descends_after_half_second_of_fresh_stability(self):
        executor = self.start()
        for t in (.1, .2, .3, .4, .5):
            _, command = self.tick(executor, t, 20)
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertEqual(command.yaw_rate_rad_s, 0)
        self.assert_descent(self.tick(executor, .61, 20)[1])
        self.assert_descent(self.tick(executor, .81, 20)[1], settled=True)

    def test_20_degree_error_does_not_shortcut_half_second_dwell(self):
        executor = self.start()
        for t in (.1, .2, .3, .4, .5, .59):
            _, command = self.tick(executor, t, 20)
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertEqual(command.yaw_rate_rad_s, 0)

    def test_angular_speed_over_three_degrees_per_second_prevents_descent(self):
        rate = self.parameters["land_yaw_alignment_rate_tolerance_deg_s"] + .1
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                executor = self.start()
                for t in (.1, .3, .5, .7, .9, 1.1):
                    _, command = self.tick(executor, t, 20,
                        yaw_rate_rad_s=math.radians(sign * rate))
                    self.assertEqual(command.velocity_enu[2], 0)

    def test_angular_speed_at_three_degree_boundary_can_complete_stability(self):
        rate = self.parameters["land_yaw_alignment_rate_tolerance_deg_s"]
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                executor = self.start()
                for t in (.1, .2, .3, .4, .5):
                    _, command = self.tick(executor, t, 20,
                        yaw_rate_rad_s=math.radians(sign * rate))
                    self.assertEqual(command.velocity_enu[2], 0)
                self.assert_descent(self.tick(executor, .61, 20,
                    yaw_rate_rad_s=math.radians(sign * rate))[1])

    def test_angular_speed_spike_restarts_stability_window(self):
        executor = self.start()
        for t in (.1, .3, .5):
            self.tick(executor, t, 20)
        rate = self.parameters["land_yaw_alignment_rate_tolerance_deg_s"] + .1
        _, command = self.tick(executor, .55, 20,
                              yaw_rate_rad_s=math.radians(rate))
        self.assertEqual(command.velocity_enu[2], 0)
        for t in (.6, .8, 1.0, 1.09):
            _, command = self.tick(executor, t, 20)
            self.assertEqual(command.velocity_enu[2], 0)
        self.assert_descent(self.tick(executor, 1.11, 20)[1])

    def test_missing_fresh_candidate_or_alignment_prevents_descent(self):
        for unavailable in ({"candidate_fresh": False},
                            {"landing_alignment_fresh": False}):
            with self.subTest(unavailable=unavailable):
                executor = self.start()
                for t in (.1, .3, .5, .7, .9, 1.1):
                    _, command = self.tick(executor, t, 20, **unavailable)
                    # The established same-height search may move horizontally.
                    self.assertEqual(command.velocity_enu[2], 0)
                    self.assertEqual(command.yaw_rate_rad_s, 0)

    def test_repeated_observation_cannot_complete_stability_dwell(self):
        executor = self.start()
        for t in (.1, .2, .3, .39):
            _, command = self.tick(executor, t, 20, observation=.1)
            self.assertEqual(command.velocity_enu[2], 0)
        _, command = self.tick(executor, .7, 20, observation=.1,
                              landing_alignment_fresh=False)
        self.assertEqual(command.velocity_enu, (0, 0, 0))
        self.assertEqual(command.yaw_rate_rad_s, 0)

    def test_observation_gap_requires_new_half_second_stability_window(self):
        executor = self.start()
        self.tick(executor, .1, 20)
        self.tick(executor, .3, 20)
        self.tick(executor, .35, 20, candidate_fresh=False,
                  landing_alignment_fresh=False)
        for t in (.7, .9, 1.1):
            _, command = self.tick(executor, t, 20)
            self.assertEqual(command.velocity_enu[2], 0)
        self.assert_descent(self.tick(executor, 1.21, 20)[1])

    def test_after_descent_large_heading_errors_pause_and_realign(self):
        executor = self.start()
        for t in (.1, .2, .3, .4, .5, .61, .81):
            self.tick(executor, t, 20)
        for t, error in ((1.0, 65), (1.2, -65), (1.4, 25.1)):
            _, command = self.tick(executor, t, error)
            self.assertEqual(command.velocity_enu[2], 0)
            self.assertNotEqual(command.yaw_rate_rad_s, 0)


if __name__ == "__main__":
    unittest.main()
