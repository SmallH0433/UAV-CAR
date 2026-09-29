"""The console shows only actual unmasked outgoing yaw commands."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


class ConsoleYawDirectionTests(unittest.TestCase):
    def test_yaw_rate_reads_outgoing_setpoint_mask(self):
        path = (Path(__file__).resolve().parents[1]
                / "ros2_ws/src/air_ground_landing_ros2/air_ground_landing_ros2/flight_status_http.py")
        tree = ast.parse(path.read_text(encoding="utf-8"))
        method = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == "command_yaw_rate")
        namespace = {"math": math, "Optional": __import__("typing").Optional,
                     "PositionTarget": SimpleNamespace(IGNORE_YAW_RATE=1 << 11)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
        read = namespace["command_yaw_rate"]
        self.assertAlmostEqual(read({"type_mask": 0, "yaw_rate": -.2}), -.2)
        self.assertIsNone(read({"type_mask": 1 << 11, "yaw_rate": -.2}))
        self.assertIsNone(read({"type_mask": 0, "yaw_rate": float("nan")}))
        self.assertIsNone(read({"type_mask": 0, "yaw_rate": None}))


if __name__ == "__main__":
    unittest.main()
