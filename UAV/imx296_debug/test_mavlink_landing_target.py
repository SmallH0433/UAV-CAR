#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from mavlink_landing_target import (
    MAV_FRAME_BODY_FRD,
    load_body_extrinsics,
    observation_to_packet,
)


HERE = Path(__file__).resolve().parent
EXTRINSICS = HERE / "imx296_body_extrinsics_20260806.json"


class BodyExtrinsicsTests(unittest.TestCase):
    def test_nominal_axis_mapping(self):
        transform = load_body_extrinsics(EXTRINSICS)
        self.assertEqual(transform.transform(1.0, 0.0, 2.0), (-1.0, 0.0, 2.0))
        self.assertEqual(transform.transform(0.0, 1.0, 2.0), (0.0, -1.0, 2.0))
        self.assertEqual(transform.transform(0.0, 0.0, 2.0), (0.0, 0.0, 2.0))

    def test_zero_translation(self):
        transform = load_body_extrinsics(EXTRINSICS)
        self.assertEqual(transform.translation_camera_in_body_m, (0.0, 0.0, 0.0))

    def test_body_packet(self):
        transform = load_body_extrinsics(EXTRINSICS)
        x_m, y_m, z_m = transform.transform(0.1, -0.2, 1.0)
        observation = SimpleNamespace(valid=True, x_m=x_m, y_m=y_m, z_m=z_m)
        packet = observation_to_packet(
            observation,
            frame=MAV_FRAME_BODY_FRD,
            position_valid=1,
        )
        self.assertEqual(packet.frame, MAV_FRAME_BODY_FRD)
        self.assertEqual(packet.position_valid, 1)
        self.assertAlmostEqual(packet.x, -0.1)
        self.assertAlmostEqual(packet.y, 0.2)
        self.assertAlmostEqual(packet.z, 1.0)

    def test_rejects_non_rotation(self):
        data = json.loads(EXTRINSICS.read_text(encoding="utf-8"))
        data["rotation_camera_optical_to_body_frd"][0][0] = 2.0
        with tempfile.TemporaryDirectory() as directory:
            bad_path = Path(directory) / "bad.json"
            bad_path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_body_extrinsics(bad_path)


if __name__ == "__main__":
    unittest.main()
