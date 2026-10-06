"""Exercise real adapter methods and frame gates without importing ROS."""
import ast
from dataclasses import replace
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from air_ground_landing.hybrid_guidance import IbvsConfig, IbvsFeatureController
from air_ground_landing.landing_alignment import LandingAlignment, alignment_payload
from air_ground_landing.landing_target_bridge import BridgeConfig, LandingTargetBridge
from test_guided_land_action import executor_source, flight
from test_moving_landing_stack import load_config, vision_status
from air_ground_landing.action_execution import ActionExecutor, ActionRequest, ActionKind


class Target:
    FRAME_BODY_NED = 8
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048

    def __init__(self):
        self.header = SimpleNamespace()
        self.velocity = SimpleNamespace()


def methods(path, names, scope):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    body = [n for c in tree.body if isinstance(c, ast.ClassDef)
            for n in c.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(body) == len(names)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), "exec"), scope)
    return scope


class DuplicateFrameTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.0
        self.sample = vision_status()
        self.sample.update(corners_px=[[630, 390], [650, 390], [650, 410], [630, 410]],
            orientation={"valid": True, "frame": "BODY_FRD", "source": "PNP_TAG_TO_PAD_TO_BODY",
                "rotation_camera_optical_to_body_frd": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                "rotation_pad_to_camera": [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],
                "quaternion_pad_to_body_frd_wxyz": [1, 0, 0, 0]})
        self.error = None
        self.candidates, self.statuses = [], []
        self.receiver = SimpleNamespace(
            _now_s=lambda: self.now,
            lifecycle=SimpleNamespace(active=False),
        )
        receiver_method = methods(executor_source(), {"_target_status"},
            dict(String=object, json=json, LandingAlignment=LandingAlignment))["_target_status"]

        def publish_status(message):
            self.statuses.append(json.loads(message.data))
            receiver_method(self.receiver, message)

        def open_status(*args, **kwargs):
            if self.error:
                raise self.error
            return io.BytesIO(json.dumps(self.sample).encode())

        path = executor_source().with_name("ibvs_adapter.py")
        scope = methods(path, {"_tick", "_status"}, dict(
            json=json, math=math, PositionTarget=Target, String=SimpleNamespace,
            time=SimpleNamespace(monotonic=lambda: self.now, time_ns=lambda: int(self.now * 1e9)),
            urllib=SimpleNamespace(request=SimpleNamespace(urlopen=open_status)),
            alignment_payload=alignment_payload))
        Adapter = type("Adapter", (), {name: scope[name] for name in ("_tick", "_status")})
        self.adapter = Adapter()
        config = load_config()
        self.adapter.bridge = LandingTargetBridge(BridgeConfig.from_mapping(config))
        camera = IbvsConfig.from_mapping(config)
        self.adapter.controller = IbvsFeatureController(camera)
        self.adapter.landing_controller = IbvsFeatureController(replace(camera, cx_px=640, cy_px=400))
        self.adapter.last_valid_capture_s = None
        self.adapter.status_url, self.adapter.http_timeout_s = "http://test", .15
        self.adapter.publisher = SimpleNamespace(publish=self.candidates.append)
        self.adapter.status_publisher = SimpleNamespace(publish=publish_status)
        self.adapter.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: self.now))

    def poll(self, now, **changes):
        self.now = now
        self.sample.update(changes)
        self.adapter._tick()

    def test_fresh_duplicate_preserves_alignment_without_renewing_timestamps(self):
        self.poll(1)
        self.assertTrue(self.receiver.target_healthy)
        evidence = self.receiver.landing_alignment
        self.assertIsNotNone(evidence)
        capture_s = self.adapter.last_valid_capture_s
        for t in (1.04, 1.08, 1.12):
            self.poll(t)
            self.assertTrue(self.receiver.target_healthy)
            self.assertIs(self.receiver.landing_alignment, evidence)
            self.assertEqual(self.receiver.target_status_received_s, 1)
            self.assertEqual(self.adapter.last_valid_capture_s, capture_s)
        self.assertEqual(len(self.candidates), 1)
        self.assertEqual(len(self.statuses), 1)
        self.assertEqual(self.candidates[0].header.stamp, 1)

    def test_frozen_sequence_expires_even_if_api_reports_young_frame(self):
        self.poll(1)
        self.poll(1.14, frame_age_ms=0)
        self.assertFalse(self.receiver.target_healthy)
        self.assertIsNone(self.receiver.landing_alignment)
        self.assertIsNone(self.adapter.last_valid_capture_s)
        self.assertEqual(self.statuses[-1]["reason"], "BRIDGE_REJECTED:STALE_FRAME")
        self.assertEqual(len(self.candidates), 1)

    def test_duplicate_after_invalid_frame_cannot_revive_previous_evidence(self):
        self.poll(1)
        self.poll(1.04, analysis_sequence=2, decision_margin=0)
        self.assertFalse(self.receiver.target_healthy)
        self.poll(1.05, decision_margin=60)
        self.assertFalse(self.receiver.target_healthy)
        self.assertIsNone(self.receiver.landing_alignment)
        self.assertEqual(len(self.candidates), 1)

    def test_actual_loss_on_same_sequence_clears_evidence_immediately(self):
        self.poll(1)
        self.poll(1.04, found=False)
        self.assertFalse(self.receiver.target_healthy)
        self.assertIsNone(self.receiver.landing_alignment)
        self.assertEqual(self.statuses[-1]["reason"], "BRIDGE_REJECTED:TARGET_NOT_FOUND")

    def test_http_error_clears_cache_and_duplicate_cannot_revive_it(self):
        self.poll(1)
        self.error = TimeoutError()
        self.poll(1.04)
        self.assertFalse(self.receiver.target_healthy)
        self.error = None
        self.poll(1.05)
        self.assertFalse(self.receiver.target_healthy)
        self.assertIsNone(self.receiver.landing_alignment)
        self.assertEqual(len(self.candidates), 1)

    def test_new_valid_frame_recovers_after_expiry(self):
        self.poll(1)
        self.poll(1.14)
        self.poll(1.2, analysis_sequence=2)
        self.assertTrue(self.receiver.target_healthy)
        self.assertIsNotNone(self.receiver.landing_alignment)
        self.assertEqual(len(self.candidates), 2)
        self.assertEqual(self.receiver.target_status_received_s, 1.2)

    def test_clock_reversal_revokes_duplicate_evidence(self):
        self.poll(1)
        self.poll(.9)
        self.assertFalse(self.receiver.target_healthy)
        self.assertIsNone(self.receiver.landing_alignment)

    def test_duplicate_uses_controller_age_limit_as_well_as_bridge_limit(self):
        self.adapter.controller.config = replace(self.adapter.controller.config, maximum_feature_age_s=.06)
        self.poll(1)
        self.poll(1.05)
        self.assertFalse(self.receiver.target_healthy)

    def test_land_alignment_dwell_survives_duplicate_polls(self):
        self.poll(1)
        executor = ActionExecutor()
        executor.start(ActionRequest("align", ActionKind.LAND, 5, {}), flight(1))
        for t, sequence in ((1, 1), (1.04, 1), (1.1, 2), (1.14, 2),
                            (1.2, 3), (1.3, 4), (1.34, 4), (1.41, 5),
                            (1.51, 6)):
            self.poll(t, analysis_sequence=sequence)
            evidence = self.receiver.landing_alignment
            status, command = executor.tick(flight(t,
                candidate_fresh=self.receiver.target_healthy,
                landing_alignment_fresh=evidence is not None and evidence.fresh(t, .3),
                landing_velocity_flu=evidence.velocity_flu if evidence else None,
                landing_center_error_px=evidence.center_error_px if evidence else math.nan,
                landing_heading_error_rad=evidence.heading_error_rad if evidence else math.nan))
            if t < 1.5:
                self.assertEqual(command.velocity_enu[2], 0)
                self.assertEqual(status.detail, "GUIDED_ALIGNMENT_SETTLING")
        self.assertEqual(status.detail, "GUIDED_TRACK_DESCENT")
        self.assertLess(command.velocity_enu[2], 0)


if __name__ == "__main__":
    unittest.main()
