"""Exercise the production HTTP handler without camera or ROS dependencies."""
import ast
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import urllib.error
import urllib.request


SOURCE = Path(__file__).with_name("ov9281_unified_service.py")


class VisionEndpointTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
        handler = next(node for node in tree.body
                       if isinstance(node, ast.ClassDef) and node.name == "Handler")
        self.flight_urlopen = Mock(side_effect=TimeoutError("flight service unavailable"))
        namespace = {
            "BaseHTTPRequestHandler": BaseHTTPRequestHandler,
            "VisionState": object,
            "json": json,
            "urllib": SimpleNamespace(request=SimpleNamespace(urlopen=self.flight_urlopen),
                                      error=urllib.error),
            "HTML": b"console",
        }
        exec(compile(ast.Module(body=[handler], type_ignores=[]), str(SOURCE), "exec"),
             namespace)
        self.handler = namespace["Handler"]
        self.vision = {
            "sensor": "ov9281", "mode": "apriltag", "found": True,
            "analysis_sequence": 42, "frame_age_ms": 31.0,
            "tag_id": 0, "tag_size_m": 0.1,
            "x_m": 0.03, "y_m": -0.02, "z_m": 1.0,
            "orientation": {"valid": True, "frame": "BODY_FRD"},
            "detections": [{"tag_id": 0, "quality_passed": True}],
            "capture_monotonic_s": 100.125,
            "capture_analysis_sequence": 42,
            "capture_sensor_timestamp_ns": 100130000000,
            "capture_timestamp_source": "libcamera_sensor_timestamp",
            "capture_timing_valid": True,
            "capture_clock_id": "CLOCK_MONOTONIC",
        }
        self.handler.state = SimpleNamespace(
            status=lambda: dict(self.vision),
            args=SimpleNamespace(flight_status_url="http://flight.invalid/api/status",
                                 flight_status_timeout_s=0.15),
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), self.handler)
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)

    def get(self, path, timeout=1):
        with urllib.request.urlopen(self.base + path, timeout=timeout) as response:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.headers["Cache-Control"], "no-store")
            return json.loads(response.read())

    def assert_vision_unchanged(self, result):
        for key, value in self.vision.items():
            self.assertEqual(result[key], value, key)
        self.assertIs(result["camera_owns_mavlink"], False)
        self.assertIs(result["flight_controller_connected"], False)

    def test_vision_endpoint_does_not_read_failed_flight_service(self):
        result = self.get("/api/vision/status?poll=1")
        self.assert_vision_unchanged(result)
        self.assertNotIn("flight", result)
        self.assertNotIn("flight_telemetry_connected", result)
        self.flight_urlopen.assert_not_called()

    def test_console_status_still_contains_live_flight_data(self):
        flight = {"available": True, "flight_controller_connected": True,
                  "armed": False, "mode": "LOITER"}
        self.flight_urlopen.side_effect = lambda *args, **kwargs: io.BytesIO(json.dumps(flight).encode())
        result = self.get("/api/status")
        self.assert_vision_unchanged(result)
        self.assertEqual(result["flight"], flight)
        self.assertIs(result["flight_telemetry_connected"], True)
        self.flight_urlopen.assert_called_once_with("http://flight.invalid/api/status", timeout=0.15)

    def test_console_fallback_preserves_vision_when_flight_service_fails(self):
        result = self.get("/api/status")
        self.assert_vision_unchanged(result)
        self.assertIs(result["flight"]["available"], False)
        self.assertIs(result["flight_telemetry_connected"], False)
        self.flight_urlopen.assert_called_once()

    def test_vision_remains_independent_while_console_waits_on_flight(self):
        started, release = threading.Event(), threading.Event()

        def blocked_flight(*args, **kwargs):
            started.set()
            if not release.wait(timeout=2):
                raise TimeoutError("test flight service remained blocked")
            return io.BytesIO(b'{"available":true,"flight_controller_connected":true}')

        self.flight_urlopen.side_effect = blocked_flight
        with ThreadPoolExecutor(max_workers=1) as pool:
            console = pool.submit(self.get, "/api/status", 3)
            try:
                self.assertTrue(started.wait(timeout=1))
                result = self.get("/api/vision/status", timeout=0.5)
                self.assert_vision_unchanged(result)
                self.assertFalse(console.done())
                self.flight_urlopen.assert_called_once()
            finally:
                release.set()
            self.assertIs(console.result(timeout=1)["flight_telemetry_connected"], True)


if __name__ == "__main__":
    unittest.main()
