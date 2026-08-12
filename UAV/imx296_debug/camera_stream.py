#!/usr/bin/env python3
"""Keep the IMX296 open and expose a lightweight browser preview."""

from __future__ import annotations

import argparse
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from picamera2 import Picamera2

from collect_calibration import view_change, view_signature


HTML = b"""<!doctype html>
<html><head><meta charset='utf-8'><title>IMX296 preview</title></head>
<body style='background:#202124;color:#eee;font-family:Arial,sans-serif'>
<h3>IMX296 live preview</h3>
<p>Keep this page open while adjusting focus and mounting the camera.</p>
<img src='/stream.mjpg' style='max-width:95vw;max-height:85vh;border:1px solid #777'>
</body></html>"""


class CameraState:
    def __init__(self, width: int, height: int, fps: float, collect_output: Path | None,
                 target_count: int, duration_s: float, min_view_change: float):
        self.picam2 = Picamera2()
        frame_period_us = int(1_000_000 / fps)
        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": (width, height)},
            controls={"FrameDurationLimits": (frame_period_us, frame_period_us)},
            buffer_count=4,
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2.0)
        self.jpeg = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.collect_output = collect_output
        self.target_count = target_count
        self.duration_s = duration_s
        self.min_view_change = min_view_change
        self.collect_start = time.monotonic()
        self.last_save = 0.0
        self.saved = 0
        self.signatures = []
        if self.collect_output:
            self.collect_output.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while not self.stop_event.is_set():
            frame = self.picam2.capture_array("main")
            now = time.monotonic()
            if self.collect_output and now - self.collect_start < self.duration_s and self.saved < self.target_count:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                pattern = (9, 6)
                found, corners = cv2.findChessboardCorners(
                    gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                )
                if found and now - self.last_save >= 1.5:
                    refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                    )
                    signature = view_signature(refined, 9, 6, frame.shape[1], frame.shape[0])
                    distinct = not self.signatures or max(
                        view_change(signature, old) for old in self.signatures
                    ) >= self.min_view_change
                    if distinct:
                        stem = f"calib_{self.saved:02d}"
                        annotated = frame.copy()
                        cv2.drawChessboardCorners(annotated, pattern, refined, found)
                        cv2.imwrite(str(self.collect_output / f"{stem}.jpg"), frame)
                        cv2.imwrite(str(self.collect_output / f"{stem}_corners.jpg"), annotated)
                        self.signatures.append(signature)
                        self.saved += 1
                        self.last_save = now
                        print(f"saved {self.saved}/{self.target_count}: {stem}.jpg", flush=True)
            # Picamera2's RGB888 array is already in the channel order expected
            # by the current OpenCV/libcamera path here. Do not swap R/B again;
            # doing so makes skin tones appear blue in the browser preview.
            preview_width = 960
            preview_height = round(frame.shape[0] * preview_width / frame.shape[1])
            preview = cv2.resize(frame, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
            height, width = preview.shape[:2]
            cv2.drawMarker(preview, (width // 2, height // 2), (0, 255, 0), cv2.MARKER_CROSS, 28, 2)
            cv2.putText(preview, "IMX296 live - focus / framing preview", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            if self.collect_output:
                cv2.putText(preview, f"calibration views: {self.saved}/{self.target_count}", (12, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
            ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with self.lock:
                    self.jpeg = encoded.tobytes()

    def latest(self):
        with self.lock:
            return self.jpeg

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.picam2.close()


class Handler(BaseHTTPRequestHandler):
    state: CameraState

    def do_GET(self):  # noqa: N802
        if self.path == "/" or self.path == "/index.html":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML)
            return
        if self.path == "/snapshot.jpg":
            data = self.state.latest()
            if data is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera warming up")
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path == "/stream.mjpg":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    data = self.state.latest()
                    if data:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                        self.wfile.flush()
                    time.sleep(0.12)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format, *args):  # noqa: A002
        return


def main():
    parser = argparse.ArgumentParser(description="IMX296 browser preview")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--collect-output", type=Path, default=None)
    parser.add_argument("--target-count", type=int, default=15)
    parser.add_argument("--duration-s", type=float, default=180.0)
    parser.add_argument("--min-view-change", type=float, default=1.0)
    args = parser.parse_args()

    state = CameraState(
        args.width, args.height, args.fps, args.collect_output,
        args.target_count, args.duration_s, args.min_view_change,
    )
    Handler.state = state
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"IMX296 preview ready: http://0.0.0.0:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        state.close()


if __name__ == "__main__":
    main()
