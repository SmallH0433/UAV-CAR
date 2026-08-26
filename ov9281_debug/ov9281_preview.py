#!/usr/bin/env python3
"""OV9281-specific browser preview and chessboard calibration collector.

The capture, browser preview, and chessboard detector run at independent
rates so calibration work cannot stall the live stream. This file is
deliberately independent from the legacy IMX296 preview.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from picamera2 import Picamera2


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV9281 Calibration Console</title>
<style>
:root{color-scheme:dark;--bg:#0b0f14;--panel:#121923;--line:#253247;--green:#38d996;--amber:#ffbd59;--muted:#91a0b6}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#edf3fb;font-family:Inter,Segoe UI,Arial,sans-serif}
header{display:flex;justify-content:space-between;align-items:center;padding:14px 20px;border-bottom:1px solid var(--line);background:#0e141d}
h1{font-size:18px;margin:0}.tag{color:var(--green);font:600 12px Consolas,monospace}.layout{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:14px;padding:14px}
.viewer,.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;overflow:hidden}.viewer{min-height:420px;display:grid;place-items:center}
.viewer img{display:block;max-width:100%;max-height:calc(100vh - 105px)}aside{display:flex;flex-direction:column;gap:12px}.card{padding:14px}
.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.09em}.value{font:600 20px Consolas,monospace;margin-top:5px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.bar{height:8px;background:#253247;border-radius:9px;overflow:hidden;margin-top:10px}.fill{height:100%;width:0;background:linear-gradient(90deg,#26b982,var(--green));transition:width .25s}
.ok{color:var(--green)}.wait{color:var(--amber)}ul{padding-left:18px;color:#b8c5d8;font-size:13px;line-height:1.55;margin-bottom:0}
@media(max-width:850px){.layout{grid-template-columns:1fr}.viewer{min-height:280px}.viewer img{max-height:none}}
</style></head><body>
<header><h1>OV9281 Calibration Console</h1><div class="tag">MONO · GLOBAL SHUTTER · 1280×800</div></header>
<main class="layout"><section class="viewer"><img src="/stream.mjpg" alt="OV9281 live stream"></section><aside>
<section class="card"><div class="label">Calibration progress</div><div class="value"><span id="saved">0</span> / <span id="target">20</span></div><div class="bar"><div class="fill" id="fill"></div></div></section>
<section class="card grid"><div><div class="label">Preview</div><div class="value"><span id="preview">0</span> fps</div></div><div><div class="label">Capture</div><div class="value"><span id="capture">0</span> fps</div></div><div><div class="label">Chessboard</div><div class="value" id="board">SEARCH</div></div><div><div class="label">Frame age</div><div class="value"><span id="age">0</span> ms</div></div></section>
<section class="card"><div class="label">Session</div><div class="value" style="font-size:14px" id="status">STARTING</div><ul><li>9×6 inner corners, 17 mm squares</li><li>Move through centre, edges and corners</li><li>Include near, far and tilted views</li><li>Keep the entire board sharp and visible</li></ul></section>
</aside></main>
<script>
async function update(){try{const r=await fetch('/api/status',{cache:'no-store'}),s=await r.json();
saved.textContent=s.saved;target.textContent=s.target;fill.style.width=(100*s.saved/s.target)+'%';preview.textContent=s.preview_fps.toFixed(1);capture.textContent=s.capture_fps.toFixed(1);age.textContent=Math.round(s.frame_age_ms);
board.textContent=s.board_found?'FOUND':'SEARCH';board.className='value '+(s.board_found?'ok':'wait');status.textContent=s.complete?'COMPLETE — READY TO CALIBRATE':'COLLECTING DISTINCT VIEWS';status.className='value '+(s.complete?'ok':'wait');}catch(e){status.textContent='RECONNECTING';status.className='value wait'}}
setInterval(update,500);update();
</script></body></html>""".encode()


def signature(corners, cols: int, rows: int, width: int, height: int) -> tuple[float, ...]:
    pts = corners.reshape(-1, 2)
    tl, tr, bl = pts[0], pts[cols - 1], pts[(rows - 1) * cols]
    center = pts.mean(axis=0)
    return (
        float(center[0] / width),
        float(center[1] / height),
        float(math.hypot(*(tr - tl)) / width),
        float(math.hypot(*(bl - tl)) / height),
        float(math.atan2(float(tr[1] - tl[1]), float(tr[0] - tl[0]))),
    )


def signature_distance(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    scales = (0.05, 0.05, 0.05, 0.05, math.radians(5.0))
    return math.sqrt(sum(((x - y) / scale) ** 2 for x, y, scale in zip(a, b, scales)))


class OV9281State:
    def __init__(self, args: argparse.Namespace):
        cameras = Picamera2.global_camera_info()
        if not cameras or str(cameras[0].get("Model", "")).lower() != "ov9281":
            raise RuntimeError(f"Expected OV9281, detected: {cameras}")

        self.args = args
        self.output = args.collect_output
        self.output.mkdir(parents=True, exist_ok=True)
        self.picam2 = Picamera2(0)
        period_us = round(1_000_000 / args.capture_fps)
        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": (1280, 800)},
            controls={"FrameDurationLimits": (period_us, period_us)},
            buffer_count=6,
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest_frame = None
        self.latest_jpeg = None
        self.latest_annotated = None
        self.frame_time = 0.0
        self.board_found = False
        self.saved = len(list(self.output.glob("calib_[0-9][0-9].jpg")))
        self.signatures: list[tuple[float, ...]] = []
        self.capture_fps = 0.0
        self.preview_fps = 0.0
        self.capture_count = 0
        self.preview_count = 0
        self.rate_time = time.monotonic()
        self.last_detect = 0.0
        self.last_save = 0.0

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.preview_thread = threading.Thread(target=self._preview_loop, daemon=True)
        self.detector_thread = threading.Thread(target=self._detector_loop, daemon=True)
        self.capture_thread.start()
        self.preview_thread.start()
        self.detector_thread.start()

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            frame = self.picam2.capture_array("main")
            now = time.monotonic()
            with self.lock:
                self.latest_frame = frame
                self.frame_time = now
                self.capture_count += 1
                elapsed = now - self.rate_time
                if elapsed >= 1.0:
                    self.capture_fps = self.capture_count / elapsed
                    self.preview_fps = self.preview_count / elapsed
                    self.capture_count = self.preview_count = 0
                    self.rate_time = now

    def _preview_loop(self) -> None:
        preview_period = 1.0 / self.args.preview_fps
        next_preview = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
            if frame is None:
                time.sleep(0.02)
                continue

            if now >= next_preview:
                annotated = frame.copy()
                with self.lock:
                    corners = None if self.latest_annotated is None else self.latest_annotated.copy()
                    found = self.board_found
                    saved = self.saved
                if corners is not None and found:
                    cv2.drawChessboardCorners(annotated, (9, 6), corners, True)
                cv2.drawMarker(annotated, (640, 400), (255, 255, 255), cv2.MARKER_CROSS, 28, 2)
                color = (80, 255, 140) if found else (60, 185, 255)
                cv2.putText(annotated, f"OV9281 | 1280x800 | views {saved}/{self.args.target_count}",
                            (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
                cv2.putText(annotated, "CHESSBOARD FOUND" if found else "SEARCHING 9x6 CHESSBOARD",
                            (18, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if ok:
                    with self.lock:
                        self.latest_jpeg = encoded.tobytes()
                        self.preview_count += 1
                next_preview = now + preview_period
            time.sleep(0.005)

    def _detector_loop(self) -> None:
        detect_period = 1.0 / self.args.detect_fps
        while not self.stop_event.is_set():
            started = time.monotonic()
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
                complete = self.saved >= self.args.target_count
            if frame is not None and not complete:
                self._detect_and_collect(frame, started)
                self.last_detect = started
            remaining = detect_period - (time.monotonic() - started)
            if remaining > 0:
                self.stop_event.wait(remaining)

    def _detect_and_collect(self, frame, now: float) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        small = cv2.resize(gray, (640, 400), interpolation=cv2.INTER_AREA)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, small_corners = cv2.findChessboardCorners(small, (9, 6), flags)
        refined = None
        if found:
            corners = small_corners * 2.0
            refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 35, 0.001),
            )
            sig = signature(refined, 9, 6, 1280, 800)
            distinct = not self.signatures or min(signature_distance(sig, old) for old in self.signatures) >= self.args.min_view_change
            if distinct and now - self.last_save >= self.args.save_interval and self.saved < self.args.target_count:
                stem = f"calib_{self.saved:02d}"
                marked = frame.copy()
                cv2.drawChessboardCorners(marked, (9, 6), refined, True)
                cv2.imwrite(str(self.output / f"{stem}.jpg"), frame)
                cv2.imwrite(str(self.output / f"{stem}_corners.jpg"), marked)
                self.signatures.append(sig)
                self.saved += 1
                self.last_save = now
                print(f"saved {self.saved}/{self.args.target_count}: {stem}.jpg", flush=True)
        with self.lock:
            self.board_found = bool(found)
            self.latest_annotated = refined

    def status(self) -> dict:
        with self.lock:
            age = max(0.0, (time.monotonic() - self.frame_time) * 1000) if self.frame_time else 0.0
            return {
                "sensor": "ov9281", "resolution": [1280, 800],
                "capture_fps": self.capture_fps, "preview_fps": self.preview_fps,
                "board_found": self.board_found, "saved": self.saved,
                "target": self.args.target_count, "complete": self.saved >= self.args.target_count,
                "frame_age_ms": age, "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            }

    def close(self) -> None:
        self.stop_event.set()
        self.capture_thread.join(timeout=2)
        self.preview_thread.join(timeout=2)
        self.detector_thread.join(timeout=2)
        self.picam2.close()


class Handler(BaseHTTPRequestHandler):
    state: OV9281State

    def do_GET(self):  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send(HTTPStatus.OK, "text/html; charset=utf-8", HTML)
        elif self.path == "/api/status":
            self._send(HTTPStatus.OK, "application/json", json.dumps(self.state.status()).encode())
        elif self.path == "/snapshot.jpg":
            with self.state.lock:
                data = self.state.latest_jpeg
            if data is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera warming up")
            else:
                self._send(HTTPStatus.OK, "image/jpeg", data)
        elif self.path == "/stream.mjpg":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = None
            try:
                while True:
                    with self.state.lock:
                        data = self.state.latest_jpeg
                    if data and data is not last:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                        self.wfile.flush()
                        last = data
                    time.sleep(0.02)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _send(self, status: HTTPStatus, content_type: str, data: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *_args) -> None:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OV9281 preview and calibration collector")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--capture-fps", type=float, default=30.0)
    parser.add_argument("--preview-fps", type=float, default=12.0)
    parser.add_argument("--detect-fps", type=float, default=2.0)
    parser.add_argument("--collect-output", type=Path, default=Path.home() / "ov9281_calibration_images")
    parser.add_argument("--target-count", type=int, default=20)
    parser.add_argument("--save-interval", type=float, default=1.2)
    parser.add_argument("--min-view-change", type=float, default=0.8)
    args = parser.parse_args()
    for name in ("capture_fps", "preview_fps", "detect_fps", "save_interval", "min_view_change"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    return args


def main() -> None:
    args = parse_args()
    state = OV9281State(args)
    Handler.state = state
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"OV9281 console ready: http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        state.close()


if __name__ == "__main__":
    main()
