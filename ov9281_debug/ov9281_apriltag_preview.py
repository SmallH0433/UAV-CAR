#!/usr/bin/env python3
"""OV9281 AprilTag ID/quality preview with no flight-controller connection.

Metric pose is deliberately disabled until a low-error OV9281 calibration is
available. The UI reports image-space measurements only.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector


HTML = r"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV9281 AprilTag Monitor</title><style>
:root{color-scheme:dark;--bg:#0a0e14;--panel:#121a25;--line:#26344a;--green:#3de19b;--amber:#ffc05c;--muted:#91a2ba}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#edf4fc;font-family:Segoe UI,Arial,sans-serif}header{display:flex;justify-content:space-between;padding:14px 20px;border-bottom:1px solid var(--line)}
h1{font-size:18px;margin:0}.badge{color:var(--green);font:600 12px Consolas,monospace}.layout{display:grid;grid-template-columns:minmax(0,1fr) 310px;gap:14px;padding:14px}.viewer,.card{background:var(--panel);border:1px solid var(--line);border-radius:12px}.viewer{overflow:hidden;display:grid;place-items:center;min-height:420px}.viewer img{display:block;max-width:100%;max-height:calc(100vh - 105px)}aside{display:flex;flex-direction:column;gap:12px}.card{padding:14px}.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}.value{font:600 20px Consolas,monospace;margin-top:5px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:13px}.ok{color:var(--green)}.wait{color:var(--amber)}.notice{font-size:13px;line-height:1.55;color:#bac7d8}.notice strong{color:var(--amber)}
@media(max-width:850px){.layout{grid-template-columns:1fr}.viewer{min-height:280px}.viewer img{max-height:none}}
</style></head><body><header><h1>OV9281 AprilTag Monitor</h1><div class="badge">TAG36H11 · ID 0 · IMAGE SPACE</div></header>
<main class="layout"><section class="viewer"><img src="/stream.mjpg"></section><aside>
<section class="card"><div class="label">Target state</div><div class="value wait" id="state">SEARCHING</div></section>
<section class="card grid"><div><div class="label">Tag ID</div><div class="value" id="id">—</div></div><div><div class="label">Decision margin</div><div class="value" id="margin">—</div></div><div><div class="label">Center X</div><div class="value" id="x">—</div></div><div><div class="label">Center Y</div><div class="value" id="y">—</div></div><div><div class="label">Area</div><div class="value" id="area">—</div></div><div><div class="label">Hamming</div><div class="value" id="hamming">—</div></div></section>
<section class="card grid"><div><div class="label">Capture</div><div class="value"><span id="capture">0</span> fps</div></div><div><div class="label">Preview</div><div class="value"><span id="preview">0</span> fps</div></div></section>
<section class="card notice"><strong>距离与 X/Y/Z 暂停显示。</strong><br>当前棋盘标定误差过大，本页面只验证标签 ID、像素位置和识别质量，不连接飞控、不发送 MAVLink。</section>
</aside></main><script>
const val=(v,d=1)=>v==null?'—':Number(v).toFixed(d);async function update(){try{const s=await(await fetch('/api/status',{cache:'no-store'})).json();
state.textContent=s.found?'DETECTED':'SEARCHING';state.className='value '+(s.found?'ok':'wait');id.textContent=s.tag_id??'—';margin.textContent=val(s.decision_margin);x.textContent=val(s.center_x_px);y.textContent=val(s.center_y_px);area.textContent=s.area_px2==null?'—':Math.round(s.area_px2);hamming.textContent=s.hamming??'—';capture.textContent=val(s.capture_fps);preview.textContent=val(s.preview_fps);}catch(e){state.textContent='RECONNECTING';state.className='value wait'}}setInterval(update,400);update();
</script></body></html>""".encode()


class State:
    def __init__(self, args):
        info = Picamera2.global_camera_info()
        if not info or str(info[0].get("Model", "")).lower() != "ov9281":
            raise RuntimeError(f"Expected OV9281, detected: {info}")
        self.args = args
        self.camera = Picamera2(0)
        period = round(1_000_000 / args.capture_fps)
        self.camera.configure(self.camera.create_video_configuration(
            main={"format": "RGB888", "size": (1280, 800)},
            controls={"FrameDurationLimits": (period, period)}, buffer_count=6,
        ))
        self.camera.start()
        self.detector = Detector(families="tag36h11", nthreads=4, quad_decimate=2.0, refine_edges=True)
        self.lock = threading.Lock(); self.stop = threading.Event()
        self.frame = None; self.jpeg = None; self.corners = None; self.observation = None
        self.capture_fps = self.preview_fps = 0.0; self.cc = self.pc = 0; self.rate_at = time.monotonic()
        self.threads = [threading.Thread(target=self._capture, daemon=True), threading.Thread(target=self._detect, daemon=True), threading.Thread(target=self._preview, daemon=True)]
        for thread in self.threads: thread.start()

    def _capture(self):
        while not self.stop.is_set():
            frame = self.camera.capture_array("main"); now = time.monotonic()
            with self.lock:
                self.frame = frame; self.cc += 1
                elapsed = now - self.rate_at
                if elapsed >= 1:
                    self.capture_fps = self.cc / elapsed; self.preview_fps = self.pc / elapsed
                    self.cc = self.pc = 0; self.rate_at = now

    def _detect(self):
        period = 1 / self.args.detect_fps
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock: frame = None if self.frame is None else self.frame.copy()
            observation = None; corners = None
            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                detections = self.detector.detect(gray, estimate_tag_pose=False)
                matches = [d for d in detections if int(d.tag_id) == self.args.tag_id]
                if matches:
                    d = max(matches, key=lambda item: float(item.decision_margin))
                    corners = np.asarray(d.corners, dtype=np.float32).reshape(4, 2)
                    center = corners.mean(axis=0)
                    observation = {"tag_id": int(d.tag_id), "decision_margin": float(d.decision_margin), "hamming": int(d.hamming), "center_x_px": float(center[0]), "center_y_px": float(center[1]), "area_px2": abs(float(cv2.contourArea(corners)))}
            with self.lock: self.observation = observation; self.corners = corners
            self.stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _preview(self):
        period = 1 / self.args.preview_fps
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock:
                frame = None if self.frame is None else self.frame.copy(); corners = None if self.corners is None else self.corners.copy(); obs = self.observation
            if frame is not None:
                color = (70, 240, 140) if obs else (60, 185, 255)
                if corners is not None:
                    cv2.polylines(frame, [corners.astype(np.int32)], True, color, 3)
                    center = tuple(np.round(corners.mean(axis=0)).astype(int)); cv2.drawMarker(frame, center, color, cv2.MARKER_CROSS, 28, 2)
                cv2.putText(frame, "OV9281 | TAG36H11 ID 0 | " + ("DETECTED" if obs else "SEARCHING"), (18, 36), cv2.FONT_HERSHEY_SIMPLEX, .72, color, 2, cv2.LINE_AA)
                cv2.putText(frame, "POSE DISABLED - CALIBRATION REQUIRED", (18, 68), cv2.FONT_HERSHEY_SIMPLEX, .56, (60, 185, 255), 2, cv2.LINE_AA)
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if ok:
                    with self.lock: self.jpeg = encoded.tobytes(); self.pc += 1
            self.stop.wait(max(0.0, period - (time.monotonic() - started)))

    def status(self):
        with self.lock:
            result = {"sensor":"ov9281","found":self.observation is not None,"capture_fps":self.capture_fps,"preview_fps":self.preview_fps,"pose_enabled":False}
            if self.observation: result.update(self.observation)
            return result

    def close(self):
        self.stop.set()
        for thread in self.threads: thread.join(timeout=2)
        self.camera.close()


class Handler(BaseHTTPRequestHandler):
    state: State
    def do_GET(self):  # noqa: N802
        if self.path in ("/", "/index.html"): return self.send_data(HTTPStatus.OK, "text/html; charset=utf-8", HTML)
        if self.path == "/api/status": return self.send_data(HTTPStatus.OK, "application/json", json.dumps(self.state.status()).encode())
        if self.path == "/snapshot.jpg":
            with self.state.lock: data = self.state.jpeg
            return self.send_data(HTTPStatus.OK, "image/jpeg", data) if data else self.send_error(503)
        if self.path == "/stream.mjpg":
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.send_header("Cache-Control", "no-store"); self.end_headers(); last = None
            try:
                while True:
                    with self.state.lock: data = self.state.jpeg
                    if data and data is not last: self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"); self.wfile.flush(); last = data
                    time.sleep(.02)
            except (BrokenPipeError, ConnectionResetError): pass
            return
        self.send_error(404)
    def send_data(self, status, content_type, data):
        self.send_response(status); self.send_header("Content-Type", content_type); self.send_header("Cache-Control", "no-store"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
    def log_message(self, *_): pass


def main():
    p = argparse.ArgumentParser(); p.add_argument("--host", default="0.0.0.0"); p.add_argument("--port", type=int, default=8765); p.add_argument("--tag-id", type=int, default=0); p.add_argument("--capture-fps", type=float, default=30); p.add_argument("--preview-fps", type=float, default=12); p.add_argument("--detect-fps", type=float, default=10); args = p.parse_args()
    state = State(args); Handler.state = state; server = ThreadingHTTPServer((args.host, args.port), Handler); print(f"OV9281 AprilTag monitor ready: http://{args.host}:{args.port}/", flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close(); state.close()


if __name__ == "__main__": main()
