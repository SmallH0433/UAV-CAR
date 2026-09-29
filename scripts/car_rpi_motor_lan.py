#!/usr/bin/env python3
"""LAN web server on the Raspberry Pi for direct GPIO motor control."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse



BASE = Path(__file__).resolve().parent
MANUAL_SCRIPT = BASE / "leadscrew_manual.py"
PAGE = BASE / "motor_lan.html"
PULSES_PER_REV = 1600
PITCH_MM = 2.0


class MotorState:
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.message = "已就绪"
        self.output = ""
        self.operation = "move"
        self.stop_requested = False

    def snapshot(self):
        with self.lock:
            return {"moving": self.process is not None,
                    "resetting": self.process is not None and self.operation == 'reset',
                    "reset_available": True, "reset_reason": "",
                    "reset_scope": "当前选择的电机",
                    "message": self.message, "output": self.output}

    @staticmethod
    def validate(data):
        try:
            motor = int(data.get("motor"))
            direction = int(data.get("direction"))
            rpm = float(data.get("rpm"))
            distance = float(data.get("distance_mm"))
            ramp = float(data.get("ramp_seconds"))
        except (TypeError, ValueError) as exc:
            raise ValueError("请填写有效的电机、方向、转速、行程和加速时间") from exc
        if motor not in (1, 2) or direction not in (0, 1):
            raise ValueError("电机或方向选择无效")
        if not math.isfinite(rpm) or not 0 < rpm <= 1000 or rpm * PULSES_PER_REV / 60 < 5:
            raise ValueError("转速范围：0.1875～1000 rpm")
        if not math.isfinite(distance) or not 0 < distance <= 62:
            raise ValueError("本次行程须在 0～62 mm 之间")
        if round(distance * PULSES_PER_REV / PITCH_MM) < 1:
            raise ValueError("行程太小，换算后不足 1 个脉冲")
        if not math.isfinite(ramp) or not 0.05 <= ramp <= 5:
            raise ValueError("加速时间须在 0.05～5 秒之间")
        return motor, direction, rpm, distance, ramp

    def start(self, data):
        motor, direction, rpm, distance, ramp = self.validate(data)
        if not MANUAL_SCRIPT.is_file():
            raise FileNotFoundError(f"缺少控制脚本：{MANUAL_SCRIPT}")
        command = [sys.executable, str(MANUAL_SCRIPT), "--motor", str(motor),
                   "--direction", str(direction), "--rpm", str(rpm),
                   "--distance-mm", str(distance), "--ramp-seconds", str(ramp)]
        with self.lock:
            if self.process is not None:
                raise RuntimeError("电机正在运动，请先停止或等待完成")
            process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True,
                                       encoding="utf-8", start_new_session=True)
            self.process = process
            self.operation = "move"
            self.stop_requested = False
            self.output = ""
            self.message = (f"运行中：电机{motor}，DIR={direction}，{rpm:g} rpm，"
                            f"{distance:g} mm，加速{ramp:g}秒")
        threading.Thread(target=self._collect, args=(process,), daemon=True).start()

    def reset(self, data):
        raw = data.get('motor')
        if isinstance(raw, bool) or raw not in (1, 2, '1', '2'):
            raise ValueError('请选择 1 号或 2 号电机')
        motor = int(raw)
        with self.lock:
            if self.process is not None:
                raise RuntimeError("请先停止运动并等待停止完成，再复位")
            process = subprocess.Popen(
                [sys.executable, str(BASE / 'cl42_en_reset.py'), '--motor', str(motor)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding='utf-8', start_new_session=True)
            self.process = process
            self.operation = 'reset'
            self.stop_requested = False
            self.output = ''
            self.message = f'正在对 {motor} 号电机执行 EN 复位…'
        threading.Thread(target=self._collect, args=(process,), daemon=True).start()

    def _collect(self, process):
        output, _ = process.communicate()
        with self.lock:
            self.output = output
            self.message = ("已停止" if self.stop_requested else "运行完成") if process.returncode == 0 else \
                           f"运行停止或失败（退出码 {process.returncode}）"
            if self.operation == 'reset':
                self.message = ('EN 复位已执行，请观察红灯是否熄灭' if process.returncode == 0
                                else '复位中止或失败，请查看运行反馈')
            if self.process is process:
                self.process = None

    def stop(self):
        with self.lock:
            if self.process is None:
                return
            self.stop_requested = True
            self.process.send_signal(signal.SIGINT)
            self.message = "已发送停止信号，等待电机停止…"


state = MotorState()


class Handler(BaseHTTPRequestHandler):
    def respond(self, status, body, content_type="application/json; charset=utf-8"):
        payload = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self.respond(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        elif path == "/api/status":
            self.respond(200, state.snapshot())
        else:
            self.respond(404, {"error": "未找到页面"})

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ("/api/move", "/api/stop", "/api/reset"):
            self.respond(404, {"error": "未找到接口"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 <= size <= 8192:
                raise ValueError("请求内容过大")
            data = json.loads(self.rfile.read(size)) if size else {}
            if not isinstance(data, dict):
                raise ValueError("请求格式错误")
            if path == "/api/move":
                state.start(data)
            elif path == "/api/reset":
                state.reset(data)
            else:
                state.stop()
            self.respond(200, state.snapshot())
        except (ValueError, RuntimeError, OSError) as exc:
            self.respond(400, {"error": str(exc)})


if __name__ == "__main__":
    if not PAGE.is_file() or not MANUAL_SCRIPT.is_file():
        raise SystemExit("缺少网页或电机控制脚本")
    server = ThreadingHTTPServer(("0.0.0.0", 8765), Handler)
    print("电机网页已监听 0.0.0.0:8765", flush=True)
    server.serve_forever()
