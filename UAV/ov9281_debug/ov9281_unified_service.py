#!/usr/bin/env python3
"""Unified OV9281 calibration and AprilTag browser service.

Architecture:
* main: 1280x800 YUV420; only the Y plane is used for analysis
* lores: 640x400 YUV420; encoded by Picamera2's MJPEGEncoder
* analysis and streaming are independent
* the HTTP stream always publishes only the newest encoded frame
* no MAVLink or flight-controller connection exists in this process
"""

from __future__ import annotations

import argparse
import io
import json
import math
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from pupil_apriltags import Detector
from ov9281_tag_orientation import body_tag_orientation, load_body_extrinsics

from ov9281_dual_tag import (
    parse_tag_quality_specs,
    parse_tag_specs,
    quality_rejection_reasons,
    select_primary_tag,
)


HTML = r"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OV9281 Vision Console</title><style>
:root{color-scheme:dark;--bg:#080d13;--panel:#101925;--line:#25364e;--green:#39dd98;--blue:#61a8ff;--amber:#ffc15c;--muted:#94a5bb}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:#edf5ff;font-family:Inter,Segoe UI,Arial,sans-serif}header{height:58px;display:flex;align-items:center;justify-content:space-between;padding:0 18px;border-bottom:1px solid var(--line)}
h1{margin:0;font-size:18px}.sensor{font:600 12px Consolas,monospace;color:var(--green)}.layout{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:14px;padding:14px}.viewer,.card{background:var(--panel);border:1px solid var(--line);border-radius:12px}.viewer{position:relative;overflow:hidden;display:grid;place-items:center;min-height:400px}.viewer img{display:block;width:100%;height:auto}.viewer canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
aside{display:flex;flex-direction:column;gap:12px}.card{padding:14px}.tabs{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tabs button{border:1px solid var(--line);border-radius:8px;padding:10px;background:#172233;color:#b9c6d8;font-weight:700;cursor:pointer}.tabs button.active{background:#184d3a;border-color:#2dbe82;color:#dfffee}.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}.value{font:600 19px Consolas,monospace;margin-top:5px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:13px}.ok{color:var(--green)}.wait{color:var(--amber)}.notice{font-size:13px;line-height:1.55;color:#becadd}.notice strong{color:var(--amber)}.bar{height:8px;background:#26364d;border-radius:8px;overflow:hidden;margin-top:9px}.fill{height:100%;background:var(--green);width:0;transition:width .2s}
.viewer{min-height:0;aspect-ratio:8/5;align-self:start}.direction-key{display:flex;gap:9px;align-items:center;margin-top:8px}.direction-key b{font-size:22px}.direction-key span{font-size:13px}
@media(max-width:880px){.layout{grid-template-columns:1fr}}</style></head><body>
<header><h1>OV9281 Vision Console</h1><div class="sensor">MONO · GLOBAL SHUTTER · MAIN 1280×800 · PREVIEW 640×400</div></header>
<main class="layout"><section class="viewer"><img id="stream" alt="OV9281 live preview"><canvas id="overlay" width="1280" height="800"></canvas></section><aside>
<section class="card tabs"><button id="tagBtn" onclick="setMode('apriltag')">APRILTAG</button><button id="calBtn" onclick="setMode('calibration')">CALIBRATION</button></section>
<section class="card"><div class="label">Vision state</div><div class="value wait" id="state">STARTING</div><div class="bar"><div class="fill" id="fill"></div></div></section>
<section class="card"><div class="label">Flight controller</div><div class="value wait" id="flightMode">DISCONNECTED</div><div class="grid" style="margin-top:13px"><div><div class="label">Link / Armed</div><div class="notice" id="flightLink">—</div></div><div><div class="label">Landed state</div><div class="notice" id="landedState">—</div></div><div><div class="label">Forward / Back</div><div class="value" id="horizontalDirection">—</div></div><div><div class="label">Up / Down</div><div class="value" id="verticalDirection">—</div></div><div><div class="label">Left / Right</div><div class="value" id="lateralDirection">—</div></div><div><div class="label">Control state</div><div class="notice" id="controlState">—</div></div><div><div class="label">Body velocity X/Y/Z</div><div class="notice" id="bodyVelocity">—</div></div><div><div class="label">Last command X/Y/Z</div><div class="notice" id="lastCommand">—</div></div></div><div class="notice" style="margin-top:10px">Actual body-frame velocity: X forward, Y left, Z up; ±0.05 m/s is shown as HOLD.</div></section>
<section class="card"><div class="label">准备运动 / Sent command</div><div class="direction-key" style="color:#ff1528"><b>➜</b><span>鲜红箭头＝树莓派已发出的水平速度方向</span></div><div class="notice" id="motionState">等待新指令</div><div class="notice" style="margin-top:8px">画面上＝机体前，画面右＝机体右（按 9 月 8 日现场确认）。箭头从画面中心出发；升降单独标注。表示指令，不是实际运动。</div></section>
<section class="card"><div class="label">Candidate quality</div><div class="notice" id="rejectReason">No candidate</div></section>
<section class="card"><div class="label">方向参照 / Direction</div><div class="direction-key" style="color:#00d9ff"><b>↑</b><span>画面正上方（相机 −Y）</span></div><div class="direction-key" style="color:#ffb020"><b>↑</b><span>大 Tag 自身上方 · ID 0</span></div><div class="direction-key" style="color:#ff54d9"><b>↑</b><span>小 Tag → 机体系 · ID 1</span></div><div class="notice" style="margin-top:10px">树莓派解算标签姿态，补偿小 Tag 的 45° 布局，再使用控制外参转换到机体前／右／下坐标。方向角 0°＝机头、+90°＝机体右侧。仅显示质量通过的姿态；不自动控制偏航。</div><div class="notice" id="directionState" style="margin-top:8px">等待标签</div></section>
<section class="card grid"><div><div class="label">Capture</div><div class="value"><span id="capture">0</span> fps</div></div><div><div class="label">Analysis</div><div class="value"><span id="analysis">0</span> fps</div></div><div><div class="label">Encoded</div><div class="value"><span id="encoded">0</span> fps</div></div><div><div class="label">Frame age</div><div class="value"><span id="age">0</span> ms</div></div></section>
<section class="card grid" id="tagStats"><div><div class="label">Tag ID</div><div class="value" id="tagId">—</div></div><div><div class="label">Margin</div><div class="value" id="margin">—</div></div><div><div class="label">Distance</div><div class="value" id="distance">—</div></div><div><div class="label">Raw distance</div><div class="value" id="rawDistance">—</div></div><div><div class="label">Camera X</div><div class="value" id="xm">—</div></div><div><div class="label">Camera Y</div><div class="value" id="ym">—</div></div><div><div class="label">Camera Z</div><div class="value" id="zm">—</div></div><div><div class="label">Reproj.</div><div class="value" id="reproj">—</div></div><div><div class="label">Center X</div><div class="value" id="cx">—</div></div><div><div class="label">Center Y</div><div class="value" id="cy">—</div></div></section>
<section class="card" id="calStats"><div class="label">Calibration views</div><div class="value"><span id="saved">0</span> / <span id="target">20</span></div><div class="notice" style="margin-top:10px">9×6 inner corners · 17 mm squares · mount board flat</div></section>
<section class="card notice"><strong>OV9281 dual-scale pose enabled.</strong><br>Green frame = accepted active Tag; blue frame = decoded candidate rejected by quality gates. Nested tag36h11: ID 0 black edge 0.100 m + ID 1 black edge 0.020 m. Camera optical frame: X right, Y down, Z forward.</section>
</aside></main><script>
const $=id=>document.getElementById(id), canvas=$('overlay'),ctx=canvas.getContext('2d'),stream=$('stream');let last={},streamRetry=null,streamStarted=false;
function reconnectStream(){clearTimeout(streamRetry);streamStarted=false;stream.src='/stream.mjpg?ts='+Date.now();streamRetry=setTimeout(()=>{if(!stream.naturalWidth)reconnectStream()},3000)}
stream.addEventListener('load',()=>{streamStarted=true;clearTimeout(streamRetry)});
stream.addEventListener('error',()=>{streamStarted=false;clearTimeout(streamRetry);streamRetry=setTimeout(reconnectStream,750)});
window.addEventListener('online',reconnectStream);
document.addEventListener('visibilitychange',()=>{if(!document.hidden)reconnectStream()});
async function setMode(mode){await fetch('/api/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode})});await update()}
function val(v,d=1){return v==null?'—':Number(v).toFixed(d)}
function direction(v){return ({FORWARD:'前进',BACKWARD:'后退',LEFT:'左移',RIGHT:'右移',UP:'上升',DOWN:'下降',HOLD:'保持',UNKNOWN:'未知'})[v]||v||'未知'}
function vector(v){return !v||v.x==null?'—':[v.x,v.y,v.z].map(n=>Number(n).toFixed(2)).join(' / ')+' m/s'}
function updateFlight(f){flightMode.textContent=f.mode||'DISCONNECTED';flightMode.className='value '+(f.flight_controller_connected?'ok':'wait');flightLink.textContent=(f.flight_controller_connected?'CONNECTED':'DISCONNECTED')+' / '+(f.armed?'ARMED':'DISARMED');landedState.textContent=f.landed_state||'UNKNOWN';horizontalDirection.textContent=direction(f.horizontal_direction);verticalDirection.textContent=direction(f.vertical_direction);lateralDirection.textContent=direction(f.lateral_direction);controlState.textContent=f.landing_active?'LAND ACTIVE':(f.follow_active?'GUIDED FOLLOW':(f.control_owner||'IDLE'));bodyVelocity.textContent=vector(f.body_velocity_mps);lastCommand.textContent=vector(f.latest_sent_velocity_mps)}
function polygon(pts,color,width){if(!pts||!pts.length)return;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(pts[0][0],pts[0][1]);for(let i=1;i<pts.length;i++)ctx.lineTo(pts[i][0],pts[i][1]);ctx.closePath();ctx.stroke()}
function tagDirection(d){
  const o=d.orientation;
  if(!o||!o.valid||o.frame!=='BODY_FRD'||o.source!=='PNP_TAG_TO_PAD_TO_BODY')return null;
  const p=o.arrow_image_px;if(!Array.isArray(p)||p.length!==2||!p.every(v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite)))return null;
  const inner=d.role==='inner'||d.tag_id===1,origin=p[0],tip=p[1];
  const dx=tip[0]-origin[0],dy=tip[1]-origin[1],n=Math.hypot(dx,dy);
  if(!Number.isFinite(n)||n<1e-6)return null;
  return {origin,unit:[dx/n,dy/n],inner,heading:o.pad_heading_body_deg,body:o.pad_forward_body_frd};
}
function directionArrow(origin,unit,length,color,label,dashed=false,width=7){
  const end=[origin[0]+unit[0]*length,origin[1]+unit[1]*length],a=Math.atan2(unit[1],unit[0]);
  ctx.save();ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=width;ctx.lineCap='round';ctx.shadowColor='#000';ctx.shadowBlur=4;ctx.setLineDash(dashed?[12,8]:[]);
  ctx.beginPath();ctx.moveTo(...origin);ctx.lineTo(...end);ctx.stroke();ctx.setLineDash([]);
  ctx.beginPath();ctx.moveTo(...end);ctx.lineTo(end[0]-22*Math.cos(a-.45),end[1]-22*Math.sin(a-.45));ctx.lineTo(end[0]-22*Math.cos(a+.45),end[1]-22*Math.sin(a+.45));ctx.closePath();ctx.fill();
  ctx.font='bold 22px "Segoe UI",sans-serif';ctx.lineWidth=5;ctx.strokeStyle='#071019';
  const x=Math.max(8,Math.min(1280-ctx.measureText(label).width-8,end[0]+14)),y=Math.max(28,Math.min(788,end[1]-12));
  ctx.strokeText(label,x,y);ctx.fillText(label,x,y);ctx.restore();
}
let directionReceivedAt=0;
function motionDirection(f,elapsedMs){
  const c=f.motion_command,v=c&&c.body_velocity_mps;
  if(!f.flight_controller_connected||f.mode!=='GUIDED'||!c||c.state!=='READY'||c.body_frame!=='FLU'||!Number.isFinite(c.age_s)||c.age_s+elapsedMs/1000>.7||!v||![v.x,v.y,v.z].every(Number.isFinite))return null;
  // This is the user's current physical image convention, independent of
  // the controller's older camera extrinsics. Do not rotate toward the tag.
  const x=-v.y,y=-v.x,speed=Math.hypot(x,y);
  return {unit:speed>1e-4?[x/speed,y/speed]:null,speed,up:v.z};
}
function drawMotion(f){
  const m=motionDirection(f,Date.now()-directionReceivedAt);
  if(!m){$('motionState').textContent=f.mode==='LAND'?'飞控自主 LAND：无新鲜树莓派速度指令':'无新鲜运动指令，红色箭头已隐藏';return;}
  const vertical=m.up>.001?'上升':m.up<-.001?'下降':'无升降';
  const label=m.unit?'准备运动 '+m.speed.toFixed(3)+' m/s':'水平保持';
  if(m.unit)directionArrow([640,400],m.unit,Math.min(235,115+m.speed*450),'#ff1528',label,false,11);
  ctx.save();ctx.font='bold 24px "Segoe UI",sans-serif';ctx.lineWidth=5;ctx.strokeStyle='#071019';ctx.fillStyle='#ff1528';
  const text=vertical+(Math.abs(m.up)>.001?' '+Math.abs(m.up).toFixed(3)+' m/s':'');
  ctx.strokeText(text,24,770);ctx.fillText(text,24,770);ctx.restore();
  $('motionState').textContent=label+' · '+text;
}
function draw(s){
  ctx.clearRect(0,0,1280,800);
  directionArrow([90,175],[0,-1],100,'#00d9ff','画面上方');
  const fresh=Number.isFinite(s.frame_age_ms)&&s.frame_age_ms<700&&Date.now()-directionReceivedAt<1000;
  const detections=s.mode==='apriltag'&&fresh?(s.detections||[]):[];
  if(s.mode==='calibration'&&fresh)polygon(s.overlay_points,'#ffc15c',5);
  const shown=[];
  for(const d of detections)polygon(d.corners_px,d.tag_id===s.tag_id?'#39dd98':'#61a8ff',d.tag_id===s.tag_id?6:3);
  // Long wide outer arrow and short narrow inner arrow remain distinct when aligned.
  for(const d of [...detections].sort((a,b)=>a.tag_id-b.tag_id)){
    const v=tagDirection(d);if(!v)continue;
    directionArrow(v.origin,v.unit,v.inner?90:155,v.inner?'#ff54d9':'#ffb020',v.inner?'小 Tag 校正':'大 Tag 上方',!d.quality_passed,v.inner?5:9);
    shown.push((v.inner?'小 Tag':'大 Tag')+'：机体系方向角 '+(Number.isFinite(v.heading)?v.heading.toFixed(1)+'°':'未知')+'，前/右/下 ['+v.body.map(n=>n.toFixed(2)).join(', ')+']');
  }
  $('directionState').textContent=shown.join(' · ')||(fresh?'未识别到标签方向':'图像或数据已过期，标签箭头已隐藏');
  drawMotion(s.flight||{});
}
setInterval(()=>draw(last),200);
async function update(){try{const s=await(await fetch('/api/status',{cache:'no-store'})).json();last=s;directionReceivedAt=Date.now();tagBtn.classList.toggle('active',s.mode==='apriltag');calBtn.classList.toggle('active',s.mode==='calibration');tagStats.style.display=s.mode==='apriltag'?'grid':'none';calStats.style.display=s.mode==='calibration'?'block':'none';state.textContent=s.state;state.className='value '+(s.found?'ok':'wait');const rejected=(s.detections||[]).filter(d=>!d.quality_passed);rejectReason.textContent=s.found?'Accepted: ID '+s.tag_id:(rejected.length?rejected.map(d=>'ID '+d.tag_id+': '+(d.quality_rejection_reasons||[]).join(', ')).join(' | '):'No candidate');capture.textContent=val(s.capture_fps);analysis.textContent=val(s.analysis_fps);encoded.textContent=val(s.encoded_fps);age.textContent=Math.round(s.frame_age_ms);tagId.textContent=s.tag_id??'—';margin.textContent=val(s.decision_margin);distance.textContent=s.distance_m==null?'—':val(s.distance_m,3)+' m';rawDistance.textContent=s.raw_distance_m==null?'—':val(s.raw_distance_m,3)+' m';reproj.textContent=s.reprojection_error_px==null?'—':val(s.reprojection_error_px,2)+' px';xm.textContent=s.x_m==null?'—':val(s.x_m,3)+' m';ym.textContent=s.y_m==null?'—':val(s.y_m,3)+' m';zm.textContent=s.z_m==null?'—':val(s.z_m,3)+' m';cx.textContent=val(s.center_x_px);cy.textContent=val(s.center_y_px);saved.textContent=s.saved;target.textContent=s.target;fill.style.width=(100*s.saved/s.target)+'%';draw(s);updateFlight(s.flight||{})}catch(e){state.textContent='RECONNECTING';state.className='value wait';updateFlight({})}}setInterval(update,350);reconnectStream();update();
</script></body></html>""".encode()


class LatestMJPEG(io.BufferedIOBase):
    def __init__(self, maximum_fps=10.0):
        if maximum_fps <= 0:
            raise ValueError('preview fps must be positive')
        self.condition = threading.Condition()
        self.frame: bytes | None = None
        self.sequence = 0
        self.frames = 0
        self.fps = 0.0
        self.rate_at = time.monotonic()
        self.minimum_period_s = 1.0 / float(maximum_fps)
        self.last_frame_at = 0.0

    def write(self, buf):
        now = time.monotonic()
        if now - self.last_frame_at < self.minimum_period_s:
            return len(buf)
        self.last_frame_at = now
        with self.condition:
            self.frame = bytes(buf)
            self.sequence += 1
            self.frames += 1
            elapsed = now - self.rate_at
            if elapsed >= 1.0:
                self.fps = self.frames / elapsed
                self.frames = 0
                self.rate_at = now
            self.condition.notify_all()
        return len(buf)


def view_signature(corners):
    pts = corners.reshape(-1, 2); tl, tr, bl = pts[0], pts[8], pts[45]
    center = pts.mean(axis=0)
    return (float(center[0]/1280), float(center[1]/800), float(math.hypot(*(tr-tl))/1280), float(math.hypot(*(bl-tl))/800), float(math.atan2(float(tr[1]-tl[1]), float(tr[0]-tl[0]))))


def view_distance(a, b):
    scales=(.05,.05,.05,.05,math.radians(5)); return math.sqrt(sum(((x-y)/s)**2 for x,y,s in zip(a,b,scales)))


class VisionState:
    def __init__(self, args):
        info = Picamera2.global_camera_info()
        if not info or str(info[0].get('Model','')).lower() != 'ov9281':
            raise RuntimeError(f'Expected OV9281, detected: {info}')
        self.args=args; self.lock=threading.Lock(); self.stop=threading.Event(); self.mode=args.mode
        self.frame=None; self.frame_time=0.0; self.observation=None; self.observations=[]; self.overlay=None; self.found=False
        self.active_tag_id=None
        self.analysis_sequence=0
        self.capture_fps=self.analysis_fps=0.0; self.capture_count=self.analysis_count=0; self.rate_at=time.monotonic()
        self.output=args.collect_output; self.output.mkdir(parents=True,exist_ok=True)
        self.saved=len(list(self.output.glob('calib_[0-9][0-9].jpg'))); self.signatures=[]; self.last_save=0.0
        self.detector=Detector(families='tag36h11',nthreads=4,quad_decimate=2.0,refine_edges=True)
        self.camera_matrix, self.distortion = self._load_calibration(args.calibration)
        self.body_extrinsics = None
        self.body_orientation_error = None
        try:
            self.body_extrinsics = load_body_extrinsics(args.landing_config)
        except (OSError, ValueError, KeyError) as exc:
            self.body_orientation_error = str(exc)
            print('Body orientation unavailable: '+str(exc), flush=True)
        self.range_corrections = self._load_range_correction(args.range_correction)
        self.tag_specs = args.tags
        self.tag_quality_gates = args.tag_quality_gates
        self.tag_object_points = {}
        for tag_id, spec in self.tag_specs.items():
            half = spec.size_m / 2.0
            self.tag_object_points[tag_id] = np.array(
                [[-half, half, 0.0], [half, half, 0.0],
                 [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)
        self.camera=Picamera2(0); period=round(1_000_000/args.capture_fps)
        config=self.camera.create_video_configuration(
            main={'format':'YUV420','size':(1280,800)},
            lores={'format':'YUV420','size':(640,400)},
            controls={'FrameDurationLimits':(period,period)},
            buffer_count=6, display=None, encode='lores')
        self.camera.configure(config); self.encoder=MJPEGEncoder(bitrate=args.mjpeg_bitrate); self.stream=LatestMJPEG(args.preview_fps)
        self.camera.start(); self.camera.start_encoder(self.encoder,FileOutput(self.stream),name='lores')
        self.thread=threading.Thread(target=self._analyse,daemon=True); self.thread.start()

    @staticmethod
    def _load_calibration(path):
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        try:
            model = storage.getNode('camera_model').string()
            matrix = storage.getNode('camera_matrix').mat()
            distortion = storage.getNode('distortion_coefficients').mat()
        finally:
            storage.release()
        if model != 'fisheye' or matrix is None or distortion is None:
            raise ValueError(f'Invalid OV9281 fisheye calibration: {path}')
        return np.asarray(matrix, dtype=np.float64), np.asarray(distortion, dtype=np.float64)

    @staticmethod
    def _load_range_correction(path):
        with path.open('r', encoding='utf-8') as correction_file:
            data = json.load(correction_file)
        default={'scale':float(data.get('scale',1.0)),'offset_m':float(data.get('offset_m',0.0)),'status':str(data.get('status','legacy_global'))}
        corrections={None:default}
        for tag_id, values in data.get('per_tag',{}).items():
            corrections[int(tag_id)]={'scale':float(values.get('scale',1.0)),'offset_m':float(values.get('offset_m',0.0)),'status':str(values.get('status','unspecified'))}
        if any(item['scale']<=0 for item in corrections.values()):
            raise ValueError('range correction scale must be positive')
        return corrections

    def set_mode(self, mode):
        if mode not in ('apriltag','calibration'): raise ValueError('invalid mode')
        with self.lock:
            self.mode=mode; self.observation=None; self.observations=[]; self.overlay=None; self.found=False
            self.active_tag_id=None

    def _analyse(self):
        period=1/self.args.analysis_fps
        while not self.stop.is_set():
            started=time.monotonic(); arr=self.camera.capture_array('main'); gray=arr[:800,:1280]
            with self.lock: self.frame=gray.copy()
            with self.lock: mode=self.mode
            if mode=='apriltag': self._tag(gray)
            else: self._chessboard(gray,started)
            now=time.monotonic()
            with self.lock:
                self.frame_time=now; self.analysis_sequence+=1; self.analysis_count+=1; self.capture_count+=1
                elapsed=now-self.rate_at
                if elapsed>=1:
                    self.analysis_fps=self.analysis_count/elapsed; self.capture_fps=self.capture_count/elapsed
                    self.analysis_count=self.capture_count=0; self.rate_at=now
            self.stop.wait(max(0.0,period-(time.monotonic()-started)))

    def _tag(self, gray):
        detections=self.detector.detect(gray,estimate_tag_pose=False)
        best_by_id={}
        for detection in detections:
            tag_id=int(detection.tag_id)
            if tag_id not in self.tag_specs:
                continue
            current=best_by_id.get(tag_id)
            if current is None or float(detection.decision_margin)>float(current.decision_margin):
                best_by_id[tag_id]=detection
        observations=[]
        for tag_id, d in best_by_id.items():
            spec=self.tag_specs[tag_id]
            pts=np.asarray(d.corners,dtype=np.float32).reshape(4,2); center=pts.mean(axis=0)
            undistorted = cv2.fisheye.undistortPoints(
                pts.reshape(-1, 1, 2).astype(np.float64),
                self.camera_matrix, self.distortion,
                R=np.eye(3), P=self.camera_matrix).reshape(-1, 2)
            solved, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                self.tag_object_points[tag_id], undistorted,
                self.camera_matrix, np.zeros((4, 1), dtype=np.float64),
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            candidates = []
            if solved:
                for index, tvec in enumerate(tvecs):
                    translation = np.asarray(tvec, dtype=float).reshape(3)
                    if translation[2] > 0:
                        error = float(np.asarray(errors[index]).reshape(-1)[0]) if errors is not None else float('inf')
                        candidates.append((error, translation, rvecs[index]))
            pose = min(candidates, key=lambda item: item[0]) if candidates else None
            obs={'tag_id':tag_id,'tag_size_m':spec.size_m,'role':spec.role,'decision_margin':float(d.decision_margin),'hamming':int(d.hamming),'center_x_px':float(center[0]),'center_y_px':float(center[1]),'area_px2':abs(float(cv2.contourArea(pts))),'center':[float(center[0]),float(center[1])],'corners_px':pts.tolist()}
            if pose is not None:
                error, translation, rvec = pose
                raw_distance = float(np.linalg.norm(translation))
                correction=self.range_corrections.get(tag_id,self.range_corrections[None])
                corrected_distance = correction['scale'] * raw_distance + correction['offset_m']
                corrected_translation = translation * (corrected_distance / raw_distance)
                obs.update({'x_m':float(corrected_translation[0]),'y_m':float(corrected_translation[1]),'z_m':float(corrected_translation[2]),'distance_m':corrected_distance,'raw_distance_m':raw_distance,'reprojection_error_px':error,'range_scale':correction['scale'],'range_offset_m':correction['offset_m'],'range_correction_status':correction['status']})
                if self.body_extrinsics is not None:
                    try:
                        # IPPE's rotation can be unstable near a 180-degree pose.
                        # Use the iterative planar initialization/refinement for
                        # orientation, without changing the established position
                        # estimate or its distance calibration.
                        orientation_solved, orientation_rvec, orientation_tvec = cv2.solvePnP(
                            self.tag_object_points[tag_id], undistorted,
                            self.camera_matrix, np.zeros((4, 1), dtype=np.float64),
                            flags=cv2.SOLVEPNP_ITERATIVE)
                        if not orientation_solved or float(orientation_tvec[2, 0]) <= 0:
                            raise ValueError('orientation pose unavailable')
                        reprojection = cv2.projectPoints(
                            self.tag_object_points[tag_id], orientation_rvec, orientation_tvec,
                            self.camera_matrix, np.zeros((4, 1), dtype=np.float64))[0].reshape(4, 2)
                        orientation_error = float(np.sqrt(np.mean(np.sum((reprojection-undistorted)**2, axis=1))))
                        gate = self.tag_quality_gates.get(tag_id)
                        maximum_error = gate.maximum_reprojection_error_px if gate is not None else 10.0
                        if not math.isfinite(orientation_error) or orientation_error > maximum_error:
                            raise ValueError('orientation reprojection error exceeds quality limit')
                        obs['orientation'] = body_tag_orientation(
                            orientation_rvec, corrected_translation, spec.role, spec.size_m,
                            *self.body_extrinsics, self.camera_matrix, self.distortion)
                        if obs['orientation'] is not None:
                            obs['orientation']['reprojection_error_px'] = orientation_error
                    except (ValueError, cv2.error) as exc:
                        obs['orientation_error'] = str(exc)
            rejection_reasons = quality_rejection_reasons(obs, self.tag_quality_gates)
            obs['quality_passed'] = not rejection_reasons
            obs['quality_rejection_reasons'] = rejection_reasons
            if obs.get('orientation') is not None:
                obs['orientation']['valid'] = obs['quality_passed']
            observations.append(obs)
        primary=select_primary_tag(
            observations,
            previous_tag_id=self.active_tag_id,
            switch_to_inner_below_m=self.args.switch_to_inner_below_m,
            hysteresis_m=self.args.tag_switch_hysteresis_m,
            quality_gates=self.tag_quality_gates or None,
            prefer_outer=self.args.tag_selection_policy=='outer_first')
        obs=None if primary is None else dict(primary)
        overlay=None if obs is None else obs['corners_px']
        with self.lock:
            self.observation=obs; self.observations=observations; self.overlay=overlay; self.found=obs is not None
            self.active_tag_id=None if obs is None else int(obs['tag_id'])

    def _chessboard(self, gray, now):
        small=cv2.resize(gray,(640,400),interpolation=cv2.INTER_AREA); found,corners=cv2.findChessboardCorners(small,(9,6),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE); refined=None
        if found:
            refined=cv2.cornerSubPix(gray,corners*2.0,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,35,.001)); sig=view_signature(refined)
            distinct=not self.signatures or min(view_distance(sig,old) for old in self.signatures)>=self.args.min_view_change
            if distinct and now-self.last_save>=self.args.save_interval and self.saved<self.args.target_count:
                stem=f'calib_{self.saved:02d}'; cv2.imwrite(str(self.output/f'{stem}.jpg'),gray); marked=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR); cv2.drawChessboardCorners(marked,(9,6),refined,True); cv2.imwrite(str(self.output/f'{stem}_corners.jpg'),marked); self.signatures.append(sig); self.saved+=1; self.last_save=now; print(f'saved {self.saved}/{self.args.target_count}: {stem}.jpg',flush=True)
        with self.lock: self.observation=None; self.observations=[]; self.overlay=None if refined is None else refined.reshape(-1,2).tolist(); self.found=bool(found)

    def status(self):
        with self.lock:
            mode=self.mode; obs=self.observation; observations=list(self.observations); overlay=self.overlay; found=self.found; age=max(0,(time.monotonic()-self.frame_time)*1000) if self.frame_time else 0
            tag_state=(('DUAL TAGS DETECTED' if len(observations)>1 else 'TAG DETECTED') if found else ('TAG CANDIDATE REJECTED' if observations else 'SEARCHING FOR TAG'))
            result={'sensor':'ov9281','mode':mode,'state':tag_state if mode=='apriltag' else ('CHESSBOARD FOUND' if found else 'SEARCHING FOR CHESSBOARD'),'found':found,'analysis_sequence':self.analysis_sequence,'capture_fps':self.capture_fps,'analysis_fps':self.analysis_fps,'encoded_fps':self.stream.fps,'frame_age_ms':age,'saved':self.saved,'target':self.args.target_count,'overlay_points':overlay,'detections':observations,'configured_tags':[spec.as_dict() for spec in self.tag_specs.values()],'configured_quality_gates':{str(tag_id):gate.as_dict() for tag_id,gate in self.tag_quality_gates.items()},'tag_selection_policy':self.args.tag_selection_policy,'pose_enabled':True,'analysis_size':[1280,800],'preview_size':[640,400],'pixel_source':'Y_MONO','tag_family':'tag36h11','tag_size_m':None if obs is None else obs['tag_size_m'],'calibration':str(self.args.calibration),'range_correction':str(self.args.range_correction),'flight_controller_connected':False}
            if obs: result.update(obs)
            result['body_orientation_contract'] = {
                'available': self.body_extrinsics is not None,
                'frame': 'BODY_FRD', 'config_path': str(self.args.landing_config),
                'rotation_camera_optical_to_body_frd': None if self.body_extrinsics is None else self.body_extrinsics[0].tolist(),
                'error': self.body_orientation_error,
                'axes': 'X forward, Y right, Z down',
                'heading': 'pad forward relative to aircraft nose; positive toward aircraft right',
                'yaw_control_enabled': False,
            }
            return result

    def close(self):
        self.stop.set(); self.thread.join(timeout=2); self.camera.stop_encoder(self.encoder); self.camera.stop(); self.camera.close()


class Handler(BaseHTTPRequestHandler):
    state: VisionState
    @classmethod
    def flight_status(cls):
        fallback={'available':False,'flight_controller_connected':False,'armed':False,'mode':'DISCONNECTED','landed_state':'UNKNOWN','horizontal_direction':'UNKNOWN','lateral_direction':'UNKNOWN','vertical_direction':'UNKNOWN','body_velocity_mps':None,'latest_sent_velocity_mps':None}
        try:
            with urllib.request.urlopen(cls.state.args.flight_status_url,timeout=cls.state.args.flight_status_timeout_s) as response:
                result=json.loads(response.read())
            return result if isinstance(result,dict) else fallback
        except (OSError,ValueError,json.JSONDecodeError,urllib.error.URLError):
            return fallback
    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/','/index.html'): return self.send_data(200,'text/html; charset=utf-8',HTML)
        if path=='/api/status':
            result=self.state.status()
            result['camera_owns_mavlink']=False
            # Legacy field describes the camera's own connection, not telemetry.
            result['flight_controller_connected']=False
            result['flight']=self.flight_status()
            result['flight_telemetry_connected']=bool(result['flight'].get('flight_controller_connected',False))
            return self.send_data(200,'application/json',json.dumps(result).encode())
        if path=='/api/flight-status': return self.send_data(200,'application/json',json.dumps(self.flight_status()).encode())
        if path=='/api/analysis.jpg':
            with self.state.lock: frame=None if self.state.frame is None else self.state.frame.copy()
            if frame is None: return self.send_error(503)
            encoded,image=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,92])
            if not encoded: return self.send_error(500)
            return self.send_data(200,'image/jpeg',image.tobytes())
        if path=='/stream.mjpg':
            self.send_response(200); self.send_header('Cache-Control','no-store, no-cache, must-revalidate'); self.send_header('Pragma','no-cache'); self.send_header('Content-Type','multipart/x-mixed-replace; boundary=FRAME'); self.end_headers(); sequence=-1
            try:
                while True:
                    with self.state.stream.condition:
                        self.state.stream.condition.wait_for(lambda:self.state.stream.sequence!=sequence,timeout=2); frame=self.state.stream.frame; sequence=self.state.stream.sequence
                    if frame: self.wfile.write(b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n'); self.wfile.flush()
            except (BrokenPipeError,ConnectionResetError): pass
            return
        self.send_error(404)
    def do_POST(self):
        if self.path!='/api/mode': return self.send_error(404)
        try:
            length=int(self.headers.get('Content-Length','0')); data=json.loads(self.rfile.read(length)); self.state.set_mode(str(data.get('mode',''))); return self.send_data(200,'application/json',json.dumps({'ok':True,'mode':self.state.mode}).encode())
        except (ValueError,json.JSONDecodeError) as exc: return self.send_data(400,'application/json',json.dumps({'ok':False,'error':str(exc)}).encode())
    def send_data(self,status,content_type,data):
        self.send_response(status); self.send_header('Content-Type',content_type); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)
    def log_message(self,*_): pass


def parse_args():
    p=argparse.ArgumentParser(); p.add_argument('--host',default='0.0.0.0'); p.add_argument('--port',type=int,default=8765); p.add_argument('--mode',choices=('apriltag','calibration'),default='apriltag')
    p.add_argument('--tag-specs',default='0:0.100:outer,1:0.020:inner',help='comma-separated ID:SIZE_M:ROLE entries')
    p.add_argument('--tag-quality-specs',default='',help='comma-separated ID:MIN_MARGIN:MAX_HAMMING:MAX_REPROJECTION_PX entries')
    p.add_argument('--tag-selection-policy',choices=('distance_hysteresis','outer_first'),default='distance_hysteresis')
    p.add_argument('--tag-id',type=int,default=None,help='legacy single-tag override; requires --tag-size-m')
    p.add_argument('--tag-size-m',type=float,default=None,help='legacy single-tag override; requires --tag-id')
    p.add_argument('--switch-to-inner-below-m',type=float,default=0.35); p.add_argument('--tag-switch-hysteresis-m',type=float,default=0.05)
    p.add_argument('--flight-status-url',default='http://127.0.0.1:8766/api/status'); p.add_argument('--flight-status-timeout-s',type=float,default=0.15)
    p.add_argument('--landing-config',type=Path,default=Path.home()/'air_ground_landing/config/moving_landing.prototype.json',help='same extrinsics file used by landing and IBVS controllers')
    p.add_argument('--calibration',type=Path,default=Path.home()/'ov9281_debug/ov9281_calibration_fisheye_run2_flat_17mm.yaml'); p.add_argument('--range-correction',type=Path,default=Path.home()/'ov9281_debug/ov9281_range_correction_20260813.json'); p.add_argument('--capture-fps',type=float,default=30); p.add_argument('--analysis-fps',type=float,default=10); p.add_argument('--preview-fps',type=float,default=10); p.add_argument('--mjpeg-bitrate',type=int,default=6000000); p.add_argument('--collect-output',type=Path,default=Path.home()/'ov9281_calibration_run2_flat_17mm'); p.add_argument('--target-count',type=int,default=20); p.add_argument('--save-interval',type=float,default=1.2); p.add_argument('--min-view-change',type=float,default=.8)
    args=p.parse_args()
    if (args.tag_id is None)!=(args.tag_size_m is None):
        p.error('--tag-id and --tag-size-m must be supplied together')
    args.tags=parse_tag_specs(args.tag_specs) if args.tag_id is None else parse_tag_specs(f'{args.tag_id}:{args.tag_size_m}:outer')
    args.tag_quality_gates=parse_tag_quality_specs(args.tag_quality_specs)
    if args.tag_quality_gates and set(args.tag_quality_gates)!=set(args.tags):
        p.error('--tag-quality-specs must configure exactly the IDs in --tag-specs')
    if args.switch_to_inner_below_m<=0 or args.tag_switch_hysteresis_m<0 or args.tag_switch_hysteresis_m>=args.switch_to_inner_below_m:
        p.error('dual-tag switch distance/hysteresis is invalid')
    if args.flight_status_timeout_s<=0:
        p.error('--flight-status-timeout-s must be positive')
    return args


def main():
    args=parse_args(); state=VisionState(args); Handler.state=state; server=ThreadingHTTPServer((args.host,args.port),Handler); print(f'OV9281 unified console ready: http://{args.host}:{args.port}/',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close(); state.close()


if __name__=='__main__': main()
