"use client";

/* eslint-disable @next/next/no-img-element -- camera frames are live ROS JPEG endpoints */

import {
  Activity,
  Camera,
  Car,
  ChevronDown,
  CircleAlert,
  Crosshair,
  Gauge,
  LockKeyhole,
  Navigation,
  Octagon,
  Pause,
  Plane,
  Play,
  Radar,
  Radio,
  RotateCcw,
  Send,
  Settings2,
  ShieldAlert,
  UnlockKeyhole,
  Wifi,
  WifiOff,
} from "lucide-react";
import { FormEvent, useCallback, useEffect, useRef, useState } from "react";

type JsonRecord = Record<string, unknown>;

type Snapshot = {
  gateway?: {
    ok?: boolean;
    uptime_s?: number;
    command_enabled?: boolean;
    token_required?: boolean;
    production_mode?: boolean;
    request_id_required?: boolean;
    operator_identity_required?: boolean;
  };
  readiness?: { ready?: boolean; system_state?: string; emergency_stop?: boolean };
  system?: JsonRecord;
  mission?: JsonRecord;
  mavlink?: JsonRecord;
  perception?: JsonRecord;
  docking?: JsonRecord;
  navigation?: JsonRecord;
  command_mux?: JsonRecord;
  ugv_control_mux?: JsonRecord;
  chassis_adapter?: JsonRecord;
  ugv_gateway?: JsonRecord;
  optical_flow?: JsonRecord;
  ugv?: { pose?: number[] | null; speed_mps?: number; minimum_scan_m?: number | null };
  paths?: { ugv_global?: number[][] };
  cameras?: Record<string, { ready?: boolean; age_s?: number }>;
  topic_ages_s?: Record<string, number>;
  world?: {
    name?: string;
    bounds?: number[];
    no_fly_zones?: Array<{ name: string; x: number; y: number; radius: number }>;
    height_limit_zones?: Array<{
      name: string;
      x_min: number;
      x_max: number;
      y_min: number;
      y_max: number;
      max_z: number;
    }>;
  };
};

const EMPTY_SNAPSHOT: Snapshot = {
  gateway: { ok: false, command_enabled: false },
  readiness: { ready: false, system_state: "UNAVAILABLE", emergency_stop: false },
  system: { state: "STARTING", ready: false, emergency_stop: false, faults: [] },
  mission: { state: "IDLE", reason: "等待 ROS 2 网关", active: false, paused: false },
  mavlink: {
    connected: false,
    armed: false,
    mode: "UNKNOWN",
    landed_state_name: "MAV_LANDED_STATE_UNDEFINED",
    relative_alt_m: null,
  },
  perception: { healthy: false, minimum_obstacle_m: null, sensors: {}, sectors_m: {} },
  docking: { state: "disabled", capture_ready: false, tag_visible: false },
  navigation: { active: false, reason: "waiting", position: [0, 0, 0], goal: null },
  command_mux: { mode: "stopped" },
  optical_flow: { healthy: false, reason: "waiting" },
  ugv: { pose: [-9, -6, 0], speed_mps: 0, minimum_scan_m: null },
  paths: { ugv_global: [] },
  cameras: {},
  world: {
    name: "air_ground_cooperative",
    bounds: [-15, 15, -11, 11],
    no_fly_zones: [
      { name: "tower_nfz", x: -3.7, y: -1.2, radius: 1.55 },
      { name: "destination_nfz", x: 8.7, y: 6.9, radius: 1.35 },
    ],
    height_limit_zones: [
      { name: "low_ceiling", x_min: 0.75, x_max: 5.25, y_min: -4.6, y_max: -2.4, max_z: 2.3 },
    ],
  },
};

const MISSION_PHASES = [
  { label: "远端搜索", states: ["RELEASE_REMOTE_DOCK", "WAIT_AUTOPILOT", "ARM_INITIAL", "TAKEOFF_INITIAL", "NAVIGATE_TO_START_DOCK"] },
  { label: "首次降落", states: ["DOCK_AT_START", "LATCH_AT_START", "DWELL_AT_START"] },
  { label: "空地分离", states: ["RELEASE_FOR_TRANSIT", "ARM_FOR_TRANSIT", "TAKEOFF_FOR_TRANSIT"] },
  { label: "并行避障", states: ["PARALLEL_TRANSIT"] },
  { label: "静态汇合", states: ["DOCK_STOPPED", "LATCH_STOPPED", "DWELL_STOPPED"] },
  { label: "移动跟飞", states: ["RELEASE_FOR_FOLLOW", "ARM_FOR_FOLLOW", "TAKEOFF_FOR_FOLLOW", "FOLLOW_MOVING_UGV"] },
  { label: "移动降落", states: ["DOCK_MOVING", "LATCH_MOVING"] },
  { label: "联合减速", states: ["RIDE_AND_DECELERATE", "COMPLETE"] },
];

const SENSOR_LABELS: Array<[string, string]> = [
  ["lidar3d_scan", "3D LiDAR扫描"],
  ["lidar3d", "3D LiDAR"],
  ["stereo_depth", "双目深度"],
  ["stereo_left", "OV9281左目"],
  ["stereo_right", "OV9281右目"],
  ["downward_camera", "OV9281下视"],
  ["downward_tof", "下视ToF"],
];

function record(value: unknown): JsonRecord {
  return value && typeof value === "object" ? (value as JsonRecord) : {};
}

function numberValue(value: unknown, fallback = 0): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function textValue(value: unknown, fallback = "—"): string {
  return typeof value === "string" && value ? value : fallback;
}

function boolValue(value: unknown): boolean {
  return value === true;
}

function formatNumber(value: unknown, digits = 1, suffix = ""): string {
  return typeof value === "number" && Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "—";
}

function defaultApiBase(): string {
  if (typeof window === "undefined") return "http://localhost:8765";
  const configured = process.env.NEXT_PUBLIC_ROS_API;
  if (configured) return configured.replace(/\/$/, "");
  // WSL's localhost forwarding may listen only on IPv4 while Chromium resolves
  // localhost to ::1. Development connects directly to the ROS gateway;
  // production uses Nginx's same-origin /api proxy and never exposes 8765.
  const hostname = window.location.hostname === "localhost" ? "127.0.0.1" : window.location.hostname;
  const developmentPort = window.location.port === "3000" || window.location.port === "3001";
  return developmentPort
    ? `${window.location.protocol}//${hostname}:8765`
    : window.location.origin;
}

function storedControlToken(): string {
  return typeof window === "undefined" ? "" : window.sessionStorage.getItem("air-ground-token") || "";
}

function storedOperatorId(): string {
  return typeof window === "undefined" ? "" : window.localStorage.getItem("air-ground-operator") || "";
}

function useGateway() {
  const [snapshot, setSnapshot] = useState<Snapshot>(EMPTY_SNAPSHOT);
  const [connected, setConnected] = useState(false);
  const [lastUpdate, setLastUpdate] = useState(0);
  const [token, setToken] = useState(storedControlToken);
  const [operatorId, setOperatorId] = useState(storedOperatorId);
  const [apiBase] = useState(defaultApiBase);

  useEffect(() => {
    const stream = new EventSource(`${apiBase}/api/events`);
    stream.addEventListener("status", (event) => {
      try {
        setSnapshot(JSON.parse((event as MessageEvent).data) as Snapshot);
        setConnected(true);
        setLastUpdate(Date.now());
      } catch {
        setConnected(false);
      }
    });
    stream.onerror = () => setConnected(false);
    return () => stream.close();
  }, [apiBase]);

  const updateToken = useCallback((value: string) => {
    setToken(value);
    window.sessionStorage.setItem("air-ground-token", value);
  }, []);

  const updateOperatorId = useCallback((value: string) => {
    setOperatorId(value);
    window.localStorage.setItem("air-ground-operator", value);
  }, []);

  const command = useCallback(
    async (path: string, body: JsonRecord = {}) => {
      const response = await fetch(`${apiBase}${path}`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "X-Request-ID": window.crypto.randomUUID(),
          ...(operatorId ? { "X-Operator-ID": operatorId } : {}),
          ...(token ? { "X-Control-Token": token } : {}),
        },
        body: JSON.stringify(body),
      });
      const payload = (await response.json()) as JsonRecord;
      if (!response.ok || payload.accepted === false) {
        throw new Error(textValue(payload.error, `HTTP ${response.status}`));
      }
      return payload;
    },
    [apiBase, operatorId, token],
  );

  return { snapshot, connected, lastUpdate, apiBase, token, operatorId, updateToken, updateOperatorId, command };
}

function MissionMap({ snapshot }: { snapshot: Snapshot }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;

    const render = () => {
      const bounds = canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.max(1, Math.floor(bounds.width * ratio));
      canvas.height = Math.max(1, Math.floor(bounds.height * ratio));
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      const width = bounds.width;
      const height = bounds.height;
      const padding = 28;
      const minX = -15;
      const maxX = 15;
      const minY = -11;
      const maxY = 11;
      const scale = Math.min((width - padding * 2) / (maxX - minX), (height - padding * 2) / (maxY - minY));
      const originX = (width - (maxX - minX) * scale) / 2;
      const originY = (height - (maxY - minY) * scale) / 2;
      const toX = (x: number) => originX + (x - minX) * scale;
      const toY = (y: number) => originY + (maxY - y) * scale;

      context.fillStyle = "#0b1116";
      context.fillRect(0, 0, width, height);
      context.fillStyle = "#10191f";
      context.fillRect(originX, originY, (maxX - minX) * scale, (maxY - minY) * scale);

      context.strokeStyle = "rgba(91, 118, 130, .16)";
      context.lineWidth = 1;
      for (let x = minX; x <= maxX; x += 2) {
        context.beginPath();
        context.moveTo(toX(x), toY(minY));
        context.lineTo(toX(x), toY(maxY));
        context.stroke();
      }
      for (let y = minY; y <= maxY; y += 2) {
        context.beginPath();
        context.moveTo(toX(minX), toY(y));
        context.lineTo(toX(maxX), toY(y));
        context.stroke();
      }

      const obstacles: Array<[number, number, number, number]> = [
        [-5.5, -3.8, 2.9, 2.6],
        [-2, 2, 2, 4.8],
        [3.1, 0.2, 2.3, 3.2],
        [7.6, 3.7, 2.8, 2.2],
        [8, -1.8, 2.5, 3.5],
        [1, -3.8, 1.2, 0.9],
      ];
      context.fillStyle = "#27353d";
      context.strokeStyle = "#41545e";
      obstacles.forEach(([x, y, w, h]) => {
        context.fillRect(toX(x - w / 2), toY(y + h / 2), w * scale, h * scale);
        context.strokeRect(toX(x - w / 2), toY(y + h / 2), w * scale, h * scale);
      });

      const ceiling = snapshot.world?.height_limit_zones?.[0];
      if (ceiling) {
        context.fillStyle = "rgba(240, 170, 30, .10)";
        context.strokeStyle = "rgba(245, 185, 50, .72)";
        context.setLineDash([6, 5]);
        context.fillRect(toX(ceiling.x_min), toY(ceiling.y_max), (ceiling.x_max - ceiling.x_min) * scale, (ceiling.y_max - ceiling.y_min) * scale);
        context.strokeRect(toX(ceiling.x_min), toY(ceiling.y_max), (ceiling.x_max - ceiling.x_min) * scale, (ceiling.y_max - ceiling.y_min) * scale);
        context.setLineDash([]);
        context.fillStyle = "#efba52";
        context.font = "10px var(--font-geist-mono)";
        context.fillText(`限高 ${ceiling.max_z.toFixed(1)}m`, toX(ceiling.x_min) + 6, toY(ceiling.y_max) + 14);
      }

      snapshot.world?.no_fly_zones?.forEach((zone) => {
        context.fillStyle = "rgba(240, 70, 55, .10)";
        context.strokeStyle = "rgba(255, 83, 67, .78)";
        context.lineWidth = 1.5;
        context.setLineDash([4, 4]);
        context.beginPath();
        context.arc(toX(zone.x), toY(zone.y), zone.radius * scale, 0, Math.PI * 2);
        context.fill();
        context.stroke();
        context.setLineDash([]);
        context.fillStyle = "#ff7163";
        context.font = "10px var(--font-geist-mono)";
        context.fillText("禁飞", toX(zone.x) - 11, toY(zone.y) + 3);
      });

      const path = snapshot.paths?.ugv_global || [];
      if (path.length > 1) {
        context.strokeStyle = "#e9ad43";
        context.lineWidth = 2;
        context.beginPath();
        path.forEach(([x, y], index) => {
          if (index === 0) context.moveTo(toX(x), toY(y));
          else context.lineTo(toX(x), toY(y));
        });
        context.stroke();
      }

      const navigation = record(snapshot.navigation);
      const rawUav = Array.isArray(navigation.position)
        ? (navigation.position as number[])
        : Array.isArray(snapshot.mavlink?.local_position_enu_m)
          ? (snapshot.mavlink?.local_position_enu_m as number[])
          : [0, 0, 0];
      const uav = rawUav.map((value) => numberValue(value));
      const ugv = snapshot.ugv?.pose || [-9, -6, 0];
      const ugvYaw = numberValue(ugv[2]);

      context.save();
      context.translate(toX(numberValue(ugv[0], -9)), toY(numberValue(ugv[1], -6)));
      context.rotate(-ugvYaw);
      context.fillStyle = "#f0ad3d";
      context.fillRect(-10, -6, 20, 12);
      context.fillStyle = "#151b1e";
      context.fillRect(2, -4, 7, 8);
      context.restore();
      context.fillStyle = "#f4c66b";
      context.font = "11px var(--font-geist-mono)";
      context.fillText("UGV", toX(numberValue(ugv[0], -9)) + 12, toY(numberValue(ugv[1], -6)) - 8);

      const ux = toX(numberValue(uav[0]));
      const uy = toY(numberValue(uav[1]));
      context.strokeStyle = "#35d5df";
      context.lineWidth = 2.5;
      context.beginPath();
      context.moveTo(ux - 11, uy - 11);
      context.lineTo(ux + 11, uy + 11);
      context.moveTo(ux + 11, uy - 11);
      context.lineTo(ux - 11, uy + 11);
      context.stroke();
      context.beginPath();
      context.arc(ux, uy, 5, 0, Math.PI * 2);
      context.fillStyle = "#35d5df";
      context.fill();
      context.font = "11px var(--font-geist-mono)";
      context.fillText(`UAV ${formatNumber(uav[2], 1, "m")}`, ux + 14, uy - 8);

      const rawGoal = navigation.goal;
      if (Array.isArray(rawGoal) && rawGoal.length >= 2) {
        const gx = toX(numberValue(rawGoal[0]));
        const gy = toY(numberValue(rawGoal[1]));
        context.strokeStyle = "#ecf5f4";
        context.lineWidth = 1.5;
        context.beginPath();
        context.arc(gx, gy, 7, 0, Math.PI * 2);
        context.stroke();
        context.beginPath();
        context.moveTo(gx - 10, gy);
        context.lineTo(gx + 10, gy);
        context.moveTo(gx, gy - 10);
        context.lineTo(gx, gy + 10);
        context.stroke();
      }

      context.strokeStyle = "rgba(164, 190, 199, .45)";
      context.lineWidth = 1;
      context.strokeRect(originX, originY, (maxX - minX) * scale, (maxY - minY) * scale);
      context.fillStyle = "#78909a";
      context.font = "10px var(--font-geist-mono)";
      context.fillText("N", originX + 10, originY + 16);
      context.beginPath();
      context.moveTo(originX + 13, originY + 34);
      context.lineTo(originX + 13, originY + 20);
      context.strokeStyle = "#78909a";
      context.stroke();
    };

    render();
    const observer = new ResizeObserver(render);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, [snapshot]);

  return <canvas ref={canvasRef} className="mission-canvas" aria-label="空地协同任务二维态势图" />;
}

function Metric({ label, value, tone = "normal" }: { label: string; value: string; tone?: "normal" | "good" | "warn" | "bad" }) {
  return (
    <div className={`metric metric-${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function CameraTile({ apiBase, name, label, ready, tick, className = "" }: { apiBase: string; name: string; label: string; ready: boolean; tick: number; className?: string }) {
  return (
    <figure className={`camera-tile ${className}`}>
      <figcaption>
        <span><Camera size={13} /> {label}</span>
        <span className={ready ? "live-dot" : "offline-dot"}>{ready ? "LIVE" : "WAIT"}</span>
      </figcaption>
      {ready ? (
        <img src={`${apiBase}/api/camera/${name}.jpg?t=${tick}`} alt={`${label}实时画面`} />
      ) : (
        <div className="camera-empty"><Camera size={25} /><span>等待图像话题</span></div>
      )}
      <div className="camera-reticle" aria-hidden="true" />
    </figure>
  );
}

function App() {
  const {
    snapshot,
    connected,
    lastUpdate,
    apiBase,
    token,
    operatorId,
    updateToken,
    updateOperatorId,
    command,
  } = useGateway();
  const [cameraTick, setCameraTick] = useState(0);
  const [notice, setNotice] = useState<{ text: string; error?: boolean } | null>(null);
  const [uavGoal, setUavGoal] = useState({ x: 0, y: 0, z: 3 });
  const [ugvGoal, setUgvGoal] = useState({ x: 4.8, y: 5.2, yaw: 0 });
  const [confirmAction, setConfirmAction] = useState<null | "abort" | "reset">(null);
  const teleopTimer = useRef<number | null>(null);
  const teleopBusy = useRef(false);
  const teleopStopPending = useRef(false);

  useEffect(() => {
    const timer = window.setInterval(() => setCameraTick(Date.now()), 400);
    return () => window.clearInterval(timer);
  }, []);

  const run = useCallback(
    async (path: string, body: JsonRecord = {}, success = "命令已提交") => {
      try {
        await command(path, body);
        setNotice({ text: success });
      } catch (error) {
        setNotice({ text: error instanceof Error ? error.message : "命令失败", error: true });
      }
      window.setTimeout(() => setNotice(null), 3200);
    },
    [command],
  );

  const mission = record(snapshot.mission);
  const missionPlan = record(mission.mission_plan);
  const mavlink = record(snapshot.mavlink);
  const perception = record(snapshot.perception);
  const docking = record(snapshot.docking);
  const navigationState = record(snapshot.navigation);
  const mux = record(snapshot.command_mux);
  const system = record(snapshot.system);
  const ugvControlMux = record(snapshot.ugv_control_mux);
  const chassisAdapter = record(snapshot.chassis_adapter);
  const opticalFlow = record(snapshot.optical_flow);
  const sensors = record(perception.sensors);
  const ranges = record(perception.sectors_m);
  const missionState = textValue(mission.state, "IDLE");
  const missionActive = boolValue(mission.active);
  const missionPaused = boolValue(mission.paused);
  const missionPlanCommissioned = boolValue(missionPlan.commissioned);
  const armed = boolValue(mavlink.armed);
  const landedState = textValue(mavlink.landed_state_name, "UNDEFINED").replace(
    "MAV_LANDED_STATE_",
    "",
  );
  const perceptionHealthy = boolValue(perception.healthy);
  const systemReady = boolValue(system.ready);
  const systemEmergency = boolValue(system.emergency_stop);
  const systemState = textValue(system.state, "STARTING");
  const systemFaults = Array.isArray(system.faults) ? system.faults.map(record) : [];
  const currentPhase = Math.max(0, MISSION_PHASES.findIndex((phase) => phase.states.includes(missionState)));
  const commandEnabled = snapshot.gateway?.command_enabled !== false;
  const teleopAllowed = commandEnabled && systemReady && !systemEmergency && (!missionActive || missionPaused);
  const updatedAgo = lastUpdate ? Math.max(0, (cameraTick - lastUpdate) / 1000) : null;

  const cameraReady = useCallback((name: string) => snapshot.cameras?.[name]?.ready === true, [snapshot.cameras]);

  const submitUav = (event: FormEvent) => {
    event.preventDefault();
    run("/api/uav/goal", uavGoal, "无人机目标已发布");
  };
  const submitUgv = (event: FormEvent) => {
    event.preventDefault();
    run("/api/ugv/goal", ugvGoal, "Nav2 目标已发送");
  };

  const issueTeleop = useCallback(
    async (linear: number, angular: number, reportError = false) => {
      if (teleopBusy.current) {
        if (linear === 0 && angular === 0) teleopStopPending.current = true;
        return;
      }
      teleopBusy.current = true;
      try {
        await command("/api/ugv/teleop", { linear, angular });
      } catch (error) {
        if (reportError) {
          setNotice({ text: error instanceof Error ? error.message : "无人车点动失败", error: true });
        }
      } finally {
        teleopBusy.current = false;
        if (teleopStopPending.current) {
          teleopStopPending.current = false;
          try {
            await command("/api/ugv/teleop", { linear: 0, angular: 0 });
          } catch {
            // The ROS gateway independently zeros teleop after 350 ms.
          }
        }
      }
    },
    [command],
  );

  const stopTeleop = useCallback(() => {
    if (teleopTimer.current !== null) window.clearInterval(teleopTimer.current);
    teleopTimer.current = null;
    void issueTeleop(0, 0, true);
  }, [issueTeleop]);

  const startTeleop = (linear: number, angular: number) => {
    if (!teleopAllowed) return;
    if (teleopTimer.current !== null) window.clearInterval(teleopTimer.current);
    void issueTeleop(linear, angular, true);
    teleopTimer.current = window.setInterval(() => {
      void issueTeleop(linear, angular);
    }, 120);
  };

  useEffect(() => {
    return () => {
      if (teleopTimer.current !== null) window.clearInterval(teleopTimer.current);
      teleopStopPending.current = true;
      void issueTeleop(0, 0);
    };
  }, [issueTeleop]);

  return (
    <main className="ops-shell">
      <header className="topbar">
        <div className="brand-block">
          <div className="brand-mark"><Plane size={19} /><Car size={17} /></div>
          <div><strong>ASTRA 联合任务台</strong><span>AIR · GROUND OPERATIONS</span></div>
        </div>
        <div className="top-status">
          <span className={`system-pill ${systemEmergency ? "emergency" : systemReady ? "ready" : "degraded"}`}>
            <ShieldAlert size={14} />{systemState}
          </span>
          <span className={`connection-pill ${connected ? "online" : "offline"}`}>
            {connected ? <Wifi size={14} /> : <WifiOff size={14} />}
            {connected ? "ROS 2 在线" : "网关离线"}
          </span>
          <span className="clock-readout">{updatedAgo === null ? "无数据" : `${updatedAgo.toFixed(1)}s 前更新`}</span>
          <span className="world-name">{snapshot.world?.name || "—"}</span>
        </div>
      </header>

      {notice && <div className={`notice ${notice.error ? "notice-error" : ""}`}>{notice.text}</div>}
      {!systemReady && (
        <div className={`safety-banner ${systemEmergency ? "emergency" : ""}`} role="status">
          <strong>{systemEmergency ? "安全闭锁已触发" : "系统尚未达到运行就绪"}</strong>
          <span>{systemFaults.length ? systemFaults.slice(0, 3).map((fault) => textValue(fault.code)).join(" · ") : "等待安全监督器完成全链路检查"}</span>
        </div>
      )}

      <section className="primary-grid">
        <section className="map-panel panel">
          <header className="panel-heading">
            <div><Navigation size={16} /><span>联合态势 / MISSION MAP</span></div>
            <div className="map-legend"><span className="legend-uav">UAV</span><span className="legend-ugv">UGV</span><span className="legend-nfz">禁飞区</span><span className="legend-ceiling">限高</span></div>
          </header>
          <MissionMap snapshot={snapshot} />
          <div className="map-overlay top-left">
            <span>控制源</span><strong>{textValue(ugvControlMux.authority, "none")} / {textValue(mux.mode, "stopped")}</strong>
          </div>
          <div className="map-overlay bottom-left">
            <span>UAV 目标</span><strong>{Array.isArray(navigationState.goal) ? (navigationState.goal as number[]).map((value) => numberValue(value).toFixed(1)).join(" / ") : "未设置"}</strong>
          </div>
        </section>

        <aside className="mission-panel panel">
          <header className="panel-heading">
            <div><Activity size={16} /><span>任务执行链</span></div>
            <span className={`state-badge ${missionState === "FAULT" || missionState === "ABORTED" ? "bad" : missionActive ? "active" : ""}`}>{missionState}</span>
          </header>
          <div className="mission-summary">
            <span>{textValue(mission.reason, "ready")} · PLAN {textValue(missionPlan.id, "UNCOMMISSIONED")}</span>
            <strong>{formatNumber(mission.elapsed_s, 0, " s")}</strong>
          </div>
          <ol className="timeline">
            {MISSION_PHASES.map((phase, index) => {
              const complete = missionState === "COMPLETE" || index < currentPhase;
              const active = index === currentPhase && missionState !== "IDLE";
              return (
                <li key={phase.label} className={`${complete ? "complete" : ""} ${active ? "active" : ""}`}>
                  <span className="timeline-index">{complete ? "✓" : String(index + 1).padStart(2, "0")}</span>
                  <div><strong>{phase.label}</strong><span>{active ? missionState : complete ? "已通过" : "待执行"}</span></div>
                </li>
              );
            })}
          </ol>
          <div className="mission-controls">
            <button className="control primary" disabled={!commandEnabled || !systemReady || systemEmergency || !missionPlanCommissioned || (missionActive && !missionPaused)} onClick={() => run("/api/mission/start", {}, "协同任务已启动")}><Play size={16} />启动任务</button>
            <button className="control" disabled={!missionActive} onClick={() => run(missionPaused ? "/api/mission/resume" : "/api/mission/pause", {}, missionPaused ? "任务已恢复" : "任务已暂停")}>{missionPaused ? <Play size={16} /> : <Pause size={16} />}{missionPaused ? "继续" : "暂停"}</button>
            <button className="control danger" disabled={!missionActive} onClick={() => setConfirmAction("abort")}><Octagon size={16} />中止</button>
          </div>
          <div className="sim-controls">
            <span>Gazebo</span>
            <button title="暂停物理仿真" aria-label="暂停 Gazebo" onClick={() => run("/api/sim/pause", {}, "Gazebo 已暂停")}><Pause size={14} /></button>
            <button title="继续物理仿真" aria-label="继续 Gazebo" onClick={() => run("/api/sim/resume", {}, "Gazebo 已继续")}><Play size={14} /></button>
            <button title="重置仿真世界" aria-label="重置 Gazebo" onClick={() => setConfirmAction("reset")}><RotateCcw size={14} /></button>
          </div>
        </aside>
      </section>

      <section className="vehicle-strip">
        <article className="vehicle-card uav-card">
          <header><span><Plane size={17} />无人机 / UAV</span><span className={armed ? "status-good" : "status-muted"}>{armed ? <LockKeyhole size={13} /> : <UnlockKeyhole size={13} />}{armed ? "ARMED" : "SAFE"} · {landedState}</span></header>
          <div className="metrics-row">
            <Metric label="飞控模式" value={textValue(mavlink.mode, "UNKNOWN")} tone={textValue(mavlink.mode) === "GUIDED" ? "good" : "normal"} />
            <Metric label="相对高度" value={formatNumber(mavlink.relative_alt_m, 2, " m")} />
            <Metric label="电池" value={formatNumber(mavlink.battery_remaining_pct, 0, "%")} tone={numberValue(mavlink.battery_remaining_pct, 100) < 25 ? "bad" : "normal"} />
            <Metric label="最近障碍" value={formatNumber(perception.minimum_obstacle_m, 2, " m")} tone={numberValue(perception.minimum_obstacle_m, 99) < 1 ? "bad" : "normal"} />
          </div>
        </article>
        <article className="vehicle-card ugv-card">
          <header><span><Car size={17} />无人车 / HUNTER</span><span className="status-good"><Radio size={13} />NAV2</span></header>
          <div className="metrics-row">
            <Metric label="闭环速度" value={formatNumber(snapshot.ugv?.speed_mps, 2, " m/s")} />
            <Metric label="激光余量" value={formatNumber(snapshot.ugv?.minimum_scan_m, 2, " m")} tone={numberValue(snapshot.ugv?.minimum_scan_m, 99) < 0.8 ? "bad" : "normal"} />
            <Metric label="目标状态" value={textValue(mission.ugv_goal_status, "idle")} />
            <Metric label="对接距离" value={formatNumber(docking.separation_m, 2, " m")} tone={boolValue(docking.capture_ready) ? "good" : "normal"} />
          </div>
        </article>
      </section>

      <section className="data-grid">
        <section className="camera-section panel">
          <header className="panel-heading"><div><Camera size={16} /><span>视觉与对接</span></div><span className="subtle">JPEG 低延迟预览</span></header>
          <div className="camera-grid">
            <CameraTile apiBase={apiBase} name="stereo_left" label="前视OV9281左目" ready={cameraReady("stereo_left")} tick={cameraTick} className="camera-main" />
            <CameraTile apiBase={apiBase} name="landing" label="下视 / AprilTag" ready={cameraReady("landing")} tick={cameraTick} />
            <CameraTile apiBase={apiBase} name="stereo_right" label="前视OV9281右目" ready={cameraReady("stereo_right")} tick={cameraTick} />
            <CameraTile apiBase={apiBase} name="ugv" label="车载前视" ready={cameraReady("ugv")} tick={cameraTick} />
          </div>
        </section>

        <section className="sensor-panel panel">
          <header className="panel-heading"><div><Radar size={16} /><span>机载感知健康</span></div><span className={perceptionHealthy ? "health-good" : "health-bad"}>{perceptionHealthy ? "FUSED" : "DEGRADED"}</span></header>
          <div className="sensor-matrix">
            {SENSOR_LABELS.map(([key, label]) => {
              const state = record(sensors[key]);
              const healthy = boolValue(state.healthy);
              return <div className={`sensor-cell ${healthy ? "healthy" : "stale"}`} key={key}><span>{label}</span><strong>{formatNumber(state.rate_hz, 1, " Hz")}</strong><i>{healthy ? "在线" : "等待"}</i></div>;
            })}
          </div>
          <div className="range-bars">
            {(["front", "left", "up", "down", "right", "rear"] as const).map((direction) => {
              const value = numberValue(ranges[direction], 6);
              return <div key={direction}><span>{direction.toUpperCase()}</span><div><i style={{ width: `${Math.min(100, (value / 6) * 100)}%` }} /></div><strong>{formatNumber(ranges[direction], 2, "m")}</strong></div>;
            })}
          </div>
        </section>

        <section className="safety-panel panel">
          <header className="panel-heading"><div><ShieldAlert size={16} /><span>系统安全与约束</span></div><span className={systemReady ? "health-good" : "health-bad"}>{systemState}</span></header>
          <div className="safety-readout">
            <div><span>运行就绪</span><strong className={systemReady ? "good-text" : "bad-text"}>{systemReady ? "READY" : "INHIBITED"}</strong></div>
            <div><span>任务计划</span><strong className={missionPlanCommissioned ? "good-text" : "bad-text"}>{missionPlanCommissioned ? textValue(missionPlan.id, "COMMISSIONED") : "UNCOMMISSIONED"}</strong></div>
            <div><span>系统急停</span><strong className={systemEmergency ? "bad-text" : "good-text"}>{systemEmergency ? "LATCHED" : "CLEAR"}</strong></div>
            <div><span>UGV 控制权</span><strong>{textValue(ugvControlMux.authority, "none")}</strong></div>
            <div><span>底盘安全门</span><strong className={boolValue(chassisAdapter.speed_gate_open) ? "good-text" : "muted-text"}>{boolValue(chassisAdapter.speed_gate_open) ? "OPEN" : "CLOSED"}</strong></div>
            <div><span>禁飞区检查</span><strong className={textValue(navigationState.reason).includes("airspace") ? "bad-text" : "good-text"}>{textValue(navigationState.reason).includes("airspace") ? "BLOCKED" : "CLEAR"}</strong></div>
            <div><span>局部避障</span><strong>{textValue(navigationState.reason, "standby")}</strong></div>
            <div><span>视觉标签</span><strong className={boolValue(docking.tag_visible) ? "good-text" : "muted-text"}>{boolValue(docking.tag_visible) ? "LOCK" : "SEARCH"}</strong></div>
            <div><span>捕获窗口</span><strong className={boolValue(docking.capture_ready) ? "good-text" : "muted-text"}>{boolValue(docking.capture_ready) ? "READY" : "OUTSIDE"}</strong></div>
            <div><span>光流状态</span><strong className={boolValue(opticalFlow.healthy) ? "good-text" : "muted-text"}>{textValue(opticalFlow.reason, "waiting")}</strong></div>
            <div><span>感知硬停止</span><strong className={boolValue(perception.hard_stop) ? "bad-text" : "good-text"}>{boolValue(perception.hard_stop) ? "ACTIVE" : "CLEAR"}</strong></div>
          </div>
          <div className="safety-actions">
            <button className="estop-button" disabled={!commandEnabled || systemEmergency} onClick={() => run("/api/safety/emergency-stop", {}, "系统急停已闭锁")}>紧急停止</button>
            <button disabled={!commandEnabled || !systemEmergency || armed || numberValue(snapshot.ugv?.speed_mps) > 0.02} onClick={() => run("/api/safety/reset", {}, "安全闭锁复位请求已提交")}>受保护复位</button>
          </div>
          <div className="fault-list">
            {systemFaults.length === 0 ? <span>当前无系统级故障</span> : systemFaults.slice(0, 5).map((fault) => <span key={textValue(fault.code)}><b>{textValue(fault.severity)}</b>{textValue(fault.code)} · {textValue(fault.summary)}</span>)}
          </div>
        </section>
      </section>

      <section className="manual-panel panel">
        <details>
          <summary><span><Settings2 size={16} />手动控制与工程调试</span><ChevronDown size={16} /></summary>
          <div className="manual-grid">
            <form onSubmit={submitUav} className="control-form">
              <header><Crosshair size={15} />UAV 目标（uav_odom）</header>
              <label>X<input type="number" step="0.1" value={uavGoal.x} onChange={(event) => setUavGoal({ ...uavGoal, x: Number(event.target.value) })} /></label>
              <label>Y<input type="number" step="0.1" value={uavGoal.y} onChange={(event) => setUavGoal({ ...uavGoal, y: Number(event.target.value) })} /></label>
              <label>Z<input type="number" min="0.8" max="8" step="0.1" value={uavGoal.z} onChange={(event) => setUavGoal({ ...uavGoal, z: Number(event.target.value) })} /></label>
              <button type="submit"><Send size={14} />发送</button>
            </form>
            <form onSubmit={submitUgv} className="control-form">
              <header><Navigation size={15} />UGV Nav2 目标（map）</header>
              <label>X<input type="number" step="0.1" value={ugvGoal.x} onChange={(event) => setUgvGoal({ ...ugvGoal, x: Number(event.target.value) })} /></label>
              <label>Y<input type="number" step="0.1" value={ugvGoal.y} onChange={(event) => setUgvGoal({ ...ugvGoal, y: Number(event.target.value) })} /></label>
              <label>Yaw<input type="number" step="0.1" value={ugvGoal.yaw} onChange={(event) => setUgvGoal({ ...ugvGoal, yaw: Number(event.target.value) })} /></label>
              <button type="submit"><Send size={14} />发送</button>
            </form>
            <div className="teleop-form">
              <header><Car size={15} />UGV 按住式点动</header>
              <p>{missionActive && !missionPaused ? "请先暂停协同任务" : "按住移动，松手立即发送零速"}</p>
              <div className="teleop-pad">
                <button className="teleop-up" disabled={!teleopAllowed} aria-label="无人车前进" onPointerDown={(event) => { event.preventDefault(); startTeleop(0.22, 0); }} onPointerUp={stopTeleop} onPointerCancel={stopTeleop} onPointerLeave={stopTeleop}>↑</button>
                <button className="teleop-left" disabled={!teleopAllowed} aria-label="无人车左转" onPointerDown={(event) => { event.preventDefault(); startTeleop(0.16, 0.45); }} onPointerUp={stopTeleop} onPointerCancel={stopTeleop} onPointerLeave={stopTeleop}>↶</button>
                <button className="teleop-stop" disabled={!commandEnabled} aria-label="无人车停车" onClick={stopTeleop}>■</button>
                <button className="teleop-right" disabled={!teleopAllowed} aria-label="无人车右转" onPointerDown={(event) => { event.preventDefault(); startTeleop(0.16, -0.45); }} onPointerUp={stopTeleop} onPointerCancel={stopTeleop} onPointerLeave={stopTeleop}>↷</button>
                <button className="teleop-down" disabled={!teleopAllowed} aria-label="无人车后退" onPointerDown={(event) => { event.preventDefault(); startTeleop(-0.16, 0); }} onPointerUp={stopTeleop} onPointerCancel={stopTeleop} onPointerLeave={stopTeleop}>↓</button>
              </div>
            </div>
            <div className="token-form">
              <header><LockKeyhole size={15} />操作者身份与令牌</header>
              <p>生产模式要求操作者 ID、唯一请求号和高熵令牌；令牌只保留在当前浏览器会话。</p>
              <input type="text" value={operatorId} placeholder="Operator ID" onChange={(event) => updateOperatorId(event.target.value)} />
              <input type="password" value={token} placeholder="X-Control-Token" onChange={(event) => updateToken(event.target.value)} />
              <span>{snapshot.gateway?.token_required ? "网关要求令牌" : "当前网关未要求令牌"}</span>
            </div>
          </div>
        </details>
      </section>

      <footer className="footer-bar">
        <span><Gauge size={13} />网关运行 {formatNumber(snapshot.gateway?.uptime_s, 0, "s")}</span>
        <span><Radio size={13} />SSE / ROS 2</span>
        <span>{systemEmergency ? "EMERGENCY STOP" : systemReady ? (commandEnabled ? "CONTROL READY" : "READ ONLY") : "MOTION INHIBITED"}</span>
      </footer>

      {confirmAction && (
        <div className="modal-backdrop" role="presentation" onMouseDown={(event) => { if (event.target === event.currentTarget) setConfirmAction(null); }}>
          <div className="confirm-dialog" role="dialog" aria-modal="true" aria-labelledby="confirm-title">
            <CircleAlert size={28} />
            <h2 id="confirm-title">确认{confirmAction === "abort" ? "中止任务" : "重置仿真"}？</h2>
            <p>{confirmAction === "abort" ? "将取消 Nav2 目标、关闭运动门控，并在 SITL 空中状态请求降落；已锁止的对接机构不会自动释放。" : "Gazebo 世界会回到初始状态，当前任务和车辆位置将失效。"}</p>
            <div><button onClick={() => setConfirmAction(null)}>取消</button><button className="danger" onClick={() => { run(confirmAction === "abort" ? "/api/mission/abort" : "/api/sim/reset", {}, confirmAction === "abort" ? "任务已中止" : "Gazebo 已重置"); setConfirmAction(null); }}>确认执行</button></div>
          </div>
        </div>
      )}
    </main>
  );
}

export default App;
