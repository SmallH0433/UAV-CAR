"""Independent fail-closed supervisor for the complete air-ground system."""

from __future__ import annotations

import json
import os
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .audit_journal import AuditJournal
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer
from .safety_logic import (
    DEFAULT_REQUIRED_SOURCES,
    Severity,
    evaluate_system_health,
    update_critical_fault_timers,
)


def _parse_json(message: String) -> dict:
    try:
        parsed = json.loads(message.data)
    except (json.JSONDecodeError, TypeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


class SystemSupervisor(Node):
    """Latch critical faults, broadcast a stop and preserve diagnostic evidence."""

    STATUS_TOPICS = {
        "mission": "/mission/status",
        "mavlink": "/uav/mavlink/status",
        "perception": "/uav/perception/status",
        "command_mux": "/uav/command_mux/status",
        "ugv_control_mux": "/ugv/control_mux/status",
        "chassis_adapter": "/ugv/chassis_adapter/status",
        "ugv_gateway": "/ugv/command_gateway/status",
        "docking_gateway": "/uav/dock/hardware_status",
    }

    def __init__(self) -> None:
        super().__init__("system_supervisor")
        self.declare_parameter("source_timeout_s", 2.0)
        self.declare_parameter("low_battery_pct", 20.0)
        self.declare_parameter("critical_battery_pct", 10.0)
        self.declare_parameter("stopped_speed_mps", 0.03)
        self.declare_parameter("required_sources_json", json.dumps(DEFAULT_REQUIRED_SOURCES))
        self.declare_parameter("external_estop_topic", "/safety/external_estop")
        self.declare_parameter("external_estop_required", True)
        self.declare_parameter("external_estop_timeout_s", 0.30)
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("event_log_path", "~/.local/state/air-ground/system-events.jsonl")
        self.declare_parameter("event_log_required", False)
        self.declare_parameter("event_log_max_bytes", 20 * 1024 * 1024)
        self.declare_parameter("auto_abort_mission", True)
        self.declare_parameter("critical_fault_hold_s", 0.75)
        self.declare_parameter("moving_capture_armed_timeout_s", 8.0)
        self.declare_parameter("moving_capture_max_altitude_m", 0.50)

        self.source_timeout = max(float(self.get_parameter("source_timeout_s").value), 0.1)
        self.low_battery = float(self.get_parameter("low_battery_pct").value)
        self.critical_battery = float(self.get_parameter("critical_battery_pct").value)
        self.stopped_speed = max(float(self.get_parameter("stopped_speed_mps").value), 0.0)
        self.auto_abort = bool(self.get_parameter("auto_abort_mission").value)
        self.critical_fault_hold = max(
            0.0, float(self.get_parameter("critical_fault_hold_s").value)
        )
        self.moving_capture_armed_timeout = max(
            0.0,
            float(self.get_parameter("moving_capture_armed_timeout_s").value),
        )
        self.moving_capture_max_altitude = max(
            0.0,
            float(self.get_parameter("moving_capture_max_altitude_m").value),
        )
        self.external_estop_required = bool(
            self.get_parameter("external_estop_required").value
        )
        self.external_estop_timeout = max(
            0.05, float(self.get_parameter("external_estop_timeout_s").value)
        )
        self.startup_grace = max(
            0.0, float(self.get_parameter("startup_grace_s").value)
        )
        try:
            required = json.loads(str(self.get_parameter("required_sources_json").value))
            if not isinstance(required, list) or not all(isinstance(item, str) for item in required):
                raise ValueError
            self.required_sources = tuple(required)
        except (json.JSONDecodeError, ValueError):
            raise ValueError("required_sources_json must be a JSON array of strings")

        event_path = os.environ.get(
            "AIR_GROUND_EVENT_LOG",
            str(self.get_parameter("event_log_path").value),
        )
        self.journal = AuditJournal(
            event_path,
            max_bytes=int(self.get_parameter("event_log_max_bytes").value),
            required=bool(self.get_parameter("event_log_required").value),
        )
        self.statuses: Dict[str, dict] = {}
        self.status_times: Dict[str, float] = {}
        self.external_estop = False
        self.external_estop_time = 0.0
        self.operator_estop = False
        self.latched = False
        self.ugv_speed = 0.0
        self.sequence = 0
        self.last_payload: dict = {}
        self.last_fault_signature = ()
        self.last_abort_request = 0.0
        self.critical_fault_first_seen: Dict[str, float] = {}
        self.boot_time = time.monotonic()

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.emergency_publisher = self.create_publisher(
            Bool, "/system/emergency_stop", latched_qos
        )
        self.status_publisher = self.create_publisher(String, "/system/health", latched_qos)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        for key, topic in self.STATUS_TOPICS.items():
            self.create_subscription(
                String,
                topic,
                lambda message, source=key: self.on_status(source, message),
                10,
            )
        self.create_subscription(
            Bool,
            str(self.get_parameter("external_estop_topic").value),
            self.on_external_estop,
            10,
        )
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ugv_odom, 20)
        self.create_subscription(String, "/mission/events", self.on_mission_event, 20)

        self.abort_client = self.create_client(Trigger, "/air_ground_mission/fault")
        self.estop_service = self.create_service(
            SetBool, "~/emergency_stop", self.on_operator_estop
        )
        self.reset_service = self.create_service(Trigger, "~/reset", self.on_reset)
        self.readiness_service = self.create_service(
            Trigger, "~/readiness", self.on_readiness
        )
        self.timer = create_steady_timer(self, 0.1, self.evaluate_and_publish)
        self.diagnostic_timer = create_steady_timer(
            self, 1.0, self.publish_diagnostics
        )
        self.journal.write({"event": "supervisor_started", "required_sources": self.required_sources})
        self.evaluate_and_publish()

    def on_status(self, source: str, message: String) -> None:
        self.statuses[source] = _parse_json(message)
        self.status_times[source] = time.monotonic()

    def on_external_estop(self, message: Bool) -> None:
        self.external_estop = bool(message.data)
        self.external_estop_time = time.monotonic()
        if self.external_estop:
            self.latched = True

    def on_ugv_odom(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        self.ugv_speed = (float(velocity.x) ** 2 + float(velocity.y) ** 2) ** 0.5

    def on_mission_event(self, message: String) -> None:
        self.journal.write({"event": "mission_transition", "details": _parse_json(message)})

    def _ages(self, now: float) -> dict[str, float | None]:
        return {
            source: None if source not in self.status_times else now - self.status_times[source]
            for source in self.required_sources
        }

    def _evaluate(self, *, operator_estop: bool | None = None):
        now = time.monotonic()
        channel_healthy = (
            not self.external_estop_required
            or now - self.boot_time <= self.startup_grace
            or (
                self.external_estop_time > 0.0
                and now - self.external_estop_time <= self.external_estop_timeout
            )
        )
        return evaluate_system_health(
            statuses=self.statuses,
            ages_s=self._ages(now),
            source_timeout_s=self.source_timeout,
            external_estop=self.external_estop or not channel_healthy,
            operator_estop=self.operator_estop if operator_estop is None else operator_estop,
            ugv_speed_mps=self.ugv_speed,
            required_sources=self.required_sources,
            low_battery_pct=self.low_battery,
            critical_battery_pct=self.critical_battery,
            stopped_speed_mps=self.stopped_speed,
            moving_capture_armed_timeout_s=self.moving_capture_armed_timeout,
            moving_capture_max_altitude_m=self.moving_capture_max_altitude,
        )

    def on_operator_estop(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            response.success = False
            response.message = "Emergency stop is latched; use /system_supervisor/reset after making the system safe"
            return response
        self.operator_estop = True
        self.latched = True
        self.journal.write({"event": "operator_emergency_stop"})
        self.evaluate_and_publish()
        response.success = True
        response.message = "Emergency stop latched"
        return response

    def on_reset(self, _request: Trigger.Request, response: Trigger.Response):
        evaluation = self._evaluate(operator_estop=False)
        mavlink = self.statuses.get("mavlink", {})
        mission = self.statuses.get("mission", {})
        blocking = [
            fault
            for fault in evaluation.faults
            if fault.severity >= Severity.CRITICAL
            and fault.code != "UGV_EMERGENCY_PATH_ACTIVE"
        ]
        external_age = (
            None
            if self.external_estop_time == 0.0
            else time.monotonic() - self.external_estop_time
        )
        external_channel_healthy = (
            not self.external_estop_required
            or (
                external_age is not None
                and external_age <= self.external_estop_timeout
            )
        )
        if self.external_estop or not external_channel_healthy:
            response.success = False
            response.message = "Physical emergency-stop channel is active or stale"
        elif bool(mavlink.get("armed", False)):
            response.success = False
            response.message = "Reset rejected while aircraft is armed"
        elif abs(self.ugv_speed) > self.stopped_speed:
            response.success = False
            response.message = f"Reset rejected while UGV speed is {self.ugv_speed:.3f} m/s"
        elif bool(mission.get("active", False)):
            response.success = False
            response.message = "Reset rejected while mission is active"
        elif blocking:
            response.success = False
            response.message = "Reset rejected: " + ", ".join(fault.code for fault in blocking)
        else:
            self.operator_estop = False
            self.latched = False
            self.journal.write({"event": "safety_reset"})
            self.evaluate_and_publish()
            response.success = True
            response.message = "Safety latch reset; readiness must still be re-established"
        return response

    def on_readiness(self, _request: Trigger.Request, response: Trigger.Response):
        evaluation = self._evaluate()
        response.success = bool(evaluation.ready and not self.latched)
        response.message = (
            "System ready"
            if response.success
            else "Not ready: " + ", ".join(fault.code for fault in evaluation.faults)
        )
        return response

    def _request_abort(self, now: float) -> None:
        if not self.auto_abort or now - self.last_abort_request < 2.0:
            return
        if not self.abort_client.service_is_ready():
            return
        self.abort_client.call_async(Trigger.Request())
        self.last_abort_request = now

    def evaluate_and_publish(self) -> None:
        now = time.monotonic()
        evaluation = self._evaluate()
        matured_critical = update_critical_fault_timers(
            evaluation.faults,
            self.critical_fault_first_seen,
            now_s=now,
            hold_s=self.critical_fault_hold,
            immediate_codes=frozenset(
                {
                    "EXTERNAL_ESTOP",
                    "OPERATOR_ESTOP",
                    "UAV_ARMED_WHILE_LATCHED",
                    "UAV_CAPTURE_DISARM_TIMEOUT",
                }
            ),
        )
        if matured_critical:
            self.latched = True
        emergency = bool(self.latched or self.external_estop or self.operator_estop)
        external_age = (
            None
            if self.external_estop_time == 0.0
            else now - self.external_estop_time
        )
        external_channel_healthy = (
            not self.external_estop_required
            or now - self.boot_time <= self.startup_grace
            or (
                external_age is not None
                and external_age <= self.external_estop_timeout
            )
        )
        emergency = bool(emergency or not external_channel_healthy)
        if emergency and evaluation.mission_active:
            self._request_abort(now)

        faults = []
        for fault in evaluation.faults:
            rendered = fault.as_dict()
            if fault.severity >= Severity.CRITICAL:
                rendered["active_for_s"] = round(
                    max(0.0, now - self.critical_fault_first_seen.get(fault.code, now)), 3
                )
                rendered["latch_confirmed"] = fault.code in matured_critical
            faults.append(rendered)
        if self.latched and not any(fault["code"] in {"EXTERNAL_ESTOP", "OPERATOR_ESTOP"} for fault in faults):
            faults.append(
                {
                    "code": "SAFETY_LATCHED",
                    "severity": "CRITICAL",
                    "level": int(Severity.CRITICAL),
                    "source": "supervisor",
                    "summary": "A prior critical fault remains latched until a guarded reset",
                }
            )
        signature = tuple((fault["code"], fault["severity"]) for fault in faults)
        if signature != self.last_fault_signature:
            self.journal.write(
                {
                    "event": "health_changed",
                    "emergency_stop": emergency,
                    "ready": bool(evaluation.ready and not emergency),
                    "faults": faults,
                }
            )
            self.last_fault_signature = signature

        self.sequence += 1
        ages = self._ages(now)
        self.last_payload = {
            "schema_version": "1.0",
            "sequence": self.sequence,
            "state": (
                "EMERGENCY_STOP"
                if emergency
                else "FAULT_PENDING"
                if evaluation.has_critical
                else evaluation.state
            ),
            "ready": bool(evaluation.ready and not emergency),
            "emergency_stop": emergency,
            "latched": self.latched,
            "external_estop": self.external_estop,
            "external_estop_channel": {
                "required": self.external_estop_required,
                "healthy": external_channel_healthy,
                "age_s": None if external_age is None else round(external_age, 3),
                "timeout_s": self.external_estop_timeout,
            },
            "operator_estop": self.operator_estop,
            "mission_active": evaluation.mission_active,
            "airborne": evaluation.airborne,
            "ugv_speed_mps": round(self.ugv_speed, 4),
            "source_ages_s": {
                key: None if age is None else round(age, 3) for key, age in ages.items()
            },
            "faults": faults,
            "event_journal": {
                "available": self.journal.available,
                "last_error": self.journal.last_error,
            },
            "timestamp_unix_ms": int(time.time() * 1000),
        }
        stop = Bool()
        stop.data = emergency
        self.emergency_publisher.publish(stop)
        status = String()
        status.data = json.dumps(self.last_payload, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(status)

    def publish_diagnostics(self) -> None:
        payload = self.last_payload
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        overall = DiagnosticStatus()
        overall.name = "air_ground/system_supervisor"
        overall.hardware_id = "air_ground_system"
        if payload.get("emergency_stop"):
            overall.level = DiagnosticStatus.ERROR
        elif payload.get("ready"):
            overall.level = DiagnosticStatus.OK
        else:
            overall.level = DiagnosticStatus.WARN
        overall.message = str(payload.get("state", "STARTING"))
        overall.values = [
            KeyValue(key="ready", value=str(bool(payload.get("ready"))).lower()),
            KeyValue(key="latched", value=str(bool(payload.get("latched"))).lower()),
            KeyValue(key="fault_count", value=str(len(payload.get("faults", [])))),
            KeyValue(key="ugv_speed_mps", value=str(payload.get("ugv_speed_mps", 0.0))),
        ]
        message.status.append(overall)
        self.diagnostics_publisher.publish(message)

    def destroy_node(self):
        if rclpy.ok():
            stop = Bool()
            stop.data = True
            run_shutdown_action(lambda: self.emergency_publisher.publish(stop))
        self.journal.write({"event": "supervisor_shutdown", "emergency_stop": True})
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
