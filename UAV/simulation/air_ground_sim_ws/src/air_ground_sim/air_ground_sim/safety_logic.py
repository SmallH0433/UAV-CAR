"""Pure system-safety evaluation used by the runtime supervisor and tests."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping, MutableMapping, Optional, Sequence


class Severity(IntEnum):
    OK = 0
    WARN = 1
    ERROR = 2
    CRITICAL = 3


@dataclass(frozen=True)
class Fault:
    code: str
    severity: Severity
    source: str
    summary: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "code": self.code,
            "severity": self.severity.name,
            "level": int(self.severity),
            "source": self.source,
            "summary": self.summary,
        }


@dataclass(frozen=True)
class HealthEvaluation:
    ready: bool
    state: str
    mission_active: bool
    airborne: bool
    faults: tuple[Fault, ...]

    @property
    def has_critical(self) -> bool:
        return any(fault.severity >= Severity.CRITICAL for fault in self.faults)


def update_critical_fault_timers(
    faults: Sequence[Fault],
    first_seen_s: MutableMapping[str, float],
    *,
    now_s: float,
    hold_s: float,
    immediate_codes: frozenset[str] = frozenset(),
) -> tuple[str, ...]:
    """Return critical codes mature enough to latch and reset cleared timers."""

    current = {
        fault.code for fault in faults if fault.severity >= Severity.CRITICAL
    }
    for code in tuple(first_seen_s):
        if code not in current:
            del first_seen_s[code]
    matured = []
    duration = max(float(hold_s), 0.0)
    for code in sorted(current):
        first_seen_s.setdefault(code, float(now_s))
        if code in immediate_codes or float(now_s) - first_seen_s[code] >= duration:
            matured.append(code)
    return tuple(matured)


DEFAULT_REQUIRED_SOURCES = (
    "mission",
    "mavlink",
    "perception",
    "command_mux",
    "ugv_control_mux",
    "chassis_adapter",
    "ugv_gateway",
)


def _number(value: Any) -> Optional[float]:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted


def evaluate_system_health(
    *,
    statuses: Mapping[str, Mapping[str, Any]],
    ages_s: Mapping[str, Optional[float]],
    source_timeout_s: float,
    external_estop: bool,
    operator_estop: bool,
    ugv_speed_mps: float,
    required_sources: tuple[str, ...] = DEFAULT_REQUIRED_SOURCES,
    low_battery_pct: float = 20.0,
    critical_battery_pct: float = 10.0,
    stopped_speed_mps: float = 0.03,
    moving_capture_armed_timeout_s: float = 8.0,
    moving_capture_max_altitude_m: float = 0.50,
) -> HealthEvaluation:
    """Evaluate readiness and faults without mutating latch state.

    Missing data prevents readiness. It becomes critical only when a vehicle is
    already moving/airborne or the mission is active, avoiding a startup latch
    while retaining fail-closed behavior during operation.
    """

    mission = statuses.get("mission", {})
    mavlink = statuses.get("mavlink", {})
    perception = statuses.get("perception", {})
    chassis = statuses.get("chassis_adapter", {})
    ugv_gateway = statuses.get("ugv_gateway", {})
    docking_gateway = statuses.get("docking_gateway", {})
    mission_active = bool(mission.get("active", False))
    mission_state = str(mission.get("state", "UNKNOWN"))
    altitude = _number(mavlink.get("relative_alt_m")) or 0.0
    armed = bool(mavlink.get("armed", False))
    # Arming on the pad is not equivalent to flight. Treat the aircraft as
    # airborne only when altitude confirms it, or when an armed autopilot says
    # it is not landed. If the landed field is missing while armed we stay
    # conservative and assume flight.
    landed = mavlink.get("landed")
    airborne = altitude > 0.8 or (armed and landed is not True)
    ugv_moving = abs(float(ugv_speed_mps)) > max(float(stopped_speed_mps), 0.0)
    motion_present = airborne or ugv_moving
    faults: list[Fault] = []

    if external_estop:
        faults.append(Fault("EXTERNAL_ESTOP", Severity.CRITICAL, "safety", "Physical emergency stop is active"))
    if operator_estop:
        faults.append(Fault("OPERATOR_ESTOP", Severity.CRITICAL, "safety", "Operator emergency stop is latched"))
    if mission_state == "FAULT":
        reason = str(mission.get("reason", "unspecified mission fault")).strip()
        faults.append(
            Fault(
                "MISSION_FAULT",
                Severity.CRITICAL,
                "mission",
                f"Mission state machine faulted: {reason or 'unspecified mission fault'}",
            )
        )

    timeout = max(float(source_timeout_s), 0.05)
    all_sources_fresh = True
    for source in required_sources:
        age = ages_s.get(source)
        stale = age is None or float(age) > timeout
        if not stale:
            continue
        all_sources_fresh = False
        critical = motion_present or (mission_active and source in {"mission", "ugv_control_mux"})
        faults.append(
            Fault(
                f"{source.upper()}_STALE",
                Severity.CRITICAL if critical else Severity.WARN,
                source,
                "Status stream is missing" if age is None else f"Status age {float(age):.2f}s exceeds {timeout:.2f}s",
            )
        )

    connected = bool(mavlink.get("connected", False))
    flight_ready = bool(mavlink.get("flight_ready", mavlink.get("prearm_checks_passed", False)))
    if not connected:
        faults.append(
            Fault(
                "MAVLINK_DISCONNECTED",
                Severity.CRITICAL if airborne else Severity.WARN,
                "mavlink",
                "Flight-controller heartbeat is unavailable",
            )
        )
    if connected and mavlink.get("required_parameters_verified") is False:
        faults.append(
            Fault(
                "UAV_PARAMETER_ATTESTATION_FAILED",
                Severity.CRITICAL if airborne else Severity.ERROR,
                "mavlink",
                "Safety-critical flight-controller parameters are missing or do not match the commissioned policy",
            )
        )
    if connected and not flight_ready and not airborne:
        faults.append(Fault("UAV_PREFLIGHT_NOT_READY", Severity.WARN, "mavlink", "Autopilot pre-arm/position checks are not ready"))

    if airborne and not bool(perception.get("healthy", False)):
        faults.append(Fault("UAV_PERCEPTION_UNHEALTHY", Severity.CRITICAL, "perception", "Required obstacle sensors are unhealthy while airborne"))

    battery = _number(mavlink.get("battery_remaining_pct"))
    if battery is not None and battery <= float(critical_battery_pct):
        faults.append(Fault("UAV_BATTERY_CRITICAL", Severity.CRITICAL if airborne else Severity.ERROR, "mavlink", f"Battery remaining is {battery:.1f}%"))
    elif battery is not None and battery <= float(low_battery_pct):
        faults.append(Fault("UAV_BATTERY_LOW", Severity.ERROR if airborne else Severity.WARN, "mavlink", f"Battery remaining is {battery:.1f}%"))

    dock_detached = mission.get("dock_detached")
    if dock_detached is False and armed:
        # A physical capture can lag mission-state entry while LAND mode and
        # redundant contact interlocks settle.  Time the exceptional armed
        # window from positive latch feedback; a missing age fails closed.
        capture_elapsed = _number(mission.get("dock_attached_age_s"))
        moving_capture = (
            mission_state == "LATCH_MOVING"
            and str(mavlink.get("mode", "")).upper() == "LAND"
            and altitude <= max(float(moving_capture_max_altitude_m), 0.0)
            and capture_elapsed is not None
        )
        if moving_capture and capture_elapsed <= max(
            float(moving_capture_armed_timeout_s), 0.0
        ):
            faults.append(
                Fault(
                    "UAV_CONTROLLED_CAPTURE_DISARMING",
                    Severity.WARN,
                    "mission",
                    "Moving-platform capture is closed while LAND completes "
                    f"normal disarm ({capture_elapsed:.2f}s)",
                )
            )
        elif moving_capture:
            faults.append(
                Fault(
                    "UAV_CAPTURE_DISARM_TIMEOUT",
                    Severity.CRITICAL,
                    "mission",
                    "Moving-platform capture remained armed for "
                    f"{capture_elapsed:.2f}s, beyond the "
                    f"{max(float(moving_capture_armed_timeout_s), 0.0):.2f}s guarded timeout",
                )
            )
        else:
            faults.append(
                Fault(
                    "UAV_ARMED_WHILE_LATCHED",
                    Severity.CRITICAL,
                    "mission",
                    "Aircraft is armed while the physical docking latch is attached outside guarded capture",
                )
            )

    if bool(chassis.get("emergency_stop", False)) or bool(ugv_gateway.get("emergency_stop", False)):
        # This can be the expected echo of the supervisor's own latched stop.
        # Keep readiness closed without creating a self-sustaining critical loop.
        faults.append(Fault("UGV_EMERGENCY_PATH_ACTIVE", Severity.ERROR, "ugv", "A downstream ground-vehicle stop path is active"))

    docking_required = "docking_gateway" in required_sources
    if docking_required:
        docking_fault = str(docking_gateway.get("critical_fault", "")).strip()
        if docking_fault:
            faults.append(
                Fault(
                    docking_fault,
                    Severity.CRITICAL,
                    "docking_gateway",
                    "Physical docking mechanism reported a critical interlock fault",
                )
            )
        elif not bool(docking_gateway.get("healthy", False)):
            faults.append(
                Fault(
                    "DOCKING_HARDWARE_NOT_READY",
                    Severity.ERROR,
                    "docking_gateway",
                    "Physical docking feedback is not ready",
                )
            )

    preflight_components_enabled = (
        bool(chassis.get("enabled", False))
        and bool(ugv_gateway.get("enabled", False))
        and bool(statuses.get("command_mux", {}).get("enabled", False))
        and bool(statuses.get("ugv_control_mux", {}).get("enabled", False))
        and (
            not docking_required
            or bool(docking_gateway.get("enabled", False))
        )
    )
    has_error = any(fault.severity >= Severity.ERROR for fault in faults)
    ready = (
        all_sources_fresh
        and not has_error
        and connected
        and flight_ready
        and bool(perception.get("healthy", False))
        and preflight_components_enabled
        and not external_estop
        and not operator_estop
    )
    if any(fault.severity >= Severity.CRITICAL for fault in faults):
        state = "EMERGENCY_STOP"
    elif has_error:
        state = "NOT_READY"
    elif faults or not ready:
        state = "DEGRADED"
    else:
        state = "READY"
    return HealthEvaluation(ready, state, mission_active, airborne, tuple(faults))
