"""Runtime acceptance monitor for the complete cooperative SIL mission."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Iterable, Mapping
from urllib.error import URLError
from urllib.parse import urlsplit, urlunsplit
from urllib.request import urlopen

from .mission_logic import MissionState, mission_state_allows_ugv_motion


EXPECTED_SEQUENCE = tuple(
    state.value
    for state in MissionState
    if state not in (MissionState.IDLE, MissionState.ABORTED, MissionState.FAULT)
)

# HTTP status is a snapshot, so a ready autopilot can make this guard state
# shorter than one polling interval.  Its execution is proven by the final
# transition counter and the supervisor's durable mission-transition journal.
REQUIRED_OBSERVED_SEQUENCE = tuple(
    state
    for state in EXPECTED_SEQUENCE
    if state
    not in {
        MissionState.RELEASE_REMOTE_DOCK.value,
        MissionState.WAIT_AUTOPILOT.value,
    }
)

REQUIRED_UAV_SENSORS = frozenset(
    {
        "downward_camera",
        "downward_tof",
        "stereo_left",
        "stereo_right",
        "stereo_depth",
        "lidar3d_scan",
        "lidar3d",
    }
)

REQUIRED_CAMERA_STREAMS = frozenset(
    {"stereo_left", "stereo_right", "landing", "downward", "ugv"}
)


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def ordered_subsequence(observed: Iterable[str], expected: Iterable[str]) -> bool:
    iterator = iter(observed)
    return all(any(item == target for item in iterator) for target in expected)


def planar_path_length(points: Any) -> float | None:
    if not isinstance(points, list) or len(points) < 2:
        return None
    total = 0.0
    try:
        for previous, current in zip(points, points[1:]):
            total += math.hypot(
                float(current[0]) - float(previous[0]),
                float(current[1]) - float(previous[1]),
            )
    except (IndexError, TypeError, ValueError):
        return None
    return total


def camera_stream_urls(status_url: str) -> dict[str, str]:
    """Build camera URLs on the same gateway as the status endpoint."""

    parsed = urlsplit(status_url)
    return {
        name: urlunsplit(
            (parsed.scheme, parsed.netloc, f"/api/camera/{name}.jpg", "", "")
        )
        for name in sorted(REQUIRED_CAMERA_STREAMS)
    }


def probe_camera_streams(
    urls: Mapping[str, str], timeout_s: float
) -> dict[str, Mapping[str, Any]]:
    """Keep lazy streams active and prove every JPEG endpoint concurrently."""

    def probe(name: str, url: str) -> tuple[str, Mapping[str, Any]]:
        try:
            with urlopen(url, timeout=timeout_s) as response:  # nosec B310: operator URL
                payload = response.read()
                result: Mapping[str, Any] = {
                    "ok": 200 <= int(response.status) < 300 and bool(payload),
                    "status": int(response.status),
                    "bytes": len(payload),
                }
        except (OSError, URLError, ValueError) as error:
            result = {"ok": False, "error": str(error)}
        return name, result

    if not urls:
        return {}
    results: dict[str, Mapping[str, Any]] = {}
    with ThreadPoolExecutor(
        max_workers=min(len(urls), 8), thread_name_prefix="camera-probe"
    ) as executor:
        futures = [executor.submit(probe, name, url) for name, url in urls.items()]
        for future in as_completed(futures):
            name, result = future.result()
            results[name] = result
    return {name: results[name] for name in urls}


def ride_path_is_current(
    mission: Mapping[str, Any], paths: Mapping[str, Any]
) -> bool:
    """Return whether the displayed Nav2 plan belongs to the active ride goal."""

    try:
        path_transition = int(paths.get("ugv_global_for_transition"))
        mission_transition = int(mission.get("transitions"))
    except (TypeError, ValueError):
        return False
    return (
        str(paths.get("ugv_global_for_state", ""))
        == MissionState.RIDE_AND_DECELERATE.value
        and path_transition == mission_transition
    )


def violation_diagnostics(
    snapshot: Mapping[str, Any], code: str
) -> Mapping[str, Any]:
    """Return compact evidence that identifies the source of a violation."""

    mission = _mapping(snapshot.get("mission"))
    if code == "OPERATIONS_CAMERA_STREAM_INCOMPLETE":
        return {
            "mission_state": mission.get("state"),
            "cameras": _mapping(snapshot.get("cameras")),
        }
    if code in {
        "RIDE_PATH_UNAVAILABLE",
        "RIDE_PATH_STALE",
        "RIDE_PATH_NONHOLONOMIC_DETOUR",
    }:
        paths = _mapping(snapshot.get("paths"))
        return {
            "mission_state": mission.get("state"),
            "mission_transition": mission.get("transitions"),
            "ugv_goal_status": mission.get("ugv_goal_status"),
            "ride_remaining_distance_m": mission.get("ride_remaining_distance_m"),
            "path_length_m": planar_path_length(paths.get("ugv_global")),
            "path_age_s": paths.get("ugv_global_age_s"),
            "path_state": paths.get("ugv_global_for_state"),
            "path_transition": paths.get("ugv_global_for_transition"),
            "path_goal_status": paths.get("ugv_global_for_goal_status"),
        }
    return {"mission_state": mission.get("state")}


def snapshot_violations(snapshot: Mapping[str, Any]) -> list[str]:
    """Return stable acceptance codes for safety-relevant runtime violations."""

    mission = _mapping(snapshot.get("mission"))
    system = _mapping(snapshot.get("system"))
    mavlink = _mapping(snapshot.get("mavlink"))
    perception = _mapping(snapshot.get("perception"))
    control_mux = _mapping(snapshot.get("ugv_control_mux"))
    docking = _mapping(snapshot.get("docking"))
    ugv = _mapping(snapshot.get("ugv"))
    cameras = _mapping(snapshot.get("cameras"))
    paths = _mapping(snapshot.get("paths"))
    state_name = str(mission.get("state", ""))
    active = bool(mission.get("active", False))
    violations: list[str] = []

    if active:
        plan = _mapping(mission.get("mission_plan"))
        plan_id = str(plan.get("id", "")).strip()
        if (
            not bool(plan.get("commissioned", False))
            or not plan_id.startswith("SIL-")
            or len(plan_id) < 12
        ):
            violations.append("MISSION_PLAN_IDENTITY_INVALID")
        if bool(system.get("emergency_stop", False)):
            violations.append("SYSTEM_ESTOP_ACTIVE")
        if bool(system.get("latched", False)):
            violations.append("SYSTEM_SAFETY_LATCHED")
        critical_faults = [
            fault
            for fault in system.get("faults", [])
            if int(_mapping(fault).get("level") or 0) >= 2
        ]
        if critical_faults:
            violations.append("SYSTEM_CRITICAL_FAULT_PRESENT")
        if not bool(mavlink.get("connected", False)):
            violations.append("MAVLINK_DISCONNECTED")
        if not bool(mavlink.get("required_parameters_verified", False)):
            violations.append("FCU_PARAMETER_ATTESTATION_FAILED")
        stream_acknowledged = int(mavlink.get("telemetry_stream_acknowledged") or 0)
        stream_required = int(mavlink.get("telemetry_stream_required") or 0)
        if (
            not bool(mavlink.get("telemetry_streams_configured", False))
            or stream_required <= 0
            or stream_acknowledged != stream_required
        ):
            violations.append("MAVLINK_STREAM_CONFIGURATION_INCOMPLETE")
        if not bool(perception.get("healthy", False)):
            violations.append("UAV_PERCEPTION_UNHEALTHY")

        sensors = _mapping(perception.get("sensors"))
        missing = REQUIRED_UAV_SENSORS.difference(sensors)
        if missing:
            violations.append("UAV_SENSOR_SUITE_INCOMPLETE")
        elif any(not bool(_mapping(sensors[name]).get("healthy", False)) for name in REQUIRED_UAV_SENSORS):
            violations.append("UAV_SENSOR_UNHEALTHY")
        if REQUIRED_CAMERA_STREAMS.difference(cameras) or any(
            not bool(_mapping(cameras[name]).get("ready", False))
            for name in REQUIRED_CAMERA_STREAMS.intersection(cameras)
        ):
            violations.append("OPERATIONS_CAMERA_STREAM_INCOMPLETE")

        try:
            state = MissionState(state_name)
        except ValueError:
            violations.append("MISSION_STATE_UNKNOWN")
        else:
            motion_or_flight_active = bool(mavlink.get("armed", False)) or (
                mission_state_allows_ugv_motion(state)
            )
            if motion_or_flight_active and not bool(system.get("ready", False)):
                violations.append("SYSTEM_NOT_READY_DURING_MOTION")
            if not mission_state_allows_ugv_motion(state) and (
                bool(mission.get("ugv_safety_gate_open", False))
                or bool(control_mux.get("gate_open", False))
            ):
                violations.append("UGV_GATE_OPEN_IN_NON_DRIVING_STATE")
            if state == MissionState.RIDE_AND_DECELERATE:
                goal_status = str(mission.get("ugv_goal_status", ""))
                if goal_status in ("executing", "succeeded"):
                    path_length = planar_path_length(paths.get("ugv_global"))
                    remaining = float(mission.get("ride_remaining_distance_m") or 0.0)
                    if not ride_path_is_current(mission, paths):
                        violations.append("RIDE_PATH_STALE")
                    elif path_length is None:
                        violations.append("RIDE_PATH_UNAVAILABLE")
                    elif path_length > max(remaining + 1.0, remaining * 1.5):
                        violations.append("RIDE_PATH_NONHOLONOMIC_DETOUR")

    if state_name == MissionState.COMPLETE.value:
        if bool(mavlink.get("armed", True)):
            violations.append("FINAL_UAV_ARMED")
        if mavlink.get("landed") is not True:
            violations.append("FINAL_UAV_NOT_LANDED")
        if docking.get("active") is True or mission.get("dock_detached") is not False:
            violations.append("FINAL_DOCK_NOT_ATTACHED")
        if abs(float(ugv.get("speed_mps") or 0.0)) > 0.03:
            violations.append("FINAL_UGV_NOT_STOPPED")
        if bool(mission.get("ugv_safety_gate_open", False)) or bool(
            control_mux.get("gate_open", False)
        ):
            violations.append("FINAL_UGV_GATE_OPEN")

    return sorted(set(violations))


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class EvidenceWriter:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, record: Mapping[str, Any]) -> None:
        payload = {"recorded_at": _utc_now(), **dict(record)}
        with self.path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())


def _fetch_json(url: str, timeout_s: float) -> Mapping[str, Any]:
    with urlopen(url, timeout=timeout_s) as response:  # nosec B310: operator URL
        payload = json.load(response)
    if not isinstance(payload, Mapping):
        raise ValueError("status endpoint did not return a JSON object")
    return payload


def _summary_snapshot(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    mission = _mapping(snapshot.get("mission"))
    mavlink = _mapping(snapshot.get("mavlink"))
    system = _mapping(snapshot.get("system"))
    perception = _mapping(snapshot.get("perception"))
    ugv = _mapping(snapshot.get("ugv"))
    cameras = _mapping(snapshot.get("cameras"))
    paths = _mapping(snapshot.get("paths"))
    return {
        "state": mission.get("state"),
        "transitions": mission.get("transitions"),
        "reason": mission.get("reason"),
        "system_state": system.get("state"),
        "faults": system.get("faults"),
        "armed": mavlink.get("armed"),
        "landed": mavlink.get("landed"),
        "mode": mavlink.get("mode"),
        "parameter_attestation": mavlink.get("required_parameters_verified"),
        "telemetry_streams": [
            mavlink.get("telemetry_stream_acknowledged"),
            mavlink.get("telemetry_stream_required"),
        ],
        "sensor_health": {
            name: bool(_mapping(value).get("healthy", False))
            for name, value in _mapping(perception.get("sensors")).items()
        },
        "ugv_speed_mps": ugv.get("speed_mps"),
        "ugv_gate_open": mission.get("ugv_safety_gate_open"),
        "dock_detached": mission.get("dock_detached"),
        "camera_streams": {
            name: {
                "active": _mapping(value).get("active"),
                "ready": _mapping(value).get("ready"),
                "age_s": _mapping(value).get("age_s"),
                "source_age_s": _mapping(value).get("source_age_s"),
            }
            for name, value in cameras.items()
        },
        "ride_path": {
            "current": ride_path_is_current(mission, paths),
            "length_m": planar_path_length(paths.get("ugv_global")),
            "age_s": paths.get("ugv_global_age_s"),
            "state": paths.get("ugv_global_for_state"),
            "transition": paths.get("ugv_global_for_transition"),
            "goal_status": paths.get("ugv_global_for_goal_status"),
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765/api/status")
    parser.add_argument("--timeout-s", type=float, default=5400.0)
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--request-timeout-s", type=float, default=3.0)
    parser.add_argument("--startup-grace-s", type=float, default=180.0)
    parser.add_argument("--transient-grace-s", type=float, default=1.5)
    parser.add_argument("--sample-every-s", type=float, default=15.0)
    parser.add_argument("--camera-probe-every-s", type=float, default=2.0)
    parser.add_argument("--camera-probe-timeout-s", type=float, default=1.0)
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--summary", required=True)
    args = parser.parse_args(argv)

    writer = EvidenceWriter(args.evidence)
    started = time.monotonic()
    deadline = started + max(1.0, args.timeout_s)
    next_sample = 0.0
    next_camera_probe = 0.0
    camera_urls = camera_stream_urls(args.url)
    last_camera_probe: Mapping[str, Any] = {}
    observed: list[str] = []
    last_state = ""
    violation_since: dict[str, float] = {}
    confirmed_violations: set[str] = set()
    connection_seen = False
    final_snapshot: Mapping[str, Any] = {}
    terminal_reason = "timeout"

    writer.append(
        {
            "event": "acceptance_started",
            "url": args.url,
            "camera_probe_every_s": max(0.0, args.camera_probe_every_s),
            "required_camera_streams": sorted(REQUIRED_CAMERA_STREAMS),
        }
    )
    while time.monotonic() < deadline:
        now = time.monotonic()
        try:
            snapshot = _fetch_json(args.url, max(0.1, args.request_timeout_s))
            connection_seen = True
            final_snapshot = snapshot
        except (OSError, URLError, ValueError, json.JSONDecodeError) as error:
            if now - started > args.startup_grace_s:
                confirmed_violations.add("STATUS_ENDPOINT_UNAVAILABLE")
                terminal_reason = f"status_endpoint_unavailable:{error}"
                break
            time.sleep(max(0.05, args.poll_s))
            continue

        mission = _mapping(snapshot.get("mission"))
        state = str(mission.get("state", ""))
        if args.camera_probe_every_s > 0.0 and now >= next_camera_probe:
            last_camera_probe = probe_camera_streams(
                camera_urls, max(0.1, args.camera_probe_timeout_s)
            )
            next_camera_probe = now + max(0.5, args.camera_probe_every_s)
        if state and state != last_state:
            observed.append(state)
            writer.append(
                {
                    "event": "mission_state",
                    "state": state,
                    "snapshot": _summary_snapshot(snapshot),
                }
            )
            last_state = state

        current = set(snapshot_violations(snapshot))
        for code in current:
            violation_since.setdefault(code, now)
            if now - violation_since[code] >= args.transient_grace_s:
                if code not in confirmed_violations:
                    writer.append(
                        {
                            "event": "violation",
                            "code": code,
                            "diagnostics": violation_diagnostics(snapshot, code),
                        }
                    )
                confirmed_violations.add(code)
        for code in set(violation_since).difference(current):
            violation_since.pop(code, None)

        if now >= next_sample:
            writer.append(
                {
                    "event": "periodic_sample",
                    "snapshot": _summary_snapshot(snapshot),
                    "camera_probe": last_camera_probe,
                    "violations": sorted(current),
                }
            )
            next_sample = now + max(1.0, args.sample_every_s)

        if state in (MissionState.FAULT.value, MissionState.ABORTED.value):
            terminal_reason = state.lower()
            break
        if state == MissionState.COMPLETE.value:
            terminal_reason = "complete"
            break
        time.sleep(max(0.05, args.poll_s))

    observable_sequence = ordered_subsequence(observed, REQUIRED_OBSERVED_SEQUENCE)
    final_mission = _mapping(final_snapshot.get("mission"))
    transition_count = int(final_mission.get("transitions") or 0)
    full_sequence = observable_sequence and transition_count >= len(EXPECTED_SEQUENCE)
    if not connection_seen:
        confirmed_violations.add("STATUS_ENDPOINT_NEVER_AVAILABLE")
    if terminal_reason != "complete":
        confirmed_violations.add("MISSION_DID_NOT_COMPLETE")
    if not full_sequence:
        confirmed_violations.add("MISSION_SEQUENCE_INCOMPLETE")
    confirmed_violations.update(snapshot_violations(final_snapshot))

    summary = {
        "schema_version": "1.0",
        "finished_at": _utc_now(),
        "passed": not confirmed_violations,
        "terminal_reason": terminal_reason,
        "duration_s": round(time.monotonic() - started, 3),
        "expected_sequence": list(EXPECTED_SEQUENCE),
        "required_observed_sequence": list(REQUIRED_OBSERVED_SEQUENCE),
        "observed_sequence": observed,
        "transition_count": transition_count,
        "full_sequence": full_sequence,
        "violations": sorted(confirmed_violations),
        "final_snapshot": _summary_snapshot(final_snapshot),
        "evidence_path": str(writer.path),
    }
    summary_path = Path(args.summary).expanduser().resolve()
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    writer.append({"event": "acceptance_finished", "summary": summary})
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
